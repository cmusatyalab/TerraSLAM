/**
 * @file orb_slam3_interface.cpp
 * @brief Implementation of the ORBSLAM3Interface class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"

namespace ORB_SLAM3_Wrapper
{
    ORBSLAM3Interface::ORBSLAM3Interface(const std::string &strVocFile,
                                         const std::string &strSettingsFile,
                                         ORB_SLAM3::System::eSensor sensor,
                                         bool bUseViewer,
                                         bool rosViz,
                                         double robotX,
                                         double robotY,
                                         std::string globalFrame,
                                         std::string odomFrame,
                                         std::string robotFrame)
        : strVocFile_(strVocFile),
          strSettingsFile_(strSettingsFile),
          sensor_(sensor),
          bUseViewer_(bUseViewer),
          rosViz_(rosViz),
          robotX_(robotX),
          robotY_(robotY),
          globalFrame_(globalFrame),
          odomFrame_(odomFrame),
          robotFrame_(robotFrame)
    {
        std::cout << "Interface constructor started" << endl;
        // std::cout << robotX_ << robotY_ << endl;
        mSLAM_ = std::make_shared<ORB_SLAM3::System>(strVocFile_, strSettingsFile_, sensor_, bUseViewer_);
        typeConversions_ = std::make_shared<WrapperTypeConversions>();
        std::cout << "Interface constructor complete" << endl;
    }

    ORBSLAM3Interface::~ORBSLAM3Interface()
    {
        std::cout << "Interface destructor" << endl;
        mSLAM_->Shutdown();
        mSLAM_.reset();
        typeConversions_.reset();
        mapReferencePoses_.clear();
        allKFs_.clear();
        
        // Close CSV file
        if (csvFile_.is_open()) {
            csvFile_.close();
        }
    }

    std::unordered_map<long unsigned int, ORB_SLAM3::KeyFrame *> ORBSLAM3Interface::makeKFIdPair(std::vector<ORB_SLAM3::Map *> mapsList)
    {
        std::unordered_map<long unsigned int, ORB_SLAM3::KeyFrame *> mpIdKFs;
        for (ORB_SLAM3::Map *pMap_i : mapsList)
        {
            std::vector<ORB_SLAM3::KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

            for (ORB_SLAM3::KeyFrame *pKF_j_Mi : vpKFs_Mi)
            {
                mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
            }
        }

        return mpIdKFs;
    }

    void ORBSLAM3Interface::calculateReferencePoses()
    {
        struct compareInitKFid
        {
            inline bool operator()(ORB_SLAM3::Map *elem1, ORB_SLAM3::Map *elem2)
            {
                return elem1->GetInitKFid() < elem2->GetInitKFid();
            }
        };

        mapReferencesMutex_.lock();
        mapReferencePoses_.clear();
        std::vector<ORB_SLAM3::Map *> mapsList = orbAtlas_->GetAllMaps();
        // sort the map array in init kf id order.
        std::sort(mapsList.begin(), mapsList.end(), compareInitKFid());
        allKFs_ = makeKFIdPair(mapsList);
        // std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
        for (size_t c = 0; c < mapsList.size(); c++)
        {
            // std::cout << "Map ID: " << mapsList[c]->GetId() << " origin kf ID: " << mapsList[c]->GetOriginKF()->mnId << " init kf id: " << mapsList[c]->GetInitKFid() << " max kf id: " << mapsList[c]->GetMaxKFid() << std::endl;
            if (c == 0)
            {
                auto poseWithoutOffset = typeConversions_->se3ToAffine(mapsList[c]->GetOriginKF()->GetPose());
                auto poseOffset = Eigen::Affine3d(
                    Eigen::Translation3d(robotX_, robotY_, 0.0) *
                    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
                mapReferencePoses_[mapsList[c]] = poseOffset * poseWithoutOffset;
            }
            else
            {
                if (static_cast<int64_t>(mapsList[c]->GetInitKFid()) - 1 < 0)
                {
                    throw std::runtime_error("The init KF id - 1 is lesser than 0. This should not happen");
                }
                else if (allKFs_.count(mapsList[c]->GetInitKFid() - 1) == 0)
                {
                    throw std::runtime_error("The KF was not found in allKFs_. This should not happen");
                }
                else if (mapReferencePoses_.count(allKFs_[mapsList[c]->GetInitKFid() - 1]->GetMap()) == 0)
                {
                    throw std::runtime_error("The parent map pose for this map ID does not exist. This should not happen.");
                }
                auto parentMapORBPose = allKFs_[mapsList[c]->GetInitKFid() - 1]->GetPose();
                mapReferencePoses_[mapsList[c]] = typeConversions_->transformPoseWithReference<Eigen::Affine3d>(mapReferencePoses_[allKFs_[mapsList[c]->GetInitKFid() - 1]->GetMap()], parentMapORBPose);
            }
        }
        // std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
        mapReferencesMutex_.unlock();
    }
    //确认

    // All Mappoints
    void ORBSLAM3Interface::getCurrentMapPoints(sensor_msgs::msg::PointCloud2 &mapPointCloud)
    {
        std::lock_guard<std::mutex> lock(currentMapPointsMutex_);
        // this flag serves to support
        std::vector<Eigen::Vector3f> trackedMapPoints;
        auto atlasAllKFs_ = orbAtlas_->GetAllKeyFrames();
        for (auto& KF : atlasAllKFs_)
        {
            for (auto& mapPoint : KF->GetMapPoints())
            {
                if (!mapPoint->isBad())
                {
                    auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                    mapReferencesMutex_.lock();
                    if(allKFs_.count(KF->mnId) == 0)
                    {
                        mapReferencesMutex_.unlock();
                        continue;
                    }
                    auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[allKFs_[KF->mnId]->GetMap()], worldPos);
                    mapReferencesMutex_.unlock();
                    trackedMapPoints.push_back(mapPointWorld);
                }
            }
        }
        mapPointCloud = typeConversions_->MapPointsToPCL(trackedMapPoints);
    }

    void ORBSLAM3Interface::getCurrentMapPointsToSave(std::vector<Eigen::Vector3f> &trackedMapPoints)
    {
        std::lock_guard<std::mutex> lock(currentMapPointsMutex_);
        // auto atlasAllKFs_ = orbAtlas_->GetAllKeyFrames();
        // for (auto& KF : atlasAllKFs_)
        // {
        //     for (auto& mapPoint : KF->GetMapPoints())
        //     {
        //         if (!mapPoint->isBad())
        //         {
        //             auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
        //             mapReferencesMutex_.lock();
        //             if(allKFs_.count(KF->mnId) == 0)
        //             {
        //                 mapReferencesMutex_.unlock();
        //                 continue;
        //             }
        //             auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[allKFs_[KF->mnId]->GetMap()], worldPos);
        //             mapReferencesMutex_.unlock();
        //             trackedMapPoints.push_back(mapPointWorld);
        //         }
        //     }
        // }
        ORB_SLAM3::Map* pActiveMap = orbAtlas_->GetCurrentMap();
        // this flag serves to support
        // std::vector<Eigen::Vector3f> referenceMapPoints;
        // auto trackedMapPoints_ = mSLAM_->GetTrackedMapPoints();
        for (auto& mapPoint : pActiveMap->GetAllMapPoints())
        {
            if (!mapPoint->isBad())
            {
                auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                mapReferencesMutex_.lock();
                if(allKFs_.count(pActiveMap->GetInitKFid()) == 0)
                {
                    mapReferencesMutex_.unlock();
                    continue;
                }
                // auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[allKFs_[pActiveMap->GetInitKFid()]->GetMap()], worldPos);
                auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[pActiveMap], worldPos);
                mapReferencesMutex_.unlock();
                trackedMapPoints.push_back(mapPointWorld);
            }
        }
    }

    //get current reference map points
    void ORBSLAM3Interface::getReferenceMapPoints(sensor_msgs::msg::PointCloud2 &mapPointCloud)
    {
        std::lock_guard<std::mutex> lock(currentMapPointsMutex_);
        ORB_SLAM3::Map* pActiveMap = orbAtlas_->GetCurrentMap();
        // this flag serves to support
        std::vector<Eigen::Vector3f> referenceMapPoints;
        // auto trackedMapPoints_ = mSLAM_->GetTrackedMapPoints();
        for (auto& mapPoint : pActiveMap->GetReferenceMapPoints())
        {
            if (!mapPoint->isBad())
            {
                auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                mapReferencesMutex_.lock();
                if(allKFs_.count(pActiveMap->GetInitKFid()) == 0)
                {
                    mapReferencesMutex_.unlock();
                    continue;
                }
                // auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[allKFs_[pActiveMap->GetInitKFid()]->GetMap()], worldPos);
                auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[pActiveMap], worldPos);
                mapReferencesMutex_.unlock();
                referenceMapPoints.push_back(mapPointWorld);
            }
        }
        mapPointCloud = typeConversions_->MapPointsToPCL(referenceMapPoints);
    }

    void ORBSLAM3Interface::mapDataToMsg(slam_msgs::msg::MapData &mapDataMsg, bool currentMapKFOnly, bool includeMapPoints, std::vector<int> kFIDforMapPoints)
    {
        std::lock_guard<std::mutex> lock(mapDataMutex_);
        slam_msgs::msg::MapGraph poseGraph_;
        getOptimizedPoseGraph(poseGraph_, currentMapKFOnly);
        // publish the map data
        mapDataMsg.graph = poseGraph_;
        mapDataMsg.header.frame_id = globalFrame_;
        if (includeMapPoints)
        {
            for (auto kFId : kFIDforMapPoints)
            {
                slam_msgs::msg::KeyFrame pushedKf;
                pushedKf.id = kFId;
                if (allKFs_.find(kFId) != allKFs_.end())
                {
                    for (auto mapPoint : allKFs_[kFId]->GetMapPoints())
                    {
                        if (!mapPoint->isBad())
                        {
                            auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                            mapReferencesMutex_.lock();
                            auto mapPointWorld = typeConversions_->transformPointWithReference<geometry_msgs::msg::Point>(mapReferencePoses_[allKFs_[kFId]->GetMap()], worldPos);
                            mapReferencesMutex_.unlock();
                            pushedKf.word_pts.push_back(mapPointWorld);
                        }
                    }
                    mapDataMsg.nodes.push_back(pushedKf);
                }
                else
                {
                    std::cerr << "Requested ID not available." << endl;
                }
            }
        }
        mapDataMutex_.unlock();
    }

    void ORBSLAM3Interface::correctTrackedPose(Sophus::SE3f &s)
    {
        mapReferencesMutex_.lock();
        latestTrackedPose_ = typeConversions_->transformPoseWithReference<Eigen::Affine3d>(
            mapReferencePoses_[orbAtlas_->GetCurrentMap()], s);
        mapReferencesMutex_.unlock();
    }

    void ORBSLAM3Interface::getDirectMapToRobotTF(std_msgs::msg::Header headerToUse, geometry_msgs::msg::TransformStamped &tf)
    {
        tf.header.frame_id = globalFrame_;
        tf.child_frame_id = robotFrame_;
        std::cout << "---###################################-------------------------------" << endl;
        std::cout << globalFrame_ << endl;
        std::cout << robotFrame_ << endl;
        if (hasTracked_)
        {
            std::cout << "---###################################=======================" << endl;
            // get transform between map and odom and send the transform.
            auto tfMapOdom = latestTrackedPose_;
            geometry_msgs::msg::Pose poseMapOdom = tf2::toMsg(tfMapOdom);
            rclcpp::Duration transformTimeout_ = rclcpp::Duration::from_seconds(0.5);
            rclcpp::Time odomTimestamp = headerToUse.stamp;
            tf.header.stamp = odomTimestamp + transformTimeout_;
            tf.header.frame_id = globalFrame_;
            tf.child_frame_id = robotFrame_;
            tf.transform.translation.x = poseMapOdom.position.x;
            tf.transform.translation.y = poseMapOdom.position.y;
            tf.transform.translation.z = poseMapOdom.position.z;
            tf.transform.rotation = poseMapOdom.orientation;
        }
    }

    void ORBSLAM3Interface::getMapToOdomTF(const nav_msgs::msg::Odometry::SharedPtr msgOdom, geometry_msgs::msg::TransformStamped &tf)
    {
        // tf.header.stamp;
        tf.header.frame_id = globalFrame_;
        tf.child_frame_id = odomFrame_;
        if (hasTracked_)
        {
            // convert odom value to Eigen::Affine3d
            auto latestOdomTransform_ = Eigen::Affine3d(
                Eigen::Translation3d(msgOdom->pose.pose.position.x,
                                     msgOdom->pose.pose.position.y,
                                     msgOdom->pose.pose.position.z) *
                Eigen::Quaterniond(msgOdom->pose.pose.orientation.w,
                                   msgOdom->pose.pose.orientation.x,
                                   msgOdom->pose.pose.orientation.y,
                                   msgOdom->pose.pose.orientation.z));
            // get transform between map and odom and send the transform.
            auto tfMapOdom = latestTrackedPose_ * latestOdomTransform_.inverse();
            geometry_msgs::msg::Pose poseMapOdom = tf2::toMsg(tfMapOdom);
            rclcpp::Duration transformTimeout_ = rclcpp::Duration::from_seconds(0.5);
            rclcpp::Time odomTimestamp = msgOdom->header.stamp;
            tf.header.stamp = odomTimestamp + transformTimeout_;
            tf.header.frame_id = globalFrame_;
            tf.child_frame_id = odomFrame_;
            tf.transform.translation.x = poseMapOdom.position.x;
            tf.transform.translation.y = poseMapOdom.position.y;
            tf.transform.translation.z = poseMapOdom.position.z;
            tf.transform.rotation = poseMapOdom.orientation;
        }
    }

    void ORBSLAM3Interface::getOptimizedPoseGraph(slam_msgs::msg::MapGraph &graph, bool currentMapKFOnly)
    {
        if (!currentMapKFOnly)
        {
            for (const auto &cKf : allKFs_)
            {
                ORB_SLAM3::KeyFrame *kf = cKf.second;
                Sophus::SE3f kfPose = kf->GetPose();
                geometry_msgs::msg::PoseStamped kfPoseStamped;
                mapReferencesMutex_.lock();
                kfPoseStamped.pose = typeConversions_->transformPoseWithReference<geometry_msgs::msg::Pose>(mapReferencePoses_[kf->GetMap()], kfPose);
                mapReferencesMutex_.unlock();
                kfPoseStamped.header.frame_id = globalFrame_;
                kfPoseStamped.header.stamp = typeConversions_->secToStamp(kf->mTimeStamp);
                graph.poses.push_back(kfPoseStamped);
                graph.poses_id.push_back(kf->mnId);
            }
        }
        else
        {
            vector<ORB_SLAM3::KeyFrame *> vKeyFrames = orbAtlas_->GetAllKeyFrames();
            // TODO: add isBad() check for keyframes. Evaluate mapping if you do this.
            // iterate over current keyframes.
            for (auto pKFcurr : vKeyFrames)
            {
                mapReferencesMutex_.lock();
                auto currReferencePose_ = mapReferencePoses_[pKFcurr->GetMap()];
                mapReferencesMutex_.unlock();
                Sophus::SE3f kFPose = pKFcurr->GetPose();
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.pose = typeConversions_->transformPoseWithReference<geometry_msgs::msg::Pose>(currReferencePose_, kFPose);
                poseStamped.header.frame_id = globalFrame_;
                poseStamped.header.stamp = typeConversions_->secToStamp(pKFcurr->mTimeStamp);
                // push to pose graph.
                graph.poses.push_back(poseStamped);
                graph.poses_id.push_back(pKFcurr->mnId);
            }
        }
    }

    void ORBSLAM3Interface::handleIMU(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        bufMutex_.lock();
        imuBuf_.push(msgIMU);
        bufMutex_.unlock();
    }

    bool ORBSLAM3Interface::trackRGBDi(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD, Sophus::SE3f &Tcw)
    {
        orbAtlas_ = mSLAM_->GetAtlas();
        cv_bridge::CvImageConstPtr cvRGB;
        cv_bridge::CvImageConstPtr cvD;
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cvRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception RGB!" << endl;
            return false;
        }

        // Copy the ros depth image message to cv::Mat.
        try
        {
            cvD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception D!" << endl;
            return false;
        }

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        bufMutex_.lock();
        if (!imuBuf_.empty())
        {
            // Load imu measurements from buffer
            vImuMeas.clear();
            while (!imuBuf_.empty() && typeConversions_->stampToSec(imuBuf_.front()->header.stamp) <= std::min(typeConversions_->stampToSec(msgRGB->header.stamp), typeConversions_->stampToSec(msgD->header.stamp)))
            {
                double t = typeConversions_->stampToSec(imuBuf_.front()->header.stamp);
                cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                imuBuf_.pop();
            }
        }
        bufMutex_.unlock();
        if (imuBuf_.size() > 0)
        {
            // track the frame.
            Tcw = mSLAM_->TrackRGBD(cvRGB->image, cvD->image, typeConversions_->stampToSec(msgRGB->header.stamp), vImuMeas);
            auto currentTrackingState = mSLAM_->GetTrackingState();
            auto orbLoopClosing = mSLAM_->GetLoopClosing();
            if (orbLoopClosing->mergeDetected())
            {
                // do not publish any values during map merging. This is because the reference poses change.
                std::cout << "Waiting for merge to finish." << endl;
                return false;
            }
            if (currentTrackingState == 2)
            {
                calculateReferencePoses();
                correctTrackedPose(Tcw);
                hasTracked_ = true;
                return true;
            }
            else
            {
                switch (currentTrackingState)
                {
                case 0:
                    std::cerr << "ORB-SLAM failed: No images yet." << endl;
                    break;
                case 1:
                    std::cerr << "ORB-SLAM failed: Not initialized." << endl;
                    break;
                case 3:
                    std::cerr << "ORB-SLAM failed: Tracking LOST." << endl;
                    break;
                }
                return false;
            }
        }
        return false;
    }

    bool ORBSLAM3Interface::trackRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD, Sophus::SE3f &Tcw)
    {
        orbAtlas_ = mSLAM_->GetAtlas();
        cv_bridge::CvImageConstPtr cvRGB;
        cv_bridge::CvImageConstPtr cvD;
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cvRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception RGB!" << endl;
            return false;
        }

        // Copy the ros depth image message to cv::Mat.
        try
        {
            cvD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception D!" << endl;
            return false;
        }
        // track the frame.
        Tcw = mSLAM_->TrackRGBD(cvRGB->image, cvD->image, typeConversions_->stampToSec(msgRGB->header.stamp));
        auto currentTrackingState = mSLAM_->GetTrackingState();
        auto orbLoopClosing = mSLAM_->GetLoopClosing();
        if (orbLoopClosing->mergeDetected())
        {
            // do not publish any values during map merging. This is because the reference poses change.
            std::cout << "Waiting for merge to finish." << endl;
            return false;
        }
        if (currentTrackingState == 2)
        {
            calculateReferencePoses();
            correctTrackedPose(Tcw);
            hasTracked_ = true;
            return true;
        }
        else
        {
            switch (currentTrackingState)
            {
            case 0:
                std::cerr << "ORB-SLAM failed: No images yet." << endl;
                break;
            case 1:
                std::cerr << "ORB-SLAM failed: Not initialized." << endl;
                break;
            case 3:
                std::cerr << "ORB-SLAM failed: Tracking LOST." << endl;
                break;
            }
            return false;
        }
    }
    
    int ORBSLAM3Interface::trackMONO(const sensor_msgs::msg::Image::SharedPtr msgRGB, Sophus::SE3f &Tcw)
    {
        orbAtlas_ = mSLAM_->GetAtlas();
        cv_bridge::CvImageConstPtr cvRGB;
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cvRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception RGB!" << endl;
            return 0; // No images yet
        }

        // track the frame.
        Tcw = mSLAM_->TrackMonocular(cvRGB->image, typeConversions_->stampToSec(msgRGB->header.stamp));
        
        auto currentTrackingState = mSLAM_->GetTrackingState();
        auto orbLoopClosing = mSLAM_->GetLoopClosing();
        if (orbLoopClosing->mergeDetected())
        {
            // do not publish any values during map merging. This is because the reference poses change.
            std::cout << "Waiting for merge to finish." << endl;
            return 0; // Return as "No images yet" since we can't track during merge
        }
        
        if (currentTrackingState == 2) // OK
        {
            calculateReferencePoses();
            correctTrackedPose(Tcw);
            hasTracked_ = true;
        }
        else
        {
            switch (currentTrackingState)
            {
            case 0:
                std::cerr << "ORB-SLAM failed: No images yet." << endl;
                break;
            case 1:
                std::cerr << "ORB-SLAM failed: Not initialized." << endl;
                break;
            case 3:
                std::cerr << "ORB-SLAM failed: Tracking LOST." << endl;
                break;
            }
            // return false;
        }

        return currentTrackingState;
    }

    bool ORBSLAM3Interface::checkSLAMShutdown()
    {
        return mSLAM_->isShutDown();
    }

    void ORBSLAM3Interface::triggerGlobalBundleAdjustment()
    {
        if (mSLAM_ && orbAtlas_) {
            std::cout << "Triggering global bundle adjustment..." << std::endl;
            ORB_SLAM3::Map* pActiveMap = orbAtlas_->GetCurrentMap();
            if (pActiveMap) {
                // Use 0 as the loop keyframe ID, or get the current keyframe ID if needed
                mSLAM_->GetLoopClosing()->RunGlobalBundleAdjustment(pActiveMap, 0);
                std::cout << "Global bundle adjustment completed" << std::endl;
            } else {
                std::cerr << "Cannot trigger global BA: No active map" << std::endl;
            }
        } else {
            std::cerr << "Cannot trigger global BA: SLAM system not initialized" << std::endl;
        }
    }

    void ORBSLAM3Interface::setupCameraPoseSubscriber(rclcpp::Node::SharedPtr node)
    {
        cameraPoseSub_ = node->create_subscription<geometry_msgs::msg::Pose>(
            "camera_pose", 10, std::bind(&ORBSLAM3Interface::cameraPoseCallback, this, std::placeholders::_1));
        
        // Create CSV file
        csvFile_.open("camera_pose.csv");
        csvFile_ << "timestamp,x,y,z\n";
    }

    void ORBSLAM3Interface::cameraPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch();
        auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
        long timestamp = value.count();

        // Write to CSV file
        csvFile_ << timestamp << ","
                << msg->position.x << ","
                << msg->position.y << ","
                << msg->position.z << "\n";
        csvFile_.flush();
    }
}
