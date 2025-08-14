/**
 * @file mono-slam-node.cpp
 * @brief Implementation of the MonoSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "mono-slam-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    MonoSlamNode::MonoSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_MONO_ROS2")
    {
        // ROS Subscribers
        rgbSub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw",10,std::bind(&MonoSlamNode::MONOCallback,
        									    	    this,std::placeholders::_1));

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1000, std::bind(&MonoSlamNode::ImuCallback, this, std::placeholders::_1));
        // odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1000, std::bind(&MonoSlamNode::OdomCallback, this, std::placeholders::_1));
        // ROS Publishers 
        // mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_map_points", 10);
        // currentMapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_current_map_points", 10);


        // referenceMapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mono_reference_map_points", 10);
        cameraPosePub_ = this->create_publisher<geometry_msgs::msg::Pose>("camera_pose", 10);
        
        // TF
        // tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(false));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("ros_visualization", rclcpp::ParameterValue(false));
        this->get_parameter("ros_visualization", rosViz_);

        this->declare_parameter("robot_base_frame", "base_link");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("no_odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("no_odometry_mode", no_odometry_mode_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("landmark_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("landmark_publish_frequency", landmark_publish_frequency_);
        
	    mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // mapReferencePointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(landmark_publish_frequency_), std::bind(&MonoSlamNode::publishReferenceMapPointCloud, this));
        
        //mapCurrentPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(100 * landmark_publish_frequency_), std::bind(&MonoSlamNode::publishCurrentMapPointCloud, this));
        mapCurrentPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(5 * landmark_publish_frequency_), std::bind(&MonoSlamNode::saveCurrentMapPointCloud, this));
        
        //mapPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(landmark_publish_frequency_), std::bind(&MonoSlamNode::combinedPublishCallback, this));


        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, rosViz_, robot_x_,
                                                                            robot_y_, global_frame_, odom_frame_id_, robot_base_frame_id_);
        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    MonoSlamNode::~MonoSlamNode()
    {
        //saveCurrentMapPointCloud();
        rgbSub_.reset();
        imuSub_.reset();
        odomSub_.reset();
        interface_.reset();
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void MonoSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        // push value to imu buffer.
        interface_->handleIMU(msgIMU);
    }

    void MonoSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if(!no_odometry_mode_)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else RCLCPP_WARN(this->get_logger(), "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }
    // Replication
    void MonoSlamNode::MONOCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB)
    {
        Sophus::SE3f Tcw;
        int trackingState = interface_->trackMONO(msgRGB, Tcw);
        
        // Create pose message
        auto camPose = typeConversion_.se3ToPoseMsg(Tcw);
        
        // Handle different tracking states
        if (trackingState == 2) // OK
        {
            isTracked_ = true;
            // publish camera's pose as normal
            cameraPosePub_->publish(camPose);
        }
        else if (trackingState == 1) // Not initialized
        {
            // Create a pose with all -1 values
            geometry_msgs::msg::Pose notInitializedPose;
            notInitializedPose.position.x = -1.0;
            notInitializedPose.position.y = -1.0;
            notInitializedPose.position.z = -1.0;
            notInitializedPose.orientation.x = -1.0;
            notInitializedPose.orientation.y = -1.0;
            notInitializedPose.orientation.z = -1.0;
            notInitializedPose.orientation.w = -1.0;
            cameraPosePub_->publish(notInitializedPose);
        }
        else if (trackingState == 3) // Tracking lost
        {
            // Create a pose with all -3 values
            geometry_msgs::msg::Pose trackingLostPose;
            trackingLostPose.position.x = -3.0;
            trackingLostPose.position.y = -3.0;
            trackingLostPose.position.z = -3.0;
            trackingLostPose.orientation.x = -3.0;
            trackingLostPose.orientation.y = -3.0;
            trackingLostPose.orientation.z = -3.0;
            trackingLostPose.orientation.w = -3.0;
            cameraPosePub_->publish(trackingLostPose);
        }
        else // No images yet or merge detected
        {
            // Create a pose with all 0 values
            geometry_msgs::msg::Pose noImagesPose;
            noImagesPose.position.x = 0.0;
            noImagesPose.position.y = 0.0;
            noImagesPose.position.z = 0.0;
            noImagesPose.orientation.x = 0.0;
            noImagesPose.orientation.y = 0.0;
            noImagesPose.orientation.z = 0.0;
            noImagesPose.orientation.w = 0.0;
            cameraPosePub_->publish(noImagesPose);
        }
    }
    // Replication
    void MonoSlamNode::publishCurrentMapPointCloud()
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);
            // interface_->getReferenceMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            currentMapPointsPub_->publish(mapPCL);
            // auto now = std::chrono::system_clock::now();
            // auto in_time_t = std::chrono::system_clock::to_time_t(now);
            // std::stringstream ss;
            // ss << std::put_time(std::localtime(&in_time_t), "map_%Y%m%d_%H%M%S.ply");
            // savePointCloudToPLY(mapPCL, ss.str());

            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            // RCLCPP_INFO_STREAM(this->get_logger(), "Time to save " << ss.str() <<" map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish current map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");


            // Calculate the time taken for each line

            // Print the time taken for each line
        }
    }

    void MonoSlamNode::saveCurrentMapPointCloud()
    {
        if (interface_->checkSLAMShutdown() && !isMapPointsSaved)
        {
            if (isTracked_)
            {
                std::vector<Eigen::Vector3f> trackedMapPoints;

                interface_->getCurrentMapPointsToSave(trackedMapPoints);
                // interface_->getReferenceMapPoints(mapPCL);

                if(trackedMapPoints.size() == 0)
                    return;

                //currentMapPointsPub_->publish(mapPCL);
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "map_%Y%m%d_%H%M%S.ply");
                savePointsToPLY(trackedMapPoints, ss.str());

                auto t3 = std::chrono::high_resolution_clock::now();
                auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - now).count();
                RCLCPP_INFO_STREAM(this->get_logger(), "Time to save " << ss.str() <<" map points: " << time_publish_map_points << " seconds");
                RCLCPP_INFO_STREAM(this->get_logger(), "=======================");
                isMapPointsSaved = true;
            }
        }
    }

    void MonoSlamNode::savePointsToPLY(const std::vector<Eigen::Vector3f> &points, const std::string &filename) 
    {
        std::ofstream outFile(filename);
        if (!outFile.is_open()) {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return;
        }

        // PLY header
        outFile << "ply\n";
        outFile << "format ascii 1.0\n";
        outFile << "element vertex " << points.size() << "\n";
        outFile << "property float x\n";
        outFile << "property float y\n";
        outFile << "property float z\n";
        outFile << "end_header\n";

        // Point data
        for (const Eigen::Vector3f &point : points) {
            outFile << point[0] << " " << point[1] << " " << point[2] << "\n";
        }

        outFile.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Saved " << points.size() << " points to '" << filename << "'");
        // std::cout << "Saved " << points.size() << " points to '" << filename << "'\n";
    }

    void MonoSlamNode::publishReferenceMapPointCloud()
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            // interface_->getCurrentMapPoints(mapPCL);
            interface_->getReferenceMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            referenceMapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");


            // Calculate the time taken for each line

            // Print the time taken for each line
        }
    }

    void MonoSlamNode::combinedPublishCallback() 
    {
        this->publishCurrentMapPointCloud();
        this->publishReferenceMapPointCloud();
    }

    // void MonoSlamNode::savePointCloudToPLY(const sensor_msgs::msg::PointCloud2 &msg, const std::string &filename) 
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //     pcl::fromROSMsg(msg, *cloud);  // Convert from ROS msg to PCL cloud
    //     pcl::io::savePLYFileASCII(filename, *cloud);  // Save as PLY file
    // }


}
