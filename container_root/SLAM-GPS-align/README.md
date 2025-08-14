# SLAM-GPS Alignment Module

This module provides tools and workflows for aligning SLAM results with GPS coordinates, enabling the transformation from local SLAM coordinate systems to global GPS coordinates.

## Overview

The SLAM-GPS alignment process bridges the gap between local SLAM maps and global positioning systems by:
- Converting RTK GPS data to a standardized CSV format
- Computing transformation matrices between SLAM and GPS coordinate systems  
- Applying transformations to convert SLAM poses to GPS coordinates
- Visualizing and validating alignment results

## Requirements

### Prerequisites

- **Server** with TerraSLAM system deployed
- RTK GPS data files (*.pos or *.llh format)
- Camera images from the surveyed area

### Dependencies
- Python 3.x with required packages
- ORB-SLAM3 system
- TerraSLAM relay components
- RTK GPS processing tools

### Directory Structure
```
SLAM-GPS-align/
├── images/              # Camera images
├── pose_GPS.py         # GPS processing scripts
├── rtk_llh_to_csv.py   
├── transform.py        
├── slam_to_gps.py      
├── plot_trajectory.py  
├── camera_pose.csv     # SLAM trajectory data
├── RTK.csv            # GPS reference data
└── transform.json     # Transformation parameter output
```

## Workflow

### Step 1: SLAM Map Generation

**On Server:**

1. **Generate base map**: Run multiple SLAM sessions to create an optimal map of the target area, saving both the map file (*.osa) and corresponding images.

2. **Store map file**: Save the generated map file (*.osa) to the appropriate directory:
   ```bash
   # Save to: container_root/Map/
   ```

### Step 2: SLAM Pose Recording

**On Server:**

3. **Record camera poses**: Run SLAM with the pre-built map loaded and use the pose recording script:
   ```bash
   python olympe_dev/pose_record.py
   ```
   This generates a `camera_pose.csv` file containing SLAM trajectory data.

4. **Organize data**: Ensure all images and the `camera_pose.csv` file are in the working directory:
   ```bash
   # Files location: SLAM-GPS-align/images/ and SLAM-GPS-align/camera_pose.csv
   ```

### Step 3: GPS Data Processing

**On Server:**

5. **Process RTK data**: Convert RTK GPS recordings to CSV format using one of the following methods:

   - **For *.pos files**:
     ```bash
     python pose_GPS.py your_file.pos
     ```
   
   - **For *.llh files**:
     ```bash
     python rtk_llh_to_csv.py your_file.LLH -o RTK.csv
     ```

### Step 4: Coordinate Transformation

**On Server:**

6. **Compute transformation matrix**:
   ```bash
   python transform.py camera_pose.csv RTK.csv -o transform.json
   ```
   This generates a `transform.json` file containing the transformation parameters.

7. **Apply transformation to SLAM poses**:
   ```bash
   python slam_to_gps.py camera_pose.csv -t transform.json
   ```
   This creates a `camera_pose_GPS.csv` file with SLAM poses converted to GPS coordinates.

### Step 5: Visualization and Validation

**On Server:**

8. **Generate trajectory plots**:
   ```bash
   python plot_trajectory.py camera_pose_GPS.csv RTK.csv --export_ply
   ```
   This creates visualization plots and optionally exports 3D point cloud files (.ply) for further analysis.

### Step 6: Fine-tuning (Optional)

**On Server:**

9. **Refine alignment**: If CloudCompare or similar tools are available on the server, use them to fine-tune the alignment using the generated .ply files:
   - Load the point cloud files
   - Manually adjust alignment if necessary
   - Generate a refinement transformation matrix
   - Apply this matrix to the original `transform.json`:
     ```bash
     # Calculate: final_transform = refinement_matrix @ transform.json
     ```

### Step 7: Deployment

**On Server:**

10. **Deploy transformation**: Copy the final `transform.json` to the SLAM engine directory:
    ```bash
    cp transform.json /path/to/slam/engine/transform.json
    ```

11. **Update SLAM configuration**: Configure SLAM parameters to use the pre-built map:
    ```bash
    # Edit configuration files as needed
    ```

### Step 8: System Initialization and Testing

**On Server:**

12. **Initialize SLAM system**: Start the SLAM system with the new transformation:
    ```bash
    python TerraSLAM_relay/relay_client.py images/your_image_directory -p 43322 -t transform.json
    ```

### Step 9: Recovery Procedures

If SLAM loses tracking during operation:

**On Server:**

1. **Activate recovery mode**:
   ```bash
   python3 TerraSLAM_relay/relay_backdoor.py
   ```

2. **Reinitialize tracking**:
   ```bash
   python TerraSLAM_relay/relay_client.py images/your_image_directory -p 43332 -t transform.json
   ```

## File Descriptions

- **`pose_GPS.py`** - Converts RTK GPS data from .pos format to CSV
- **`rtk_llh_to_csv.py`** - Converts RTK GPS data from .llh format to CSV  
- **`transform.py`** - Computes transformation matrix between SLAM and GPS coordinates
- **`slam_to_gps.py`** - Applies transformation to convert SLAM poses to GPS coordinates
- **`plot_trajectory.py`** - Visualizes trajectories and exports point cloud data

## Data Formats

### Input Files
- **SLAM poses**: `camera_pose.csv` - Timestamp, position, and orientation data from SLAM
- **GPS data**: `RTK.csv` - Timestamp and GPS coordinates from RTK system
- **Images**: Raw camera images used for SLAM processing

### Output Files
- **`transform.json`** - Transformation parameters for SLAM-to-GPS coordinate conversion
- **`camera_pose_GPS.csv`** - SLAM trajectory converted to GPS coordinates
- **Visualization plots** - Trajectory comparison and alignment validation
- **Point clouds** (optional) - 3D data for external analysis tools


## Troubleshooting

### Common Issues

1. **Coordinate system misalignment**: Ensure both SLAM and GPS data cover the same spatial area and time period
2. **Insufficient overlap**: Verify that SLAM trajectory and GPS path have adequate spatial overlap for transformation calculation
3. **Timestamp synchronization**: Check that SLAM and GPS timestamps are properly synchronized
4. **Transformation accuracy**: Validate alignment results using visualization tools and manual inspection
5. **Server resource allocation**: Ensure sufficient computational resources for processing large datasets

### Performance Optimization

- **Batch processing**: Process multiple datasets simultaneously when server resources allow
- **Data preprocessing**: Clean and filter input data to improve transformation accuracy
- **Caching**: Store intermediate results to avoid recomputation during iterative refinement

### Support

For additional support and detailed parameter explanations, refer to the individual script documentation or contact the development team.