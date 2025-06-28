import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pymap3d import geodetic2enu
from mpl_toolkits.mplot3d import Axes3D
import argparse

def save_to_ply(points, filename, color):
    """
    Save trajectory points as a PLY point cloud file
    
    Parameters:
        points: numpy array with shape (n,3), containing x,y,z coordinates of points
        filename: output PLY filename
        color: RGB color tuple (r,g,b), each component in range 0-255
    """
    with open(filename, 'w') as f:
        # Write PLY file header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        
        # Write point data
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]} {color[0]} {color[1]} {color[2]}\n")
    
    print(f"Point cloud saved to {filename}")

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Plot SLAM and RTK trajectories')
    parser.add_argument('slam_csv', help='Path to SLAM trajectory CSV file')
    parser.add_argument('rtk_csv', help='Path to RTK trajectory CSV file')
    parser.add_argument('--export_ply', action='store_true', help='Export trajectories as PLY point clouds')
    args = parser.parse_args()

    # Read GPS coordinate data
    df = pd.read_csv(args.slam_csv)
    df_rtk = pd.read_csv(args.rtk_csv)

    # Select reference point (first RTK point) as origin
    try:    
        lat0 = df_rtk['latitude'].iloc[0]
        lon0 = df_rtk['longitude'].iloc[0]
        h0 = df_rtk['height'].iloc[0]
    except Exception as e:
        print(f"Error selecting reference point: {e}")
        lat0 = df_rtk['latitude'].iloc[0]
        lon0 = df_rtk['longitude'].iloc[0]
        h0 = df_rtk['height'].iloc[0]

    # Convert SLAM trajectory to ENU coordinate system
    e_slam, n_slam, u_slam = geodetic2enu(df['latitude'], df['longitude'], df['altitude'],
                                        lat0, lon0, h0)

    # Convert RTK trajectory to ENU coordinate system
    try:
        e_rtk, n_rtk, u_rtk = geodetic2enu(df_rtk['latitude'], df_rtk['longitude'], df_rtk['height'],
                                        lat0, lon0, h0)
    except Exception as e:
        print(f"Error converting RTK trajectory: {e}")
        e_rtk, n_rtk, u_rtk = geodetic2enu(df_rtk['latitude'], df_rtk['longitude'], df_rtk['height'],
                                        lat0, lon0, h0)

    # Convert pandas Series to numpy arrays
    e_slam = np.array(e_slam)
    n_slam = np.array(n_slam)
    u_slam = np.array(u_slam)
    e_rtk = np.array(e_rtk)
    n_rtk = np.array(n_rtk)
    u_rtk = np.array(u_rtk)

    # If PLY point cloud export is needed
    if args.export_ply:
        # Prepare SLAM and RTK point cloud data
        slam_points = np.column_stack((e_slam, n_slam, u_slam))
        rtk_points = np.column_stack((e_rtk, n_rtk, u_rtk))
        
        # Save as PLY files, SLAM trajectory in green, RTK trajectory in red
        save_to_ply(slam_points, "slam_trajectory.ply", (0, 255, 0))  # Green
        save_to_ply(rtk_points, "rtk_trajectory.ply", (255, 0, 0))    # Red
        
        print("Trajectory point cloud export completed!")

    # Print endpoint coordinates
    print("\nSLAM endpoint coordinates:")
    print(f"East: {e_slam[-1]:.2f} m")
    print(f"North: {n_slam[-1]:.2f} m")
    print(f"Up: {u_slam[-1]:.2f} m")
    
    print("\nRTK endpoint coordinates:")
    print(f"East: {e_rtk[-1]:.2f} m")
    print(f"North: {n_rtk[-1]:.2f} m")
    print(f"Up: {u_rtk[-1]:.2f} m")

    # Create charts
    plt.figure(figsize=(15, 10))

    # Plot East direction changes
    plt.subplot(3, 1, 1)
    plt.plot(e_slam, 'g-', label='SLAM')
    plt.plot(e_rtk, 'r--', label='RTK')
    plt.title('East (m)')
    plt.ylabel('East (m)')
    plt.grid(True)
    plt.legend()

    # Plot North direction changes
    plt.subplot(3, 1, 2)
    plt.plot(n_slam, 'g-', label='SLAM')
    plt.plot(n_rtk, 'r--', label='RTK')
    plt.title('North (m)')
    plt.ylabel('North (m)')
    plt.grid(True)
    plt.legend()

    # Plot Up direction changes
    plt.subplot(3, 1, 3)
    plt.plot(u_slam, 'g-', label='SLAM')
    plt.plot(u_rtk, 'r--', label='RTK')
    plt.title('Up (m)')
    plt.xlabel('Point Sequence')
    plt.ylabel('Up (m)')
    plt.grid(True)
    plt.legend()

    # Adjust layout
    plt.tight_layout()

    # Save first chart
    # plt.savefig('enu_components.png', dpi=300, bbox_inches='tight')

    # Create 3D trajectory chart
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot 3D trajectories
    ax.plot3D(e_slam, n_slam, u_slam, 'g-', label='SLAM Trajectory')
    ax.plot3D(e_rtk, n_rtk, u_rtk, 'r--', label='RTK Trajectory')

    # Add start and end point markers
    ax.scatter(e_slam[0], n_slam[0], u_slam[0], c='green', marker='o', s=100, label='SLAM Start')
    ax.scatter(e_slam[len(e_slam)-1], n_slam[len(n_slam)-1], u_slam[len(u_slam)-1], c='darkgreen', marker='o', s=100, label='SLAM End')
    ax.scatter(e_rtk[0], n_rtk[0], u_rtk[0], c='red', marker='o', s=100, label='RTK Start')
    ax.scatter(e_rtk[len(e_rtk)-1], n_rtk[len(n_rtk)-1], u_rtk[len(u_rtk)-1], c='darkred', marker='o', s=100, label='RTK End')

    # Set axis labels
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')
    ax.set_title('3D Trajectory Comparison')

    # Add legend
    ax.legend()

    # Adjust viewing angle for better visual effect
    ax.view_init(elev=20, azim=45)

    # Save 3D trajectory chart
    # plt.savefig('enu_3d_trajectory.png', dpi=300, bbox_inches='tight')

    # Display all charts
    plt.show()

if __name__ == "__main__":
    main() 