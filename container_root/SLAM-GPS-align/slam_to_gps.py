import pandas as pd
import numpy as np
import argparse
from transform import SLAM2GPS

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Convert SLAM coordinates to GPS coordinates')
    parser.add_argument('input_csv', help='Path to input SLAM coordinates CSV file')
    parser.add_argument('-t', '--transform', default='transform.json', 
                       help='Path to transform parameter file (default: transform.json)')
    args = parser.parse_args()

    # Generate output filename
    input_path = args.input_csv
    if input_path.endswith('.csv'):
        output_path = input_path[:-4] + '_gps.csv'
    else:
        output_path = input_path + '_gps.csv'

    # Load converter
    converter = SLAM2GPS(args.transform)

    # Read camera poses
    df = pd.read_csv(input_path)

    # Convert each x,y,z coordinate to GPS
    gps_coords = []
    for _, row in df.iterrows():
        xyz = np.array([row['x'], row['y'], row['z']])
        lat_lon_alt = converter.slam_to_lla(xyz)
        gps_coords.append(lat_lon_alt[0])  # Get first row since slam_to_lla returns 2D array

    # Create new DataFrame containing GPS coordinates
    gps_df = pd.DataFrame(gps_coords, columns=['latitude', 'longitude', 'altitude'])

    # Save to new CSV file
    gps_df.to_csv(output_path, index=False)
    print(f"Conversion completed. Results saved to {output_path}")

if __name__ == "__main__":
    main() 