import pandas as pd
import os
import sys

def pos_to_csv(pos_file_path, output_csv_path=None):
    """
    Convert RTK .pos file to CSV format, keeping only time, latitude, longitude, and height
    
    Args:
        pos_file_path (str): Path to the .pos file
        output_csv_path (str, optional): Path for the output CSV file. If None, will use same name as input with .csv extension
    """
    # If output path not specified, create one based on input file
    if output_csv_path is None:
        output_csv_path = os.path.splitext(pos_file_path)[0] + '.csv'
    
    # Read the .pos file
    with open(pos_file_path, 'r') as f:
        lines = f.readlines()
    
    # Find the start of data (skip header lines)
    data_start = 0
    for i, line in enumerate(lines):
        if line.startswith('2025'):  # Looking for the first data line
            data_start = i
            break
    
    # Read data into DataFrame
    df = pd.read_csv(pos_file_path, 
                    skiprows=data_start,
                    delim_whitespace=True,
                    header=None,
                    names=['GPST', 'latitude', 'longitude', 'height', 'Q', 'ns', 
                          'sdn', 'sde', 'sdu', 'sdne', 'sdeu', 'sdun', 'age', 'ratio'])
    
    # Keep only the required columns
    df = df[['GPST', 'latitude', 'longitude', 'height']]
    
    # Save to CSV
    df.to_csv(output_csv_path, index=False)
    print(f"Successfully converted {pos_file_path} to {output_csv_path}")

if __name__ == "__main__":
    # Default pose file
    default_pos_file = "CMUSatyaRTK_raw_20250505.pos"
    
    # Get pose file from command line argument if provided
    if len(sys.argv) > 1:
        pos_file = sys.argv[1]
    else:
        pos_file = default_pos_file
        
    if os.path.exists(pos_file):
        pos_to_csv(pos_file)
    else:
        print(f"Error: {pos_file} not found")
        print(f"Usage: python {os.path.basename(__file__)} [path_to_pos_file]")
