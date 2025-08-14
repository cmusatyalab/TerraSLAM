import sys
import re
import pandas as pd
import argparse

def parse_llh_file(input_path, output_path):
    # Read file content
    with open(input_path, 'r') as f:
        lines = f.readlines()

    # Match time, lat, lon, alt
    pattern = re.compile(r"(\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s+([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)")
    results = []

    for line in lines:
        match = pattern.search(line)
        if match:
            time, lat, lon, alt = match.groups()
            results.append([time, float(lon), float(lat), float(alt)])

    # Create DataFrame
    df = pd.DataFrame(results, columns=["time", "longitude", "latitude", "height"])

    # Write to CSV
    df.to_csv(output_path, index=False)
    print(f"Output written to: {output_path}")

def main():
    # Create argument parser
    parser = argparse.ArgumentParser(
        description='Convert RTK LLH file to CSV format',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Add required parameters
    parser.add_argument('input_file', 
                       help='Path to input LLH file')
    
    # Add optional parameters
    parser.add_argument('-o', '--output', 
                       help='Path to output CSV file (default: input filename with .csv extension)')
    
    # Parse arguments
    args = parser.parse_args()
    
    # If output file is not specified, auto-generate one
    if args.output is None:
        if args.input_file.endswith('.llh'):
            output_file = args.input_file[:-4] + '.csv'
        else:
            output_file = args.input_file + '.csv'
    else:
        output_file = args.output
    
    # Execute conversion
    parse_llh_file(args.input_file, output_file)

if __name__ == "__main__":
    main()