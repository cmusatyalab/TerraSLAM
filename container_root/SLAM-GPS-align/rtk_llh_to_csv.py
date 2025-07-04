import sys
import re
import pandas as pd
import argparse

def parse_llh_file(input_path, output_path):
    # 读取文件内容
    with open(input_path, 'r') as f:
        lines = f.readlines()

    # 匹配 time, lat, lon, alt
    pattern = re.compile(r"(\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d+)\s+([\d\.\-]+)\s+([\d\.\-]+)\s+([\d\.\-]+)")
    results = []

    for line in lines:
        match = pattern.search(line)
        if match:
            time, lat, lon, alt = match.groups()
            results.append([time, float(lon), float(lat), float(alt)])

    # 创建 DataFrame
    df = pd.DataFrame(results, columns=["time", "longitude", "latitude", "height"])

    # 写入 CSV
    df.to_csv(output_path, index=False)
    print(f"Output written to: {output_path}")

def main():
    # 创建参数解析器
    parser = argparse.ArgumentParser(
        description='Convert RTK LLH file to CSV format',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # 添加必需参数
    parser.add_argument('input_file', 
                       help='Path to input LLH file')
    
    # 添加可选参数
    parser.add_argument('-o', '--output', 
                       help='Path to output CSV file (default: input filename with .csv extension)')
    
    # 解析参数
    args = parser.parse_args()
    
    # 如果没有指定输出文件，则自动生成
    if args.output is None:
        if args.input_file.endswith('.llh'):
            output_file = args.input_file[:-4] + '.csv'
        else:
            output_file = args.input_file + '.csv'
    else:
        output_file = args.output
    
    # 执行转换
    parse_llh_file(args.input_file, output_file)

if __name__ == "__main__":
    main()