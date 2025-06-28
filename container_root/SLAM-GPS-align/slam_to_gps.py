import pandas as pd
import numpy as np
import argparse
from transform import SLAM2GPS

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Convert SLAM coordinates to GPS coordinates')
    parser.add_argument('input_csv', help='Path to input SLAM coordinates CSV file')
    parser.add_argument('-t', '--transform', default='transform.json', 
                       help='Path to transform parameter file (default: transform.json)')
    args = parser.parse_args()

    # 生成输出文件名
    input_path = args.input_csv
    if input_path.endswith('.csv'):
        output_path = input_path[:-4] + '_gps.csv'
    else:
        output_path = input_path + '_gps.csv'

    # 加载转换器
    converter = SLAM2GPS(args.transform)

    # 读取相机位姿
    df = pd.read_csv(input_path)

    # 转换每个 x,y,z 坐标到 GPS
    gps_coords = []
    for _, row in df.iterrows():
        xyz = np.array([row['x'], row['y'], row['z']])
        lat_lon_alt = converter.slam_to_lla(xyz)
        gps_coords.append(lat_lon_alt[0])  # 获取第一行，因为 slam_to_lla 返回 2D 数组

    # 创建新的 DataFrame 包含 GPS 坐标
    gps_df = pd.DataFrame(gps_coords, columns=['latitude', 'longitude', 'altitude'])

    # 保存到新的 CSV 文件
    gps_df.to_csv(output_path, index=False)
    print(f"转换完成。结果已保存到 {output_path}")

if __name__ == "__main__":
    main() 