#!/bin/bash

# 参数传入
input_bag=$1
output_folder=$2
interval=$3
shift 3
time_stamps=($@)

# 创建输出文件夹
mkdir -p "$output_folder"

# 检查输入文件是否存在
if [ ! -f "$input_bag" ]; then
    echo "输入袋文件不存在: $input_bag"
    exit 1
fi

# 拆分袋文件
for (( i=0; i<${#time_stamps[@]}; i++ )); do
    start=${time_stamps[$i]}
    end=$(echo "$start + $interval" | bc)
    output_bag="$output_folder/output_$(printf "%04d" $i).bag"

    # 使用 rosbag filter 命令拆分袋文件
    echo "Processing split $i: $output_bag (from $start to $end)"
    rosbag filter "$input_bag" "$output_bag" "t.secs >= $start and t.secs < $end"
done

echo "拆分完成，文件保存在 $output_folder 文件夹下"