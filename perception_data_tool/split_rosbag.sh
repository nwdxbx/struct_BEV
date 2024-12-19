#!/bin/bash

# 输入袋文件
input_bag="tuopan_filter1_pre.bag"

# 输出文件夹
output_folder="split_bag"

# 时间间隔（秒）
interval=21

# 创建输出文件夹
mkdir -p "$output_folder"

# 获取袋文件的起始时间和结束时间
# start_time=$(rostopic echo -n 1 /rosout | grep "header.stamp.secs" | awk '{print $2}')
# end_time=$(rostopic echo -n 1 /rosout | grep "header.stamp.secs" | awk '{print $2}')

# 读取袋文件的起始时间和结束时间
start_time=$(rosbag info "$input_bag" | grep "start" | awk '{print $2}')
end_time=$(rosbag info "$input_bag" | grep "end" | awk '{print $2}')

# 计算总时间
total_time=$(echo "$end_time - $start_time" | bc)

# 计算需要拆分的次数
num_splits=$(echo "scale=0; ($total_time + $interval - 1) / $interval" | bc)

# 拆分袋文件
for (( i=0; i<$num_splits; i++ )); do
    start=$(echo "$start_time + $i * $interval" | bc)
    end=$(echo "$start + $interval" | bc)
    output_bag="$output_folder/output_$(printf "%04d" $i).bag"
    
    # 使用 rosbag filter 命令拆分袋文件
    rosbag filter "$input_bag" "$output_bag" "t.secs >= $start and t.secs < $end"
done

echo "拆分完成，文件保存在 $output_folder 文件夹下"