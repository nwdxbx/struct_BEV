

# 输入的 rosbag 文件
# BAG_DIR=$1

source_dir="/home/zhengyim/data_for_bev/split_bag"

BAG_DIR="/home/zhengyim/data_for_bev/split_bag/output_0000"
# 点云话题
# TOPIC_NAME=$2
TOPIC_NAME="/perception/sensing/lidar/preprocessor/pointcloud"

# 输出目录
# OUTPUT_DIR=$3
OUTPUT_DATA_DIR="$source_dir/Result/Data_lidar_cam_test"
CALIB_FILE_DIR="./camera_params.json"
OUTPUT_KOTEI_DIR="$source_dir/Result/struct-BEV_test"

source /opt/ros/noetic/setup.bash

# 记录开始时间
start_data_time=$(date +%s)

# 创建输出目录

# 检查给定的路径是否为有效的目录
if [ ! -d "$OUTPUT_DATA_DIR" ]; then
    echo "Error: $OUTPUT_DATA_DIR is not a valid directory."
    mkdir $OUTPUT_DATA_DIR
fi
if [ ! -d "$OUTPUT_KOTEI_DIR" ]; then
    echo "Error: $OUTPUT_KOTEI_DIR is not a valid directory."
    mkdir $OUTPUT_KOTEI_DIR
fi


# 遍历指定目录下的所有文件和子目录,提取pcd文件
for FILE in $(find "$BAG_DIR" -type f); do
    echo "File: $FILE"
        PACKAGE_NAME=$(basename "$FILE" .bag)
        TMP_LIDAR_DIR="$OUTPUT_DATA_DIR/$PACKAGE_NAME/lidar/lidar_front_tmp"
        FINAL_LIDAR_DIR="$OUTPUT_DATA_DIR/$PACKAGE_NAME/lidar/lidar_front"
        mkdir -p $FINAL_LIDAR_DIR &
        mkdir -p $TMP_LIDAR_DIR &
        sleep 3
        echo "package_name: $PACKAGE_NAME"
        # 提取点云
        rosrun pcl_ros bag_to_pcd $FILE $TOPIC_NAME $TMP_LIDAR_DIR
        sleep 5
        echo "Waiting for pcd conversion..."
        python3 rename.py --input_dir $TMP_LIDAR_DIR --out_dir $FINAL_LIDAR_DIR
        echo "Pcd conversion done."
        sleep 5
        rm -r $TMP_LIDAR_DIR
done


# sleep 3

# 提取图像
topics=(
    '/driver/camera/front_l/image/compressed' 
    '/driver/camera/front_s/image/compressed' 
    '/driver/camera/left_back/image/compressed' 
    '/driver/camera/left_front/image/compressed'
    '/driver/camera/right_back/image/compressed'
    '/driver/camera/right_front/image/compressed'
    '/driver/camera/back/image/compressed'
)
topics_str="${topics[@]}"
cam_names=(
    'front_l'
    'front_s'
    'left_back'
    'left_front'
    'right_back'
    'right_front'
    'back'
)
cam_names_str="${cam_names[@]}"

# echo "cam_names_str : $cam_names_str"
# echo "OUTPUT_DATA_DIR : $OUTPUT_DATA_DIR"
echo "BAG_DIR : $BAG_DIR"
# echo "CALIB_FILE_DIR : $CALIB_FILE_DIR"
# echo "topics_str : $topics_str"


echo "Waiting for image conversion..."
python3 extract_all_img_pcd.py --out_dir $OUTPUT_DATA_DIR \
--bagdir $BAG_DIR \
--json_file_dir $CALIB_FILE_DIR \
--topics  $topics_str \
--save_names $cam_names_str
echo "Image conversion done."

# 记录所有数据提取结束时间
end_data_time=$(date +%s)

# #--scenes_dir tuopan_filter1_lidarAxis tuopan_filter2_lidarAxis \
echo "Waiting for sync data..."
# #同步数据并生成最终目录
start_sync_time=$(date +%s)



python3 convert_sus.py --data_dir $OUTPUT_DATA_DIR \
--scenes_dir output_0000 \
--lidar_freq 10 \
--out_dir $OUTPUT_KOTEI_DIR \
--main_channel lidar_front 


end_sync_time=$(date +%s)

# 计算花费的时间
elapsed_data_time=$((end_data_time - start_data_time))
elapsed_sync_time=$((end_sync_time - start_sync_time))

# 打印花费的时间
echo "Total data time spent: $elapsed_data_time seconds"
echo "Total sync time spent: $elapsed_sync_time seconds"