# -*- coding: utf-8 -*-
# created by: bxu
import os
import rosbag
import gnss_pb2
import argparse

def parse_opt():
    parser = argparse.ArgumentParser(description='extract gnss messages from rosbag')
    parser.add_argument('--out_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/L12/bev_data/data/Result/Data_lidar_cam_test/', \
        help='output directory')
    parser.add_argument('--bagdir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/L12/bev_data/data/bags/', \
        help='total bag dir, including sub_bag_dir1, sub_bag_dir2...')
    parser.add_argument('--bag_subdirs', nargs='+', type=str, default=[''], \
        help='bag sub directory list, eg. sub_bag_dir1')
    parser.add_argument(
    '--topics', nargs='+', type=str, default=[
    '/localization/gnss', 
    ], 
    help='rosbag topic(s)')
    parser.add_argument(
    '--input_bag', type=str, default='/home/xbx/1.bag',
    help='rosbag abs path')
    opt = parser.parse_args()
    return opt

# 根据精度的不同，分为 10 位（秒级），13 位（毫秒级），16 位（微妙级）和 19 位（纳秒级）。最常用的为13位和10位，python里一般默认为10位。

def parse_msg(bag_path, output_bag_path, opt):
    topics = opt.topics
    bag = rosbag.Bag(bag_path, "r")
    # parse_gnss_file = os.path.splitext(output_bag_path)[0] + "_gnss.txt"
    parse_gnss_file = os.path.splitext(output_bag_path)[0] + "gnss.txt"
    fw = open(parse_gnss_file, 'w')
    for topic_name in topics:
        bag_data = bag.read_messages(topic_name)
        for topic, msg, t in bag_data:
            # print("Message structure:", msg)  
            # print("Data type:", type(msg.data)) 
            # 将元组转换字节流
            if isinstance(msg.data, tuple):
                byte_data = bytes(byte if byte >= 0 else 256 + byte for byte in msg.data)
                gnss_data = gnss_pb2.Gnss()
                gnss_data.ParseFromString(byte_data)
                stamp_str = str(gnss_data.header.timestamp*1000)
                utm_x = gnss_data.x_utm
                utm_y = gnss_data.y_utm
                utm_z = gnss_data.z_utm
                utm_yaw = gnss_data.heading_utm
                utm_pitch = gnss_data.pitch
                utm_roll = gnss_data.roll
                res = stamp_str + " " + str(utm_x) + " " + str(utm_y) + " " + str(utm_z) \
                        + " " + str(utm_yaw) + " " + str(utm_pitch) + " " + str(utm_roll) + "\n"
                fw.write(res)
            else:
                print("Received data is not a tuple. Type of data received:", type(msg.data))
    fw.close()

def run(opt):
    for subdir in opt.bag_subdirs:
        bag_path = os.path.join(opt.bagdir, subdir)
        bagfiles = os.listdir(bag_path)
        for bagfile in bagfiles:
            if bagfile.endswith('.bag'):
                output_dir = os.path.join(opt.out_dir, subdir)
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                output_bag_path = os.path.join(output_dir, bagfile)
                parse_msg(os.path.join(bag_path, bagfile), output_bag_path, opt)

def run_bagfile(opt):
    output_dir = opt.out_dir
    pathbag = opt.input_bag
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    parse_msg(pathbag, output_dir, opt)


def main():
    opt = parse_opt()
    # run(opt)
    run_bagfile(opt)
    

if __name__ == '__main__':
    main()