import os
import shutil
import argparse

# srcdir = "/home/wangxin/Disk2/KOTEI标注/test_rosbag2/Data_lidar_cam"
# dstdir = "/home/wangxin/Disk2/KOTEI标注/test_rosbag2/Data_lidar_cam_rename"


# scenes_dir = os.listdir(srcdir)
# for scene_dir in scenes_dir:
#     scene_path = os.path.join(srcdir, scene_dir)
#     sensors_dir = os.path.join(scene_path)
#     for sensor_dir

def run_trans(opt):
    srcdir = opt.input_dir
    dstdir = opt.out_dir
    if not os.path.exists(dstdir):
        os.makedirs(dstdir)
    pcd_files = os.listdir(srcdir)
    for pcd_file in pcd_files:
        sec, ns, fix = pcd_file.split('.')
        dst_file = sec + ns + '.' + fix
        scrpath = os.path.join(srcdir, pcd_file)
        dstpath = os.path.join(dstdir, dst_file)
        # import ipdb; ipdb.set_trace()
        shutil.copyfile(scrpath, dstpath)
    # import ipdb; ipdb.set_trace()


def parse_opt():
    parser = argparse.ArgumentParser(description='trans pcd file name from rosTime to int')
    parser.add_argument('--input_dir', type=str, default='/home/wangxin/Disk2/KOTEI标注/test_rosbag2/Data_lidar_cam', \
        help='input directory')
    parser.add_argument('--out_dir', type=str, default='/home/wangxin/Disk2/KOTEI标注/test_rosbag2/Data_lidar_cam', \
        help='output directory')
    opt = parser.parse_args()
    return opt


def main():
    opt = parse_opt()
    run_trans(opt)

if __name__== "__main__":
    main()
