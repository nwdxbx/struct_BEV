import os
import cv2
import time
import rosbag


import numpy as np
from tqdm import tqdm
from cv_bridge import CvBridge

class ExtractBagData(object):

    def __init__(self, bagfile_path, camera_topic, pointcloud_topic, root):
        self.bagfile_path = bagfile_path
        self.camera_topic = camera_topic
        self.pointcloud_topic = pointcloud_topic
        self.root = root
        self.image_dir = os.path.join(root, "images")
        self.pointcloud_dir = os.path.join(root, "pointcloud")

        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        if not os.path.exists(self.pointcloud_dir):
            os.makedirs(self.pointcloud_dir)

    def extract_camera_topic(self):
        bag = rosbag.Bag(self.bagfile_path, "r")
        bridge = CvBridge()
        bag_data_imgs = bag.read_messages(self.camera_topic)

        index = 0
        pbar = tqdm(bag_data_imgs)
        for topic, msg, t in pbar:
            pbar.set_description("Processing extract image id: %s" % (index+1))
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # timestr = "%.6f" %  msg.header.stamp.to_sec()
            cv2.imwrite(os.path.join(self.image_dir, str(index) + ".jpg"), cv_image)
            index += 1

    def extract_pointcloud_topic(self):
        bag = rosbag.Bag(self.bagfile_path, "r")
        bag_data_pointclouds = bag.read_messages(self.pointcloud_topic)
        for topic, msg, t in bag_data_pointclouds:
            import pdb; pdb.set_trace()
            np_pointcloud = np.fromstring(msg.data, dtype=np.float32)
            np_pointcloud = np_pointcloud.reshape(msg.height, msg.width, 4)
            np.save(os.path.join(self.pointcloud_dir, str(t) + ".npy"), np_pointcloud)

if __name__ == "__main__":
    bagfile_path = "/media/xubenxiang/L4TeamShare/Datasets/C03/rosbags/mutil_sensor/0710/2023-07-10-14-03-49.bag"
    camera_topic = "/driver/camera/front_l/image/compressed"
    pointcloud_topic = "/driver/lidar/front/pointcloud"
    root = "./"
    extbag = ExtractBagData(bagfile_path, camera_topic, pointcloud_topic, root)
    extbag.extract_pointcloud_topic()