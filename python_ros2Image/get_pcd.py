import rospy
import pclpy
from pclpy import pcl
from sensor_msgs.msg import PointCloud2
def pointcloud_callback(msg):
    print("pointcloud_callback")
    cloud = pcl.PointCloud.PointXYZI()
    pcl.io.fromROSMsg(msg, cloud)
    save_path = '/home/yzp/wx/rosbag/c03/pcd/' + str(msg.header.stamp.secs) + '_' + str(msg.header.stamp.nsecs) + '.pcd'
    pcl.io.savePCD(save_path, cloud, binary=False)

rospy.init_node('pointcloud_subscriber')
rospy.Subscriber("perception/sensing/lidar/undistortion/0/pointcloud", PointCloud2, pointcloud_callback)
rospy.spin()