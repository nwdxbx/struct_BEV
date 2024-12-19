# from rosbags.rosbag2 import Reader
# from rosbags.serde import deserialize_cdr

# with Reader('/media/xubenxiang/L4TeamShare/Datasets/C03/rosbags/0912_lslidar_bag/rosbag2_2023_03_02-21_39_50') as reader:
# 	connections = [x for x in reader.connections if x.topic == '/cloud_data']
# 	for connection, timestamp, rawdata in reader.messages(connections=connections):
# 		msg = deserialize_cdr(rawdata, connection.msgtype)
# 		import pdb; pdb.set_trace()
# 		print(msg.header.frame_id)

# from pathlib import Path

# from rosbags.highlevel import AnyReader

# # create reader instance and open for reading
# with AnyReader([Path('/media/xubenxiang/L4TeamShare/Datasets/C03/rosbags/0912_lslidar_bag/rosbag2_2023_03_02-21_39_50')]) as reader:
# 	connections = [x for x in reader.connections if x.topic == '/cloud_data']
# 	for connection, timestamp, rawdata in reader.messages(connections=connections):
# 		msg = reader.deserialize(rawdata, connection.msgtype)
# 		import pdb; pdb.set_trace()
# 		print(msg.header.frame_id)
