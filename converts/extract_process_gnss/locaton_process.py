import os
import math
import numpy as np

from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

# Quaternion(r1).yaw_pitch_roll
# R.from_matrix(Quaternion(r1).rotation_matrix).as_euler('zyx')
def euler2quat(euler):	#[yaw, pitch, roll]
	return R.from_euler('zyx', euler, degrees=False).as_quat()

def quat2euler(quat):
	return R.from_quat(quat).as_euler('zyx', degrees=False)

def normalize_radian(radian):
    return (radian + math.pi) % (2 * math.pi) - math.pi

# def euler2quat1(yaw, pitch, roll):
# 	cy = np.cos(yaw*0.5)
# 	sy = np.sin(yaw*0.5)
# 	cp = np.cos(pitch*0.5)
# 	sp = np.sin(pitch*0.5)
# 	cr = np.cos(roll*0.5)
# 	sr = np.sin(roll*0.5)

# 	return np.array([cr * cp * cy + sr * sp * sy,
# 					sr * cp * cy - cr * sp * sy,
# 					cr * sp * cy + sr * cp * sy,
# 					cr * sp * cy + sr * cp * sy])

def euler_to_quaternion(yaw, pitch, roll):
	q = Quaternion(axis=[1, 0, 0], angle=roll) * \
		Quaternion(axis=[0, 1, 0], angle=pitch) * \
		Quaternion(axis=[0, 0, 1], angle=yaw)
	return q

# def quaternion_to_euler(q):
#     roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
#     pitch = math.asin(2 * (q.w * q.y - q.x * q.z))
#     yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
#     return roll, pitch, yaw


def interpolate_radian(radian_start, radian_end, time_start, time_end, time_current):
    # 标准化弧度值
    radian_start = normalize_radian(radian_start)
    radian_end = normalize_radian(radian_end)
    import ipdb; ipdb.set_trace()
    # 计算差值
    radian_diff = radian_end - radian_start
    if radian_diff > math.pi:
        radian_diff -= 2 * math.pi
    elif radian_diff < -math.pi:
        radian_diff += 2 * math.pi
    
    # 插值
    radian_interp = radian_start + radian_diff * (time_current - time_start) / (time_end - time_start)
    
    # 调整结果
    radian_interp = normalize_radian(radian_interp)
    
    return radian_interp


def interpolate_pos(t0, t, t1, trans_c0, trans_c1, rot0, rot1):
	# trans_c = np.interp(t, [t0, t1], [trans_c0, trans_c1]) # 插值位置
	trans_c = [np.interp(t, [t0, t1], [trans_c0[i], trans_c1[i]]) for i in range(3)]
	q = Quaternion.slerp(q0=Quaternion(rot0), q1=Quaternion(rot1), amount=(t-t0)/(t1-t0)) # 插值姿态

	return trans_c, q


def posture_interpolation(start_pyr, start_xyz, start_stamp, end_pyr, end_xyz, end_stamp, cur_stamp):
	if np.abs(cur_stamp - start_stamp)<1e-6:
		return start_xyz, start_pyr
	if abs(end_stamp - cur_stamp)<1e-6:
		return end_xyz, end_pyr
	
	cur_xyz = (end_stamp - cur_stamp)/(end_stamp - start_stamp)*start_xyz \
			+ (cur_stamp - start_stamp)/(end_stamp - start_stamp)*end_xyz

	start_pitch, start_yaw, start_roll = start_pyr
	end_pitch, end_yaw, end_roll = end_pyr
	
	cur_pitch = interpolate_radian(start_pitch, end_pitch, start_stamp, end_stamp, cur_stamp)
	cur_yaw   = interpolate_radian(start_yaw, end_yaw, start_stamp, end_stamp, cur_stamp)
	cur_roll  = interpolate_radian(start_roll, end_roll, start_stamp, end_stamp, cur_stamp)

	return cur_xyz, (cur_pitch, cur_yaw, cur_roll)

def relative_pos(start_translation, start_rotation, cur_translation, cur_rotation):
	"""
	Pw = R1P1 + t1	
	Pw = R2P2 + t2
	R1, t1为start时刻，目标相对世界坐标系的位姿， 同理R2, t2为cur时刻，目标相对于世界坐标系的位姿
	P1 = R1T R2 P2 +R1T (t2-t1)
	"""
	start_translation = np.array(start_translation).reshape(3,1)
	cur_translation = np.array(cur_translation).reshape(3,1)
	rel_rot = start_rotation.T*cur_rotation
	rel_trans = start_rotation.T @ (cur_translation - start_translation)

	rel_trans = rel_trans.reshape(3).tolist()
	return rel_rot, rel_trans



if __name__ == '__main__':
	yaw = 30/180*np.pi
	pitch = 50/180*np.pi
	roll = 135/180*np.pi
	q1 = euler_to_quaternion(yaw, pitch, roll)
	# r1, p1, y1 = quaternion_to_euler(q1)
	y2, p2, r2 = q1.yaw_pitch_roll
	import ipdb; ipdb.set_trace()

	# 四元数旋转矩阵相互转换
	link2caml = np.array([-0.0445445,0.998705,-0.0245634,
						0.045696,-0.0225252,-0.998701,
						-0.997962,-0.0456091,-0.0446334]).reshape(3, 3)
	q_link2caml = Quaternion(matrix=link2caml, rtol=1e-06, atol=1e-06)
	link2caml1 = q_link2caml.rotation_matrix
	q_link2caml1 = Quaternion(matrix=link2caml1, rtol=1e-06, atol=1e-06)

	link2camr = np.array([0.00186151,0.999993,0.00313965,
							0.0736859,0.00299395,-0.997277,
							-0.99728,0.00208779,-0.0736798]).reshape(3, 3)
	q_link2camr = Quaternion(matrix=link2camr, rtol=1e-06, atol=1e-06)
	link2camr1 = q_link2camr.rotation_matrix
	q_link2camr1 = Quaternion(matrix=link2camr1, rtol=1e-06, atol=1e-06)

	link2lidar = np.array([0,-1,0,
							1,0,0,
							0,0,1]).reshape(3, 3)
	q_link2lidar = Quaternion(matrix=link2lidar, rtol=1e-06, atol=1e-06)
	link2lidar1 = q_link2lidar.rotation_matrix
	q_link2lidar1 = Quaternion(matrix=link2lidar1, rtol=1e-06, atol=1e-06)
	import ipdb; ipdb.set_trace()
	print("ok...")


	#四元数表示旋转
	Xc = np.array([4, 5, 6])
	Xw = link2caml.dot(Xc)
	Xw_rot = q_link2caml.rotate(Xc)

	#四元数的逆, 
	q_link2caml_inv = q_link2caml.inverse
	q_link2caml_inv1 = Quaternion(matrix=link2caml.T, rtol=1e-06, atol=1e-06)
	q_link2caml_inv.rotation_matrix == q_link2caml_inv1.rotation_matrix
	q_link2caml_inv.yaw_pitch_roll == q_link2caml_inv1.yaw_pitch_roll