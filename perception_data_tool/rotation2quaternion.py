from pyquaternion import Quaternion
import numpy as np
# gnss2cam
old2left = np.array([-0.0445445,0.998705,-0.0245634,-1.44962,
				0.045696,-0.0225252,-0.998701,0.460355,
				-0.997962,-0.0456091,-0.0446334,-8.02168,
				0,0,0,1]).reshape(4, 4)[:3, :3]
old2right = np.array([0.00186151,0.999993,0.00313965,1.05577,
            0.0736859,0.00299395,-0.997277,0.580209,
            -0.99728,0.00208779,-0.0736798,-8.05667,
            0,0,0,1]).reshape(4, 4)[:3, :3]
old2back = np.array([0.00592227,0.999533,0.0299676,-0.012956,
            0.281306,0.0270926,-0.959236,2.50915,
            -0.9596,0.0141109,-0.281015,-1.15133,
            0,0,0,1]).reshape(4, 4)[:3, :3]

q_old_left = Quaternion(matrix=old2left, rtol=1e-06, atol=1e-06)
q_old_right = Quaternion(matrix=old2right, rtol=1e-06, atol=1e-06)
q_old_back = Quaternion(matrix=old2back, rtol=1e-06, atol=1e-06)

print("---------------------------")
print(q_old_left)
print("---------------------------")
print(q_old_right)
print("---------------------------")
print(q_old_back)

#  lidar2cam
flidar2yangji_cambl = np.array([0.998705,0.0445445,-0.0245634,-1.43215,
            -0.0225252,-0.045696,-0.998701,-1.75179,
            -0.0456091,0.997962,-0.0446334,-6.54654,
            -0,-0,0,1]).reshape(4, 4)[:3, :3]
yangji_flidar2cambr = np.array([0.999993,-0.00186151,0.00313965,1.05957,
            0.00299395,-0.0736859,-0.997277,-1.67294,
            0.00208779,0.99728,-0.0736798,-6.64484,
            -0,0,0,1]).reshape(4, 4)[:3, :3]
yangjiflidar2camback = np.array([9.9981506308815593e-01, -3.2730467088564813e-03,1.8950640814190938e-02, 4.9112993901468699e-02,
       1.7402049522908032e-02, -2.6546667373080574e-01,-9.6396297325711866e-01, -7.6614329818684726e-02,
       8.1858594190892392e-03, 9.6411448091165175e-01,-2.6536062142305011e-01, -2.4049554742972951e-01, 
       0., 0., 0., 1.]).reshape(4, 4)[:3, :3]


q_flidar2yangji_cambl = Quaternion(matrix=flidar2yangji_cambl, rtol=1e-06, atol=1e-06)
q_yangji_flidar2cambr = Quaternion(matrix=yangji_flidar2cambr, rtol=1e-06, atol=1e-06)
q_yangjiflidar2camback= Quaternion(matrix=yangjiflidar2camback, rtol=1e-06, atol=1e-06)

print("----------lidar2cam-----------------")
print(q_flidar2yangji_cambl)
print("---------------------------")
print(q_yangji_flidar2cambr)
print("---------------------------")
print(q_yangjiflidar2camback)


#  baselink2cam
yangji_baselink2cambl = np.array([-0.0445445,0.998705,-0.0245634,-1.19658,
            0.045696,-0.0225252,-0.998701,0.721936,
            -0.997962,-0.0456091,-0.0446334,-2.60997,
            0,0,0,1]).reshape(4, 4)[:3, :3]
yangji_baselink2cambr = np.array([0.00186151,0.999993,0.00313965,1.04412,
            0.0736859,0.00299395,-0.997277,0.689919,
            -0.99728,0.00208779,-0.0736798,-2.63385,
            0,0,0,1]).reshape(4, 4)[:3, :3]
yangjiflidar2camback = np.array([0.00327305,0.999815,0.0189506,-0.0136668,
            0.265467,0.017402,-0.963963,1.46414,
            -0.964114,0.00818586,-0.265361,4.15194,
            0,0,0,1]).reshape(4, 4)[:3, :3]


q_yangji_baselink2cambl = Quaternion(matrix=yangji_baselink2cambl, rtol=1e-06, atol=1e-06)
q_yangji_baselink2cambr = Quaternion(matrix=yangji_baselink2cambr, rtol=1e-06, atol=1e-06)
q_yangjiflidar2camback = Quaternion(matrix=yangjiflidar2camback, rtol=1e-06, atol=1e-06)

print("Baselink2cam-----------------")
print(q_yangji_baselink2cambl)
print("---------------------------")
print(q_yangji_baselink2cambr)
print("---------------------------")
print(q_yangjiflidar2camback)

#gnss2lidar
yangji_gnss2flidar = np.array([0,1,0,-0,
            -1,0,-0,-1.574,
            0,0,1,-2.143,
            -0,0,-0,1]).reshape(4, 4)[:3, :3]
yangji_baselink2lidar = np.array([0,1,0,-0,
            -1,0,-0,3.826,
            0,-0,1,-2.652,
            -0,0,-0,1]).reshape(4, 4)[:3, :3]

q_yangji_gnss2flidar = Quaternion(matrix=yangji_gnss2flidar, rtol=1e-06, atol=1e-06)
q_yangji_baselink2lidar = Quaternion(matrix=yangji_baselink2lidar, rtol=1e-06, atol=1e-06)

print("gnss2flidar-----------------")
print(q_yangji_gnss2flidar)
print("q_yangji_baselink2lidar---------------------------")
print(q_yangji_baselink2lidar)

