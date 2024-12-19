#!/bin/bash
rosbag filter yangjituopan.bag tuopan_filter1.bag "t.to_sec() > 1723865400.786164 and t.to_sec() < 1723865420.786164"
rosbag filter yangjituopan.bag tuopan_filter2.bag "t.to_sec() > 1723865460.790788 and t.to_sec() < 1723865480.790788"