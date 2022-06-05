#!/usr/bin/env python
import struct
import rospy
from sensor_msgs.msg import PointCloud2

def callback_ptclud(ptcloud_data):
    print('-------------')
    assert isinstance(ptcloud_data, PointCloud2)
    index = (670*ptcloud_data.row_step) + (692*ptcloud_data.point_step)
    (X, Y, Z) = struct.unpack_from('fff', ptcloud_data.data, offset=index)
    print('x: ' + str(round(X,2)))
    print('y: ' + str(round(Y,2)))
    print('z: ' + str(round(Z,2)))

def listener():
    rospy.Subscriber("/camera/depth/points", PointCloud2, callback_ptclud)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("realsense_subscriber", anonymous=True)
    listener()
