#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header


def callback(pointcloud_msg):
    # Crie uma nova mensagem CustomMsg
    custom_cloud = CustomMsg()

    # Preencha o cabeçalho da mensagem CustomMsg
    custom_cloud.header = Header()
    custom_cloud.header.stamp = pointcloud_msg.header.stamp
    custom_cloud.header.frame_id = pointcloud_msg.header.frame_id

    # Use o tempo da mensagem PointCloud2 como a base de tempo
    #print(pointcloud_msg.header.stamp.to_nsec()) #DEBUG
    custom_cloud.timebase = pointcloud_msg.header.stamp.to_nsec()

    # Extraia os pontos da PointCloud2
    points = pc2.read_points(pointcloud_msg, skip_nans=True)

    custom_points = []
    points_num = 0
    
    for point in points:
        # Crie um novo CustomPoint
        custom_point = CustomPoint()
        custom_point.offset_time = int(point[6] - pointcloud_msg.header.stamp.to_nsec())
        custom_point.x = point[0]
        custom_point.y = point[1]
        custom_point.z = point[2]
        custom_point.reflectivity = int(point[3])
        custom_point.tag = point[4]
        custom_point.line = point[5]
        
        custom_points.append(custom_point)
        points_num = points_num + 1

    # Preencha a mensagem CustomMsg
    custom_cloud.points = custom_points
    custom_cloud.point_num = points_num
    custom_cloud.lidar_id = 0  # Um exemplo de lidar_id

    # Publique a mensagem convertida
    pub.publish(custom_cloud)


def pointcloud_converter():
    rospy.init_node('pointcloud_converter', anonymous=True)

    # Assinar o tópico PointCloud2
    rospy.Subscriber("/livox/lidar", PointCloud2, callback)

    # Definir o publicador do novo formato CustomMsg
    global pub
    pub = rospy.Publisher('/livox/lidar_CustomMsg', CustomMsg, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        pointcloud_converter()
    except rospy.ROSInterruptException:
        pass
