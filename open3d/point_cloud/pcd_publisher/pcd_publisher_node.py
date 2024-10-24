import sys
import os

import rclpy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from rclpy.node import Node

import numpy as np
import open3d as o3d
import struct

class PCDPublisher(Node):
    def __init__(self, voxel_size):
        super().__init__('pcd_publisher_node')

        assert len(sys.argv) > 1, "No ply file."
        assert os.path.exists(sys.argv[1]), "File doesn't exists."

        pcd_path = sys.argv[1]

        pcd = o3d.io.read_point_cloud(pcd_path)

        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)

        self.points = np.asarray(pcd.points)
        self.colors = np.asarray(pcd.colors)


        # print(self.points.shape)
        self.points = self.rotate_points_90(self.points)

        self.points[:, 2] += 2.5

        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'pcd', 10)
        timer_period = 1/30.0

        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):


        self.pcd = point_cloud(self.points, self.colors, 'map')

        self.pcd_publisher.publish(self.pcd)

    def rotate_points_90(self, points):
        """포인트 클라우드를 X축을 기준으로 90도 회전"""
        theta = np.radians(90)  # 90도 회전
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
        
        return np.dot(points, rotation_matrix)


# def point_cloud(points, parent_frame):
def point_cloud(points, colors, parent_frame):    
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

    """
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    # data = points.astype(dtype).tobytes() 


    # 포인트 클라우드 데이터 (xyz)
    data = []
    for i in range(points.shape[0]):
        x, y, z = points[i]
        r, g, b = colors[i]
        # 색상을 float32로 변환 (ROS에서의 rgb 값은 packed float32 형태로 사용됨)
        rgb = (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255)
        s = struct.pack('fffI', x, y, z, rgb)
        data.append(s)

    data = b''.join(data)  # 데이터를 바이트 배열로 결합    



    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    # fields = [sensor_msgs.PointField(
    #     name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
    #     for i, n in enumerate('xyz')]

    fields = [
        sensor_msgs.PointField(name='x', offset=0, datatype=ros_dtype, count=1),
        sensor_msgs.PointField(name='y', offset=4, datatype=ros_dtype, count=1),
        sensor_msgs.PointField(name='z', offset=8, datatype=ros_dtype, count=1),
        sensor_msgs.PointField(name='rgb', offset=12, datatype=sensor_msgs.PointField.UINT32, count=1),
    ]


    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    # return sensor_msgs.PointCloud2(
    #     header=header,
    #     height=1, 
    #     width=points.shape[0],
    #     is_dense=False,
    #     is_bigendian=False,
    #     fields=fields,
    #     point_step=(itemsize * 3), # Every point consists of three float32s.
    #     row_step=(itemsize * 3 * points.shape[0]),
    #     data=data
    # )

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=16,  # Every point consists of three float32s and one uint32 (xyz + rgb)
        row_step=16 * points.shape[0],
        data=data
    )    

def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)

    voxel_size = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0


    pcd_publisher = PCDPublisher(voxel_size)
    rclpy.spin(pcd_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()