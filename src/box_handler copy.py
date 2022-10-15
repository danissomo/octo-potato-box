#! /usr/bin/env python3
from point_cloud import Realsense2PointCloud
from aruco_localization.msg import aruco_msg
import rospy
from arguments import Args
import objects
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
class BoxHandler(Realsense2PointCloud):
    def __init__(self):
        super().__init__(frame="rs_camera")
        self.aruco_sub = rospy.Subscriber(Args.aruco_tn, aruco_msg, self.aruco_callback)
        self.box : objects.Box = None
        self.table : objects.Table = None
        self.box_xy_plane : objects.Plane = None
        self.table_xy_plane : objects.Plane = None
        self.box_ids = (1, 2, 3)
        self.box_pub = rospy.Publisher("/box/pc", PointCloud)

    
    def aruco_callback(self, ar_msg : aruco_msg):
        if not  ar_msg.arucoId in self.box_ids:
            return
        '''
        aruco 
                 y   
            0---->1
            |.....|
          x v.....|
            3---->2
        '''
        p32_as_np = lambda p: np.array([p.x, p.y, p.z])
        points_as_np = [p32_as_np(v) for v in ar_msg.points]
        x_vec = points_as_np[3] - points_as_np[0]
        y_vec = points_as_np[1] - points_as_np[0]
        z_vec = np.cross(x_vec, y_vec)
        box  =objects.Box(
            position = np.mean(points_as_np),
            basis = np.array( [x_vec, y_vec, z_vec]),
            frame="rs_camera",
            sizes=(0, 0, 0), 
            plane=objects.Plane().FromPoints(*points_as_np[:3])
        )
        self.box = box.TransformToFrame(ar_msg.timeTag, "ur_arm_base", False)
        pc_lower_then_box = self.cv_pc[ not self.box.plane.IsPointUpper_vec(self.cv_pc)]

        table_points = pc_lower_then_box[self.box.plane.Dist_vec(pc_lower_then_box) > 0.03 & self.box.plane.Dist_vec(pc_lower_then_box) < 0.5]
        rand_points = table_points[np.random.randint(table_points.size[0])[:3]]
        table_plane = objects.Plane().FromPoints(*rand_points)
        box_point_cloud = pc_lower_then_box[table_plane.IsPointUpper_vec(pc_lower_then_box)]
        pointcloud = PointCloud()
        pointcloud.header.frame_id = "map"
        pointcloud.points = self.v_func(box_point_cloud)
        self.box_pub.publish(pointcloud)
        #pc_cv = self.v_func(box_point_cloud)

if __name__ == "__main__":
    rospy.init_node('test_box_pc')
    from door_tests import Robot
    r = Robot("192.16.131.40")
    bh = BoxHandler()
    
    rospy.spin()