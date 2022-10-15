#! /usr/bin/env python3
from point_cloud import Realsense2PointCloud
from aruco_localization.msg import aruco_msg
import rospy
from arguments import Args
import objects
import numpy as np
from visualization_msgs.msg import Marker
import copy
from sensor_msgs.msg import PointCloud
class BoxHandler(Realsense2PointCloud):
    def __init__(self):
        super().__init__(frame="map")
        self.aruco_sub = rospy.Subscriber(Args.aruco_tn, aruco_msg, self.aruco_callback, queue_size = 2)
        self.box : objects.Box = None
        self.table : objects.Table = None
        self.box_xy_plane : objects.Plane = None
        self.table_xy_plane : objects.Plane = None
        self.box_ids = (1, 2, 3)
        self.box_pub = rospy.Publisher("/box/pc", PointCloud, queue_size=10)

    
    def aruco_callback(self, ar_msg : aruco_msg):
        if not  ar_msg.arucoId in self.box_ids:
            return
        if self.semaphore:
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
        if self.cv_pc is None:
            rospy.sleep(1)
            rospy.loginfo("none")
            return
    
        point_cloud_cp = self.cv_pc.copy()
        dist =  ((point_cloud_cp-box.plane.base_point) @ box.plane.normal)
        pc_lower_then_box = point_cloud_cp[ dist <= 0.02]

        dist_2 = np.abs((pc_lower_then_box - box.plane.base_point) @ box.plane.normal)
        table_points = pc_lower_then_box[ (dist_2 > 0.03) & (dist_2 < 0.25)]

        rnd_indexes = [np.random.randint(table_points.shape[0]) for i in range(3)]
        rand_points = np.array([
            np.mean(table_points[r:r+6], axis=0)
            for r in rnd_indexes
        ])
        table_plane = objects.Plane().FromPoints(*rand_points)
        if table_plane.normal @ box.plane.normal < 0:
            table_plane.normal *= -1
        i = 0
        while np.abs( table_plane.normal @ box.plane.normal ) <= 0.98:
            rand_points = np.array([
            np.mean(table_points[r:r+6], axis=0)
            for r in rnd_indexes
            ])
            table_plane = objects.Plane().FromPoints(*rand_points)
            i+=1
            if i > 20: return

        
        box_point_cloud = pc_lower_then_box[((pc_lower_then_box - table_plane.base_point) @ table_plane.normal) >= 0]
        box_plane_points = point_cloud_cp[np.abs((point_cloud_cp - box.plane.base_point) @ box.plane.normal) <= 0.05]
        print(box_plane_points.shape, box_point_cloud.shape)
        dist_to_box = np.abs((box.position - table_plane.base_point) @ table_plane.normal)
        buf = box_plane_points.copy()
        print(dist_to_box)
        print(np.linalg.norm(box.plane.normal))
        for t in np.linspace(0, 1, 5):
            reconstructed_points = buf  - t*0.05*box.plane.normal
            box_plane_points =  np.concatenate([reconstructed_points, box_plane_points]) 
        
        pointcloud = PointCloud()
        pointcloud.header.frame_id = "map"
        pointcloud.points = self.v_func(*(-box_plane_points.T))
        #self.box = box.TransformToFrame(ar_msg.timeTag, "ur_arm_base", False)
        self.box_pub.publish(pointcloud)
        #pc_cv = self.v_func(box_point_cloud)

if __name__ == "__main__":
    rospy.init_node('test_box_pc')
    from husky import Robot
    r = Robot("192.168.131.40")
    bh = BoxHandler()
    
    rospy.spin()