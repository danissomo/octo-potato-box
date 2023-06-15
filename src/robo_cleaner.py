#! /usr/bin/env python3
from point_cloud import Realsense2PointCloud
from aruco_localization_v2.msg import aruco_array_msg, aruco_msg
import rospy
from arguments import Args
import objects
import numpy as np
from visualization_msgs.msg import Marker
import copy
from sensor_msgs.msg import PointCloud
from husky import Robot
import time

from communication_msgs.srv import PickObject, PlaceObject
class BoxHandler(Realsense2PointCloud):
    def __init__(self):
        super().__init__(frame="rs_camera")
        self.aruco_sub = rospy.Subscriber(Args.aruco_tn, aruco_array_msg, self.aruco_callback, queue_size = 1)
        self.box : objects.Box = None
        self.table : objects.Table = None
        self.actualId = None
        self.box_ids = {i for i in range(15)}
        self.boxes = dict()
        self.box_pub = rospy.Publisher("/box/pc", PointCloud, queue_size=10)
        self.last_update = rospy.Time(0)
        
    def process_aruco_msg(self, ar_msg : aruco_msg):
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
        x_vec /= np.linalg.norm(x_vec)
        y_vec = points_as_np[1] - points_as_np[0]
        y_vec /= np.linalg.norm(y_vec)
        z_vec = np.cross(x_vec, y_vec)
        z_vec /= np.linalg.norm(z_vec)
        box  =objects.Box(
            position = np.mean(points_as_np, axis=0),
            basis = np.array( [x_vec, y_vec, z_vec]),
            frame="rs_camera",
            sizes=(0, 0, 0), 
            plane=objects.Plane().FromPoints(*points_as_np[:3])
        )
        # rospy.loginfo("%s ", box.position)
        br = tf.TransformBroadcaster()
        from scipy.spatial.transform import Rotation
        br.sendTransform(
                box.position,
                list(Rotation.from_matrix(box.basis).as_quat()),
                rospy.Time.now(),
                f"aruco_{ar_msg.arucoId}",
                "rs_camera",
            )
        return box
    
    def aruco_callback(self, ar_ar_msg = None):
        buf = dict()
        for ar_msg in ar_ar_msg.data:
            if not  ar_msg.arucoId in self.box_ids:
                continue
            box = self.process_aruco_msg(ar_msg)
            box =box.TransformToFrame(rospy.Time.now(), "ur_arm_base", True)
            # try:
            #     box =box.TransformToFrame(rospy.Time.now(), "ur_arm_base", True)
            # except:
            #     rospy.logerr('cant transform')
                # return False
            rospy.loginfo(box.position)
            buf[ar_msg.arucoId] = copy.copy(box)
        self.boxes = copy.copy(buf)
        self.last_update = rospy.Time.now()
        return True

    def get_aruco(self):
        t = rospy.Rospy.now()
        ar_msg = rospy.wait_for_message(Args.aruco_tn, aruco_msg)
        if not  ar_msg.arucoId in self.box_ids:
            return None
        self.actualId = ar_msg.arucoId
        self.box = self.process_aruco_msg(ar_msg)
        self.box = self.box.TransformToFrame(t, "ur_arm_base", True)
        rospy.loginfo(self.box.position)
        return copy.copy(self.box)
    
import tf
def TransformVec(vec, from_frame, to_frame,  tf_listner, time):
    from geometry_msgs.msg import Vector3Stamped, Vector3
    from std_msgs.msg import Header
    
    vec_s = Vector3Stamped(
        Header(
            stamp = time,
            frame_id = from_frame
        ),
        Vector3(*vec)
    )
    new_vec : Vector3Stamped =  tf_listner.transformVector3(to_frame, vec_s)
    return np.array([new_vec.vector.x, new_vec.vector.y, new_vec.vector.z])
    
def TransformPoint(point, from_frame, to_frame, tf_listner, time):
    from geometry_msgs.msg import PointStamped, Point
    from std_msgs.msg import Header
    point_s = PointStamped(
        Header(
            stamp = time,
            frame_id = from_frame
        ),
        Point(*point)
    )
    new_point : PointStamped=  tf_listner.transformPoint(to_frame, point_s)
    return np.array([new_point.point.x, new_point.point.y, new_point.point.z])


class Node():
    def __init__(self) -> None:
        rospy.init_node('object_operator', log_level=rospy.DEBUG)
        self.tf_listener = tf.TransformListener()
        self.robot = Robot("192.168.131.40")
        rospy.on_shutdown(self.cancel)
        self.robot.OpenGripper()
        self.robot.ActivateTeachMode()
        self.watch_on_floor_pose = [1.384433388710022, -1.9341991583453577, 1.6779465675354004, -1.3742497603045862, -1.5462692419635218, 0.004470490384846926]
        self.bh = BoxHandler()
        self.pick_srv = rospy.Service('pick_object', PickObject, self.cb_take_box)
        self.place_srv = rospy.Service('place_object', PlaceObject, self.cb_place_box_on_table)

    def cb_take_box(self, msg):
        self.robot.DeactivateTeachMode()
        self.robot.OpenGripper()
        self.robot.MoveJ(self.watch_on_floor_pose)
        rospy.sleep(3)
        wait_time = 4
        start_wait = time.time()
        while time.time() - start_wait < wait_time and rospy.Time.now() - self.bh.last_update > rospy.Duration(0.1):           
            rospy.sleep(1)
        if rospy.Time.now() - self.bh.last_update > rospy.Duration(wait_time):
            return 'no objects'
        cp_box = copy.copy(self.bh.boxes)
        key = list(cp_box)[0]
        cmd1 = cp_box[key].position  + 0.05 * cp_box[key].top_plane.normal
        cmd2 = cp_box[key].position
        cmd1 = list(cmd1) +  list(self.robot.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        cmd2 = list(cmd2) +  list(self.robot.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        self.robot.MoveL         (list(cmd1), 0.5, 0.5)
        self.robot.MoveL         (list(cmd2), 0.5, 0.5)
        rospy.logdebug(cmd1)
        rospy.logdebug(cp_box[key].position)
        rospy.logdebug(self.robot.GetActualTCPPose())
        self.robot.CloseGripper  ()
        self.robot.Fold()
        self.robot.MoveJ([1.5955524444580078, -2.8719032446490687, 2.799407482147217, -3.0721920172320765, -1.581970516835348, 0.00016777915880084038])
        self.robot.ActivateTeachMode()
        return  'success'

        
    def cb_place_box_on_table(self, msg):
        self.robot.DeactivateTeachMode()
        rospy.sleep(3)
        wait_time = 3
        start_wait = time.time()
        while time.time() - start_wait < wait_time and rospy.Time.now() - self.bh.last_update > rospy.Duration(1):           
            rospy.sleep(1)
        if rospy.Time.now() - self.bh.last_update > rospy.Duration(1):
            return 'no objects'
        self.robot.MoveJ( self.robot.INITIAL_JOINTS)
        cp_box = copy.copy(self.bh.boxes)
        key = list(cp_box)[0]
        cmd1 = cp_box[key].position  + [0, 0, 0.10]
        cmd2 = cp_box[key].position + [0, 0, 0.10]
        cmd3 = cp_box[key].position + [0, 0, 0.10] + 0.1 * cp_box[key].top_plane.normal
        cmd1 = list(cmd1) +  list(self.robot.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        cmd2 = list(cmd2) +  list(self.robot.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        cmd3 = list(cmd3) +  list(self.robot.RotvecFromBasis([-cp_box[key].basis[0], cp_box[key].basis[1], -cp_box[key].basis[2]]))
        self.robot.MoveL         (list(cmd1), 0.1, 0.1)
        self.robot.MoveL         (list(cmd2), 0.1, 0.1)
        self.robot.OpenGripper()
        self.robot.MoveL         (list(cmd3), 0.1, 0.1)
        
        rospy.logdebug(cmd1)
        rospy.logdebug(cp_box[key].position)
        rospy.logdebug(self.robot.GetActualTCPPose())
        self.robot.Fold()
        self.robot.MoveJ([1.5955524444580078, -2.8719032446490687, 2.799407482147217, -3.0721920172320765, -1.581970516835348, 0.00016777915880084038])
        self.robot.ActivateTeachMode()
        return 'success'

    def cancel(self):
        self.robot.ActivateTeachMode()
        self.robot.OpenGripper()


if __name__ == "__main__": 
    n = Node()
    rospy.spin()