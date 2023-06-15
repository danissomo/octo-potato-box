
import numpy as np
import tf
import rospy
from geometry_msgs.msg import Point, Vector3Stamped, Vector3, PointStamped, Quaternion
from fork_rostypes import m_Point, m_PointStamped, m_Vector3, m_Vector3Stamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import copy
from scipy.spatial.transform import Rotation


class RealObject:
    def __init__(self, position = np.zeros(3), basis = np.zeros((3,3)), sizes = (0, 0, 0), frame = "") -> None:
        self.position = np.array(position)
        self.frame = frame
        self.basis = np.array(basis)
        self.sizes = sizes
        self.tf_listner = tf.TransformListener()
        

    def TransformToFrame(self, time, frame, inplace = False):
        while not self.tf_listner.frameExists(frame) or not self.tf_listner.frameExists(self.frame):
            rospy.logdebug('wait for transform')
            rospy.sleep(0.5)
        
        time = rospy.Time(0)
        self.tf_listner.waitForTransform(self.frame, frame, time, rospy.Duration(4.0))
        if self.tf_listner.frameExists(frame) and self.tf_listner.frameExists(self.frame) and  self.tf_listner.canTransform(frame, self.frame, time):
            new_basis_vec3 = [self.tf_listner.transformVector3(frame,m_Vector3Stamped().from_np(v, self.frame, time)) for v in self.basis]
            new_pos_Point = self.tf_listner.transformPoint(frame, m_PointStamped().from_np(self.position, self.frame, time))
            if inplace:
                self.basis = [ np.array([v.vector.x, v.vector.y, v.vector.z]) for v in new_basis_vec3 ]
                self.position = np.array([new_pos_Point.point.x, new_pos_Point.point.y, new_pos_Point.point.z])
                self.frame = frame
                return self
            else:
                new_obj = RealObject()
                new_obj.sizes = copy.copy(self.sizes)
                new_obj.basis = [ np.array([v.vector.x, v.vector.y, v.vector.z]) for v in new_basis_vec3 ]
                new_obj.position = np.array([new_pos_Point.point.x, new_pos_Point.point.y, new_pos_Point.point.z])
                new_obj.frame = frame
                return new_obj

        rospy.logwarn("frame not found")

    def MakeMarker(self) -> Marker:
        m = Marker()
        m.action = Marker.ADD
        m.pose.position = m_Point().from_np(self.position)
        m.pose.orientation = Quaternion(*Rotation.from_matrix(self.basis).as_quat())
        m.header = Header(stamp = rospy.get_rostime(), frame_id = self.frame)
                
class Plane:
    def __init__(self, base_point = np.zeros(3), normal = np.zeros(3), frame_id = ""):
        self.base_point = base_point
        self.normal = normal
        self.frame_id = frame_id
        self.IsPointUpper_vec = np.vectorize(self.IsPointUpper)
        self.ProjectPoint_vec = np.vectorize(self.ProjectPoint)
        self.Dist_vec = np.vectorize(self.Dist)
        self.tf_listner = tf.TransformListener()

    def FromPoints(self, p1, p2, p3):
        self.normal = np.cross((p3 - p1)/np.linalg.norm(p3 - p1), (p2 - p1)/np.linalg.norm(p2 - p1))
        self.normal/=np.linalg.norm(self.normal)
        self.base_point = copy.copy(p1)
        return self

    def CrosPointCloud(self, pc):
        upper = pc[(pc - self.base_point) @ self.normal >= 0 ]
        lower = pc[(pc - self.base_point) @ self.normal <= 0 ]
        return upper, lower

    def MoveUp(self, delta, is_inplace = True):
        if is_inplace:
            self.base_point+= delta*self.normal
        else:
            return Plane(self.base_point + delta*self.normal, self.normal)
    def MoveDown(self, delta, is_inplace = True):
        if is_inplace:
            self.base_point -= delta*self.normal
            return self
        else:
            return Plane(self.base_point - delta*self.normal, self.normal)

    def ProjectPoint(self, point : np.ndarray):
        dist = (point - self.base_point ) @ self.normal
        return point - dist*self.normal

    def IsPointUpper(self, point):
        point = np.nan_to_num(point)
        try:
            return np.dot(point - self.base_point, self.normal) > 0 
        except:
            return False
    
    def Dist(self, point):
        return np.abs((point - self.base_point ) @ self.normal)

    def TransformToFrame(self, time, frame, inplace = False):
        if inplace:
            time = rospy.Time(0)
            self.tf_listner.waitForTransform(self.frame_id, frame, time, rospy.Duration(4.0))
            V = m_Vector3Stamped().from_np(self.normal, self.frame_id,  time)
            vs = self.tf_listner.transformVector3(frame,V)
            self.normal = m_Vector3Stamped(vs.header, vs.vector).as_np()
            p = m_PointStamped().from_np(self.base_point, self.frame_id, time)
            ps = self.tf_listner.transformPoint(frame,p)
            self.base_point = m_PointStamped(ps.header, ps.point).as_np()
            return self

class Box(RealObject):
    def __init__(self, position=np.zeros(3), basis=np.zeros((3, 3)), sizes=(0, 0, 0), frame="", plane : Plane = None, color = np.random.random(3)) -> None:
        super().__init__(position, basis, sizes, frame)
        self.top_plane : Plane = plane
        self.top_plane.frame_id = frame
        self.bottom_plane : Plane = None
        self.front_plane : Plane = None
        self.left_plane : Plane = None
        self.right_plane : Plane = None
        self.color = color

    def TransformToFrame(self, time, frame, inplace = False):
        rs : Box = RealObject.TransformToFrame(self, time, frame, inplace= inplace)
        rs.top_plane = rs.top_plane.TransformToFrame(time, frame, inplace)
        return rs
    
    def Makemarker(self) -> Marker:
        m = RealObject.MakeMarker(self)
        m.type = Marker.CUBE
        m.color = ColorRGBA(*self.color, 1)
        return m
    


class Table(RealObject):
    def __init__(self, position=np.zeros(3), basis=np.zeros((3, 3)), sizes=(0, 0, 0), frame="") -> None:
        super().__init__(position, basis, sizes, frame)