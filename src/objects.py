
import numpy as np
import tf
import rospy
from geometry_msgs.msg import Point, Vector3Stamped, Vector3, PointStamped
from std_msgs.msg import Header
import copy
class RealObject:
    def __init__(self, position = np.zeros(3), basis = np.zeros((3,3)), sizes = (0, 0, 0), frame = "") -> None:
        self.position = np.array(position)
        self.frame = frame
        self.basis = np.array(basis)
        self.sizes = sizes
        self.tf_listner = tf.TransformListener()

    def TransformToFrame(self, time, frame : str, inplace = False):
        def asVectorS(v, time, frame):
            vec3 = Vector3(*v)
            header = Header(stamp = time, frame_id = frame)
            vec3s = Vector3Stamped(vector = vec3, header = header)
            return vec3s
        def asPointS(p, time, frame):
            ros_point = Point(*p)
            header = Header(stamp = time, frame_id = frame)
            pS = PointStamped(point = ros_point, header = header)
            return pS
        if self.tf_listner.frameExists(frame) and self.tf_listner.frameExists(self.frame) and  self.tf_listner.canTransform(frame, self.frame, time):
            new_basis_vec3 = [self.tf_listner.transformVector3(frame, asVectorS(v, time, self.frame)) for v in self.basis]
            new_pos_Point = self.tf_listner.transformPoint(frame, asPointS(self.position, time, self.frame))
            if inplace:
                self.basis = [ np.array([v.point.x, v.point.y, v.point.z]) for v in new_basis_vec3 ]
                self.position = np.array([new_pos_Point.point.x, new_pos_Point.point.y, new_pos_Point.point.z])
                self.frame = frame
                return self
            else:
                new_obj = RealObject()
                new_obj.sizes = copy.copy(self.sizes)
                new_obj.basis = [ np.array([v.point.x, v.point.y, v.point.z]) for v in new_basis_vec3 ]
                new_obj.position = np.array([new_pos_Point.point.x, new_pos_Point.point.y, new_pos_Point.point.z])
                new_obj.frame = frame
                return new_obj
                
class Plane:
    def __init__(self, base_point = np.zeros(3), normal = np.zeros(3)):
        self.base_point = base_point
        self.normal = normal
        self.IsPointUpper_vec = np.vectorize(self.IsPointUpper)
        self.ProjectPoint_vec = np.vectorize(self.ProjectPoint)
        self.Dist_vec = np.vectorize(self.Dist)
    def FromPoints(self, p1, p2, p3):
        self.normal = np.cross((p3 - p1)/np.linalg.norm(p3 - p1), (p2 - p1)/np.linalg.norm(p2 - p1))
        self.normal/=np.linalg.norm(self.normal)
        self.base_point = copy.copy(p1)
        return self

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

class Box(RealObject):
    def __init__(self, position=np.zeros(3), basis=np.zeros((3, 3)), sizes=(0, 0, 0), frame="", plane : Plane = None) -> None:
        super().__init__(position, basis, sizes, frame)
        self.pointcloud : np.ndarray = None
        self.plane : Plane = plane

    def TransformToFrame(self, time, frame, inplace = False):
        rs : Box =  RealObject.TransformToFrame(self, time, frame, inplace= inplace) 
        return rs


class Table(RealObject):
    def __init__(self, position=np.zeros(3), basis=np.zeros((3, 3)), sizes=(0, 0, 0), frame="") -> None:
        super().__init__(position, basis, sizes, frame)