import rospy
from visualization_msgs.msg import Marker
import threading
import math
from interfaces.threadPublisher import ThreadPublisher
from scipy.spatial.transform import Rotation


def vector2quat(marker, data):
    theta = math.atan2(data[0], data[1])
    rot = Rotation.from_euler('xyz', [0, 0, theta]).as_quat()

    marker.scale.x = math.sqrt(data[0] ** 2 + data[1] ** 2)

    if marker.scale.x < 1:
        marker.scale.x = 1
    
    if marker.scale.x > 4:
        marker.scale.x = 4

    marker.pose.orientation.x = rot[0]
    marker.pose.orientation.y = rot[1]
    marker.pose.orientation.z = rot[2]
    marker.pose.orientation.w = rot[3]

    return marker

class Arrow:
    def __init__(self, id, color, orientation):
        self.pub = rospy.Publisher(f"/vff_marker_{id}", Marker, queue_size = 10)
        
        self.lock = threading.Lock()
        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)
        
        self.marker = self.create_marker(id, color, orientation)
        self.data = [0, 0]

        self.thread.daemon = True
        self.start()

    def publish(self):
        self.lock.acquire()
        tw = vector2quat(self.marker, self.data)
        self.lock.release()
        self.pub.publish(tw)

    def stop(self):
        self.kill_event.set()
        self.pub.unregister()

    def start (self):
        self.kill_event.clear()
        self.thread.start()

    def send_vector(self, vector):
        self.lock.acquire()
        self.data = vector
        self.lock.release()

    def create_marker(self, id, color, orientation):
        marker = Marker()

        marker.header.frame_id = "f1_renault__f1"
        marker.header.stamp = rospy.Time.now()
        
        marker.type = 0
        marker.id = id

        marker.scale.x = 0.1
        marker.scale.y = 0.6
        marker.scale.z = 0.6

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        return marker


class Sphere:
    def __init__(self, id):
        self.pub = rospy.Publisher(f"/vff_marker_{id}", Marker, queue_size = 2)
        
        self.lock = threading.Lock()
        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)
        
        self.marker = self.create_marker(id)
        self.data = [0, 0]

        self.thread.daemon = True
        self.start()

    def publish(self):
        self.lock.acquire()
        self.marker.pose.position.x = self.data[0]
        self.marker.pose.position.y = self.data[1]
        self.lock.release()
        self.pub.publish(self.marker)

    def stop(self):
        self.kill_event.set()
        self.pub.unregister()

    def start (self):
        self.kill_event.clear()
        self.thread.start()

    def send_vector(self, vector):
        self.lock.acquire()
        self.data = vector
        self.lock.release()

    def create_marker(self, id):
        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        
        marker.type = 2
        marker.id = id

        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker