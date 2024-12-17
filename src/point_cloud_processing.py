import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d

class keyCloud(Node):
    def __init__(self):
        super().__init__('keyPointCloud')
        self.sub_ = self.create_subscription('profiles', PointCloud2, self.callback, 10)
        self.sub_
    
    def callback(self, data):
        
class ViewerWidget(QtWidgets.QWidget):
    def __init__(self, subscriber, parent=None):
        self.subscriber = subscriber
        rospy.loginfo('initialization')

        self.vis = open3d.visualization.Visualizer()
        self.point_cloud = None
        self.updater()

    ############################################################################
    def updater(self):

        rospy.loginfo('start')
        self.first = False
        while (self.subscriber.pc is None):
            time.sleep(2)
        self.point_cloud = open3d.geometry.PointCloud()
        self.point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.subscriber.pc))
        self.vis.create_window()
        print('get points')
        self.vis.add_geometry(self.point_cloud)
        print ('add points')
        self.vis.update_geometry()
        self.vis.poll_events()

        self.vis.update_renderer()

        while not rospy.is_shutdown():
            self.point_cloud.points =  open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.subscriber.pc))
            self.vis.update_geometry()
            self.vis.poll_events()
            self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)

    key_cloud = keyCloud()
    updater = Viewer(key_cloud)

    rclpy.spin(key_cloud)

if __name__ == '__main__':
    main()