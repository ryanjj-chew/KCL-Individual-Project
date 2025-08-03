import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, TransformStamped
import joint_state_publisher
from sensor_msgs.msg import JointState
import numpy as np
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.time import Time
# import tf_transformations as tft
from scipy.spatial.transform import Rotation as R
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class Control(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.tf = None

        self.wrist_joints_index = [3,4,5]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        # MoveIt
        

        self.create_subscription(Vector3, '/direction', self.control_callback, 10)
        self.create_publisher(JointState, '/isaac/joint_command', 10)

    def control_callback(self, msg: Vector3):
        try:
            x, y, z = msg.x, msg.y, msg.z

            vector = np.array([x,y,z])
            direction = vector / np.linalg.norm(vector)

            # Ensure end effector roll is fixed
            up_direction = np.array([0,1,0])
            z_axis = direction
            x_axis = np.cross(up_direction, z_axis)
            x_axis /= np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= np.linalg.norm(y_axis)

            rot_mat = np.vstack([x_axis, y_axis, z_axis]).T
            qx, qy, qz, qw = R.from_matrix(rot_mat).as_quat()

            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'wrist_3_link'
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)

            goal = PoseStamped()
            goal.header = t.header
            goal.header.frame_id = 'base_link'
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw

            # MoveIt Plan & Move
            
            
        except:
            return
            
            
    

def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()