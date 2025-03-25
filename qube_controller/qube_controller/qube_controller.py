import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class PIDController:
    def __init__(self):
        pass

    def update(self, setpoint, current_value, dt):
        pass

class QubeController(Node):

    def __init__(self):
        super().__init__('qube_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'velocity_controller/command', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        joint_velocities = msg.velocity


def main(args=None):
    rclpy.init(args=args)

    qube_controller = QubeController()

    rclpy.spin(qube_controller)

    qube_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()