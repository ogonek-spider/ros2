import rclpy
from rclpy.node import Node, Parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
import math

class Joint:
    def __init__(self, name, position, min=0, max=3.13, frequency=0):
        self.name = name
        self.position = position
        self.min = min
        self.max = max
        self.frequency = frequency
        self.phase = 0

    def recalculate_position(self):        
        if self.frequency != 0:
            self.phase += 2 * math.pi * self.frequency * self.timer_period

            if self.phase > 2 * math.pi:
                self.phase -= 2 * math.pi

            self.amplitude = (self.max - self.min) / 2
            self.offset = (self.max + self.min) / 2
            sine_value = self.amplitude * math.sin(self.phase) + self.offset
            self.position = max(self.min, min(self.max, sine_value))

class SinPositionGenerator(Node):
    def __init__(self):
        super().__init__('sinusoidal_joint_command')
        
        self.publisher_ = self.create_publisher(Float64MultiArray, '/spider_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )        

        self.joints = {
            '1-1': Joint('1-1', 1.5, 0.5, 2, 0.5),
            '1-2': Joint('1-2', 1.2, 1, 2.5, 0.3),
            '1-3': Joint('1-3', 3.13, 0, 3.13, 0)
        }
        self.joint_states_received = False
        
        # for joint in self.joints:
        #     self.declare_parameter(f'{joint.name}_min', joint.min)
        #     self.declare_parameter(f'{joint.name}_max', joint.max)
        #     self.declare_parameter(f'{joint.name}_frequency', joint.frequency)

        self.declare_parameter('publish_rate', 50.0)
        self.publish_rate = self.get_parameter('publish_rate').value        

        # Add parameter callback
#        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.phase = 0.0
                
        self.timer_period = 1.0 / self.publish_rate
        for joint in self.joints.values():
            joint.timer_period = self.timer_period
            joint.logger = self.get_logger()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("Sinusoidal Joint Command Node Started")

    def joint_state_callback(self, msg):
        # Update current joint positions from joint_state message
        #params_to_set = []
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joints and i < len(msg.position):
                self.joints[joint_name].position = msg.position[i]
                #params_to_set.append(Parameter(name=f'{joint_name}_position', value=msg.position[i]))
        # self.set_parameters(params_to_set)

        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info("Received first joint states message")


    def timer_callback(self):
        joint_positions = []
        for joint in self.joints.values():
            joint.recalculate_position()
            joint_positions.append(joint.position)

        msg = Float64MultiArray()        
        msg.data = joint_positions
        self.publisher_.publish(msg)
        
        # Log output occasionally for debugging
        self.get_logger().info(
            f'1-1: {joint_positions[0]:.3f}, 1-2: {joint_positions[1]:.3f}, 1-3: {joint_positions[2]:.3f}', 
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = SinPositionGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()