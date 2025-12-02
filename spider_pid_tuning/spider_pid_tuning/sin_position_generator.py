import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
import math

class SinPositionGenerator(Node):
    def __init__(self):
        super().__init__('sinusoidal_joint_command')
        
        # Create publisher for joint commands
        self.publisher_ = self.create_publisher(Float64MultiArray, '/spider_controller/commands', 10)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Define available joints
        self.available_joints = ['1-1', '1-2', '1-3']
        
        # Current joint positions (initialize with zeros)
        self.current_joint_positions = {joint: 0.0 for joint in self.available_joints}
        self.joint_states_received = False
        
        # Declare parameters with ranges and descriptions
        # self.declare_parameter('amplitude', 1.0)
        # self.declare_parameter('offset', 0.5)
        self.declare_parameter('wavetype', 'sine') #[sine, square]
        self.declare_parameter('frequency', 0.05)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_to_control', '1-2')
        self.declare_parameter('min_position', 0.5)
        self.declare_parameter('max_position', 2.5)
        
        # Set parameter descriptors for better rqt_reconfigure display
        # param_descriptors = [
        #     self.ParameterDescriptor(description='Amplitude of the sine wave (half of peak-to-peak)'),
        #     self.ParameterDescriptor(description='Vertical offset to center in range'),
        #     self.ParameterDescriptor(description='Frequency of the sine wave in Hz'),
        #     self.ParameterDescriptor(description='Publishing rate in Hz'),
        #     self.ParameterDescriptor(description='Joint to control: 1-1, 1-2, or 1-3'),
        #     self.ParameterDescriptor(description='Minimum position in radians'),
        #     self.ParameterDescriptor(description='Maximum position in radians'),
        # ]
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Initialize phase
        self.phase = 0.0
        
        # Get initial parameter values
        # self.amplitude = self.get_parameter('amplitude').value
        # self.offset = self.get_parameter('offset').value
        self.wavetype = self.get_parameter('wavetype').value
        self.frequency = self.get_parameter('frequency').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joint_to_control = self.get_parameter('joint_to_control').value
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value

        self.amplitude = (self.max_position - self.min_position)/2
        self.offset = (self.max_position + self.min_position)/2
        
        
        # Validate joint_to_control parameter
        if self.joint_to_control not in self.available_joints:
            self.get_logger().warn(f"Invalid joint '{self.joint_to_control}'. Defaulting to '1-1'")
            self.joint_to_control = '1-1'
        
        # Create timer with initial rate
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("Sinusoidal Joint Command Node Started")
        self.get_logger().info(f"Controlling joint: {self.joint_to_control}")
        self.get_logger().info("Other joints will maintain their current positions from /joint_states")

    def joint_state_callback(self, msg):
        # Update current joint positions from joint_state message
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.available_joints and i < len(msg.position):
                self.current_joint_positions[joint_name] = msg.position[i]
        
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info("Received first joint states message")

    def parameters_callback(self, params):
        # Handle parameter changes
        for param in params:
            # if param.name == 'amplitude':
            #     self.amplitude = param.value
            #     self.get_logger().info(f"Amplitude updated to: {self.amplitude}")
            # elif param.name == 'offset':
            #     self.offset = param.value
            #     self.get_logger().info(f"Offset updated to: {self.offset}")
            if param.name == 'frequency':
                self.frequency = param.value
                self.get_logger().info(f"Frequency updated to: {self.frequency} Hz")
            elif param.name == 'publish_rate':
                self.publish_rate = param.value
                self.timer_period = 1.0 / self.publish_rate
                self.timer.cancel()
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
                self.get_logger().info(f"Publish rate updated to: {self.publish_rate} Hz")
            elif param.name == 'joint_to_control':
                if param.value in self.available_joints:
                    old_joint = self.joint_to_control
                    self.joint_to_control = param.value
                    self.get_logger().info(f"Now controlling joint: {self.joint_to_control} (was {old_joint})")
                else:
                    self.get_logger().error(f"Invalid joint '{param.value}'. Must be one of: {self.available_joints}")
                    return SetParametersResult(successful=False, reason=f"Joint must be one of: {self.available_joints}")
            elif param.name == 'min_position':
                self.min_position = param.value
                self.amplitude = (self.max_position - self.min_position)/2
                self.offset = (self.max_position + self.min_position)/2
                self.get_logger().info(f"Min position updated to: {self.min_position}")
            elif param.name == 'max_position':
                self.max_position = param.value
                self.amplitude = (self.max_position - self.min_position)/2
                self.offset = (self.max_position + self.min_position)/2                
                self.get_logger().info(f"Max position updated to: {self.max_position}")
            elif param.name == 'wavetype':
                self.wavetype = param.value
                self.get_logger().info(f"Wavetype updated to: {self.wavetype}")
        
        # Validate that the combination of amplitude and offset stays within min/max bounds
        calculated_min = self.offset - self.amplitude
        calculated_max = self.offset + self.amplitude
        
        if calculated_min < self.min_position or calculated_max > self.max_position:
            self.get_logger().warn(
                f"Parameter combination may exceed bounds! "
                f"Calculated range: [{calculated_min:.2f}, {calculated_max:.2f}], "
                f"Allowed range: [{self.min_position:.2f}, {self.max_position:.2f}]"
            )
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        # Calculate sine wave value for the controlled joint
        if self.wavetype == 'square':
            sine_value = self.amplitude * (1.0 if math.sin(self.phase) >= 0 else -1.0) + self.offset
        else: 
            sine_value = self.amplitude * math.sin(self.phase) + self.offset
        
        # Clamp the value to the specified range
        controlled_joint_value = max(self.min_position, min(self.max_position, sine_value))
        
        # Update phase for next iteration
        self.phase += 2 * math.pi * self.frequency * self.timer_period
        
        # Wrap phase to prevent overflow
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi
        
        # Create message with positions for all three joints
        msg = Float64MultiArray()
        
        # Use current positions for all joints, then override the controlled one
        joint_positions = [
            self.current_joint_positions['1-1'],
            self.current_joint_positions['1-2'], 
            self.current_joint_positions['1-3']
        ]
        
        # Override the controlled joint with our sine wave value
        if self.joint_to_control == '1-1':
            joint_positions[0] = controlled_joint_value
        elif self.joint_to_control == '1-2':
            joint_positions[1] = controlled_joint_value
        elif self.joint_to_control == '1-3':
            joint_positions[2] = controlled_joint_value
        
        msg.data = joint_positions
        
        # Publish message
        self.publisher_.publish(msg)
        
        # Log output occasionally for debugging
        self.get_logger().info(
            f'Controlling {self.joint_to_control}: {controlled_joint_value:.3f} rad | '
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