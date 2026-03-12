#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import threading
import time

try:
    import hid
    HID_AVAILABLE = True
except ImportError:
    HID_AVAILABLE = False


class GulikitJoystickNode(Node):
    def __init__(self):
        super().__init__('gulikit_joystick_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        
        self.declare_parameter('debug_raw_bytes', False)
        
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 15
        
        if not HID_AVAILABLE:
            self.get_logger().error("python 'hid' module not found! Please install via 'pip install hid'")
            return
            
        self.running = True
        self.gamepad = None
        
        self.read_thread = threading.Thread(target=self.joystick_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def connect_device(self):
        # Look for controllers that match XInput / Xbox / GuliKit
        for device in hid.enumerate():
            # You can print all devices for debugging:
            # self.get_logger().info(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")
            
            prod_str = (device.get('product_string') or '').lower()
            if 'gulikit' in prod_str or 'xbox' in prod_str or 'controller' in prod_str or 'gamepad' in prod_str:
                try:
                    self.gamepad = hid.device()
                    self.gamepad.open(device['vendor_id'], device['product_id'])
                    self.gamepad.set_nonblocking(True)
                    self.get_logger().info(f"Connected to HID Device: {device['product_string']} (0x{device['vendor_id']:04x}:0x{device['product_id']:04x})")
                    return True
                except Exception as e:
                    self.get_logger().warn(f"Failed to open device: {e}")
                    
        return False

    def joystick_loop(self):
        while self.running and rclpy.ok():
            if self.gamepad is None:
                if self.connect_device():
                    time.sleep(0.1)
                else:
                    self.get_logger().info("Searching for GuliKit/Xbox HID controller...", throttle_duration_sec=5.0)
                    time.sleep(2.0)
                continue

            try:
                debug = self.get_parameter('debug_raw_bytes').value
                report = self.gamepad.read(64)
                
                if report:
                    if debug:
                        self.get_logger().info(f"Raw report: {report}")
                        
                    self.parse_xinput_report(report)
                    self.joy_msg.header.stamp = self.get_clock().now().to_msg()
                    self.publisher_.publish(self.joy_msg)
                else:
                    # Non-blocking read returned None, meaning no new data
                    time.sleep(0.005)
                    
            except OSError as e:
                self.get_logger().warn(f"Gamepad disconnected: {e}")
                if self.gamepad:
                    self.gamepad.close()
                self.gamepad = None
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"Error reading joystick: {e}")
                time.sleep(1)

    def parse_xinput_report(self, data):
        """
        Parses a typical Xbox Wireless over Bluetooth HID Report.
        Note: The actual byte indices might slightly shift depending on OS (macOS/Linux)
        and exact firmware. If buttons are wrong, enable 'debug_raw_bytes' and see which
        bytes change when you press buttons!
        
        A typical macOS XInput Bluetooth layout is often 15-17 bytes:
        Byte 1,2: Left Stick X (16-bit LE)
        Byte 3,4: Left Stick Y (16-bit LE)
        Byte 5,6: Right Stick X (16-bit LE)
        Byte 7,8: Right Stick Y (16-bit LE)
        Byte 9: Left Trigger (0-255)
        Byte 10: Right Trigger (0-255)
        Byte 11: Switch Hat (D-Pad) 1-8
        Byte 12-14: Buttons bitmasks
        """
        if len(data) < 14:
            return  # packet too small
            
        # We will dynamically find standard patterns or just use typical offsets.
        # Assuming typical Xbox BT layout for Mac:
        # data[0] could be the Report ID.
        # If Report ID is present, we offset by 1. Wait, let's look at the length.
        offset = 0
        if len(data) == 16 or len(data) == 17:
             offset = 1 # Skip report ID
            
        def u16le(b1, b2):
            return b1 | (b2 << 8)
            
        def normalize_axis_16(val):
            # 0 to 65535 -> -1.0 to 1.0
            return (val - 32768) / 32768.0

        def normalize_axis_8(val):
            # 0 to 255 -> 0.0 to 1.0
            return val / 255.0

        try:
            # Axes
            lx = u16le(data[offset+0], data[offset+1])
            ly = u16le(data[offset+2], data[offset+3])
            rx = u16le(data[offset+4], data[offset+5])
            ry = u16le(data[offset+6], data[offset+7])
            
            # Left Trigger, Right Trigger
            # Often on byte 9 & 10, or 16-bit on 8,9 and 10,11
            # Let's try 10-bit or 8-bit. We'll assume 8-bit or 10-bit and map as 8-bit for simplicity
            lt = u16le(data[offset+8], data[offset+9])
            rt = u16le(data[offset+10], data[offset+11])
            
            self.joy_msg.axes[0] = normalize_axis_16(lx)
            self.joy_msg.axes[1] = -normalize_axis_16(ly) # Invert Y for standard ROS Joy
            self.joy_msg.axes[2] = normalize_axis_16(lt) # Trigger
            self.joy_msg.axes[3] = normalize_axis_16(rx)
            self.joy_msg.axes[4] = -normalize_axis_16(ry)
            self.joy_msg.axes[5] = normalize_axis_16(rt) # Trigger
            
            # D-Pad (Hat switch) usually 1 nibble, often around data[13]
            # Buttons bitmask usually around data[14], data[15]
            # Since layouts vary wildly, we'll try standard 8-byte buttons
            
            buttons_1 = data[offset+12] if len(data) > offset+12 else 0
            buttons_2 = data[offset+13] if len(data) > offset+13 else 0
            
            # Example mapping:
            self.joy_msg.buttons[0] = (buttons_1 & 0x01) > 0 # A
            self.joy_msg.buttons[1] = (buttons_1 & 0x02) > 0 # B
            self.joy_msg.buttons[2] = (buttons_1 & 0x08) > 0 # X
            self.joy_msg.buttons[3] = (buttons_1 & 0x10) > 0 # Y
            self.joy_msg.buttons[4] = (buttons_1 & 0x40) > 0 # LB
            self.joy_msg.buttons[5] = (buttons_1 & 0x80) > 0 # RB
            
            self.joy_msg.buttons[6] = (buttons_2 & 0x04) > 0 # View
            self.joy_msg.buttons[7] = (buttons_2 & 0x08) > 0 # Menu
            self.joy_msg.buttons[8] = (buttons_2 & 0x10) > 0 # Xbox
            self.joy_msg.buttons[9] = (buttons_2 & 0x20) > 0 # LS
            self.joy_msg.buttons[10] = (buttons_2 & 0x40) > 0 # RS
            
            # D-pad (hat) often encoded as 1..8 value in another byte
            hat = data[offset+11] if len(data) > offset+11 else 0
            # 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW
            self.joy_msg.axes[6] = 0.0
            self.joy_msg.axes[7] = 0.0
            
            if hat == 1 or hat == 2 or hat == 8:
                self.joy_msg.axes[7] = 1.0 # Up
            elif hat == 4 or hat == 5 or hat == 6:
                self.joy_msg.axes[7] = -1.0 # Down
                
            if hat == 2 or hat == 3 or hat == 4:
                self.joy_msg.axes[6] = -1.0 # Right
            elif hat == 6 or hat == 7 or hat == 8:
                self.joy_msg.axes[6] = 1.0 # Left
                
        except IndexError:
            pass

    def destroy_node(self):
        self.running = False
        if self.gamepad:
            self.gamepad.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GulikitJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
