#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import RcChannels
from rclpy.qos import QoSPresetProfiles
import tkinter as tk
import threading

class MockRcChannelsPub(Node):
    def __init__(self, axes_vars):
        super().__init__('mock_rc_channels_pub')
        self.axes_vars = axes_vars
        
        # Publish to the EXACT same topic the rc_publisher is listening to
        self.publisher_ = self.create_publisher(RcChannels, '/fmu/out/rc_channells', QoSPresetProfiles.SYSTEM_DEFAULT.value)
        
        # Publish at 20 Hz
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Mock RC GUI publisher started on /fmu/out/rc_channels')

    def timer_callback(self):
        msg = RcChannels()
        # PX4 expects timestamps in microseconds
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        channels = [0.0] * 18
        
        # Pull values directly from Tkinter variables
        for i, var in enumerate(self.axes_vars):
            channels[i] = float(var.get())
            
        msg.channels = channels
        msg.channel_count = 18
        
        # Fake a good signal
        msg.rssi = 100
        msg.signal_lost = False
        
        self.publisher_.publish(msg)

def spin_ros(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize UI
    root = tk.Tk()
    root.title("Mock RC Inputs")
    root.geometry("400x350")
    
    channel_labels = [
        "Roll (CH0)", 
        "Pitch (CH1)", 
        "Throttle (CH2)", 
        "Yaw (CH3)", 
        "Aux1 (CH4)", 
        "Aux2 (CH5)",
        "Aux3 (CH6)",
        "Aux4 (CH7)"
    ]
    
    axes_vars = []
    
    # Create sliders for the first 8 channels
    for i, label_text in enumerate(channel_labels):
        frame = tk.Frame(root)
        frame.pack(fill=tk.X, padx=10, pady=5)
        
        lbl = tk.Label(frame, text=label_text, width=12, anchor='w')
        lbl.pack(side=tk.LEFT)
        
        # Default throttle down (-1.0), else center (0.0)
        default_val = -1.0 if i == 2 else 0.0
        var = tk.DoubleVar(value=default_val)
        axes_vars.append(var)
        
        slider = tk.Scale(frame, variable=var, from_=-1.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL)
        slider.pack(side=tk.RIGHT, expand=True, fill=tk.X)
    
    # Create node and start ROS spin in background
    node = MockRcChannelsPub(axes_vars)
    spin_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Blocks here until user closes the window
        root.mainloop() 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
