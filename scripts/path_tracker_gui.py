import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tkinter as tk
from PIL import Image, ImageTk
import math

class OdomSubscriber(Node):
    def __init__(self, canvas, pointer_img):
        super().__init__('odom_subscriber')
        self.canvas = canvas
        self.pointer_img = pointer_img
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.last_x = None
        self.last_y = None
        self.scale = 100  # scale factor to adjust position on canvas
        self.offset_x = 250  # offset to center the drawing on canvas
        self.offset_y = 250
        self.pointer = None  # Handle for pointer image on canvas
        self.path_segments = []  # Store path segments for animation
        self.animation_speed = 50  # Time in ms for animation speed
        self.animation_index = 0  # To track current segment being drawn

    def odom_callback(self, msg):
        # Extract position from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert orientation quaternion to yaw angle (rotation around Z axis)
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

        # Swap x and y axes for the canvas display
        pixel_x = -y * self.scale + self.offset_x  # Swap x and y, mirror y for correct path
        pixel_y = -x * self.scale + self.offset_y  # Swap x and y, mirror x for correct orientation

        # Store the current position to draw the path
        if self.last_x is not None and self.last_y is not None:
            self.path_segments.append((self.last_x, self.last_y, pixel_x, pixel_y))

        # Update the pointer position and rotation
        self.update_pointer(pixel_x, pixel_y, yaw)

        # Update the last position
        self.last_x = pixel_x
        self.last_y = pixel_y

        # Start the animation if there are segments to draw
        if self.animation_index < len(self.path_segments):
            self.draw_next_segment()

    def draw_next_segment(self):
        if self.animation_index < len(self.path_segments):
            segment = self.path_segments[self.animation_index]
            self.canvas.create_line(segment[0], segment[1], segment[2], segment[3], fill="green", width=2)
            self.animation_index += 1
            self.canvas.after(self.animation_speed, self.draw_next_segment)  # Call this method again after the specified speed

    def quaternion_to_euler_yaw(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to euler yaw angle
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def update_pointer(self, x, y, yaw):
        # Remove the previous pointer if it exists
        if self.pointer is not None:
            self.canvas.delete(self.pointer)

        # Rotate the image based on the yaw angle
        rotated_img = self.pointer_img.rotate(math.degrees(yaw))  # No need to negate yaw now

        # Convert the rotated image to a format Tkinter can use
        tk_img = ImageTk.PhotoImage(rotated_img)

        # Place the rotated image at the robot's position
        self.pointer = self.canvas.create_image(x, y, image=tk_img)
        
        # Keep a reference to the image to prevent garbage collection
        self.canvas.tk_img = tk_img


class PathTrackerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Turtlebot Path Tracker")

        # Create a canvas to draw the path
        self.canvas = tk.Canvas(root, width=500, height=500, bg="white")
        self.canvas.pack()

        # Load the pointer image (e.g., an arrow PNG file)
        self.pointer_img = Image.open("pointer.png")  # Update with your pointer file path

        # Initialize ROS node in a separate thread
        rclpy.init()
        self.node = OdomSubscriber(self.canvas, self.pointer_img)

        # Run the ROS node in a non-blocking way
        self.update_ros()

    def update_ros(self):
        # Spin the ROS node to process messages
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Keep updating the GUI every 100ms
        self.root.after(100, self.update_ros)

    def close(self):
        # Shutdown ROS node gracefully when closing the GUI
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
        self.root.quit()


if __name__ == '__main__':
    # Set up the Tkinter GUI
    root = tk.Tk()
    app = PathTrackerApp(root)

    # Set the Tkinter close event to properly shutdown ROS
    root.protocol("WM_DELETE_WINDOW", app.close)

    # Start the Tkinter main loop
    root.mainloop()
