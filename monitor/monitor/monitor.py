import rclpy
from rclpy.node import Node
import tkinter as tk
import threading
import numpy as np
from moveit_msgs.msg import PlanningScene
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from example_interfaces.srv import Trigger

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
    
class VectorDisplay(tk.Canvas):
    def __init__(self, master, *args, **kwargs):
        super().__init__(master, *args, **kwargs)

        # Set the canvas size
        self.config(width=200, height=200)

        # Create a line for the vector
        self.line = self.create_line(0, 0, 0, 0, 0, 0, arrow=tk.LAST, fill='black', width=2)

        # Initialize the vector direction to [0, 1, 0]
        self.direction = np.array([0, 0, 1])

        # Update the vector every 500ms
        self.after(500, self.update_vector)

    def set_direction(self, direction):
        # Normalize the direction vector
        self.direction = direction / np.linalg.norm(direction)

    def update_vector(self):
        # Get the center of the canvas
        center_x = self.winfo_width() / 2
        center_y = self.winfo_height() / 2

        # Compute the end point of the line
        end_x = center_x + self.direction[0] * center_x
        end_y = center_y - self.direction[2] * center_y  # Reverse the y-axis to match ROS convention

        # Update the line coordinates
        self.coords(self.line, center_x, center_y, end_x, end_y)

        # Schedule the next update
        self.after(500, self.update_vector)
        
class Monitor(Node):

    def __init__(self):
        super().__init__('Monitor')
        self.subscription = self.create_subscription(
            PlanningScene,
            '/monitored_planning_scene',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.x_raw = 0
        self.y_raw = 0
        self.z_raw = 0
        
        self.cli = self.create_client(Trigger, '/mtc_node/start_pick_and_place')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()
        
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def button_clicked(self):
        self.start_pick_and_place()
        
    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Vention Assignement")
        
        # Create a button
        self.button = tk.Button(self.root, text="Start", command=self.button_clicked, width=20, height=4)
        self.button.grid(row=0, column=0, padx=20, pady=20)
        
        # Create a frame to hold the labels and widgets
        self.frame = tk.Frame(self.root)
        self.frame.grid(row=1, column=0)

        self.x = tk.DoubleVar()
        self.y = tk.DoubleVar()
        self.z = tk.DoubleVar()
        
        # Create labels for each value
        self.x_label = tk.Label(self.frame, text="X:")
        self.y_label = tk.Label(self.frame, text="Y:")
        self.z_label = tk.Label(self.frame, text="Z:")

        # Create label widgets for each value
        self.x_widget = tk.Label(self.frame, textvariable=self.x)
        self.y_widget = tk.Label(self.frame, textvariable=self.y)
        self.z_widget = tk.Label(self.frame, textvariable=self.z)

        # Pack the labels and label widgets in a grid layout
        self.x_label.grid(row=0, column=0, padx=20, pady=20)
        self.x_widget.grid(row=0, column=1, padx=20, pady=20)
        self.y_label.grid(row=1, column=0, padx=20, pady=20)
        self.y_widget.grid(row=1, column=1, padx=20, pady=20)
        self.z_label.grid(row=2, column=0, padx=20, pady=20)
        self.z_widget.grid(row=2, column=1, padx=20, pady=20)
        
        # Create a label for the VectorDisplay
        vector_label = tk.Label(self.root, text='Vector Display:', font=('Arial', 14))
        vector_label.grid(row=3, column=0, padx=20, pady=20, sticky='w')

        # Create the VectorDisplay
        self.vector_display = VectorDisplay(self.root, bg='white')
        self.vector_display.grid(row=3, column=1, padx=20, pady=20)
        
        # Update live data every 500ms
        self.root.after(500, self.update_live_data)
    
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def euler_to_direction(self, euler_angles):
        # Convert the Euler angles to a rotation matrix
        roll, pitch, yaw = euler_angles
        c_r, s_r = np.cos(roll), np.sin(roll)
        c_p, s_p = np.cos(pitch), np.sin(pitch)
        c_y, s_y = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [c_p*c_y, c_y*s_p*s_r - c_r*s_y, s_r*s_y + c_r*c_y*s_p],
            [c_p*s_y, c_r*c_y + s_p*s_r*s_y, c_r*s_p*s_y - c_y*s_r],
            [-s_p, c_p*s_r, c_p*c_r]
        ])
    
        # Compute the direction vector for an euler of 0, 0, 0
        base_direction = np.array([0, 0, 1])

        # Compute the direction vector
        direction = R @ base_direction
    
        return direction

        
    def update_vector_display(self, orientation):
        # Convert the orientation quaternion to a direction vector
        direction = self.euler_to_direction(orientation)
        self.vector_display.set_direction(direction)
        
    def on_closing(self):
        # Stop the thread when the window is closed
        self.gui_thread.join()
        self.root.destroy()
        
    def update_live_data(self):
        # Update the live data label with some new data
        self.x.set(round(self.x_raw, 2))
        self.y.set(round(self.y_raw, 2))
        self.z.set(round(self.z_raw, 2))
        # Schedule the next update in 500ms
        self.root.after(500, self.update_live_data)
        
    def listener_callback(self, msg):
        try:
            t = self.tf_buffer.lookup_transform("world", "panda_hand", rclpy.time.Time())
            #self.get_logger().info('x transform: "%s"' % t.transform.translation.x)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return
                    
        if (len(msg.world.collision_objects)>0):
            self.x_raw = msg.world.collision_objects[0].pose.position.x
            self.y_raw = msg.world.collision_objects[0].pose.position.y
            self.z_raw = msg.world.collision_objects[0].pose.position.z
            orientation = msg.world.collision_objects[0].pose.orientation
            q = [orientation.w, orientation.x, orientation.y, orientation.z]
            
            roll, pitch, yaw = euler_from_quaternion(orientation)
            self.update_vector_display([roll, pitch, yaw])
            #print("Roll: ", roll)
            #print("Pitch: ", pitch)
            #print("Yaw: ", yaw)
        if (len(msg.robot_state.attached_collision_objects)>0):
            object_in_world = tf2_geometry_msgs.do_transform_pose(msg.robot_state.attached_collision_objects[0].object.pose, t)
            self.x_raw = object_in_world.position.x
            self.y_raw = object_in_world.position.y
            self.z_raw = object_in_world.position.z
            orientation = object_in_world.orientation
            q = [orientation.w, orientation.x, orientation.y, orientation.z]
            roll, pitch, yaw = euler_from_quaternion(orientation)
            self.update_vector_display([roll, pitch, yaw])
            #print("Roll: ", roll)
            #print("Pitch: ", pitch)
            #print("Yaw: ", yaw)

    def start_pick_and_place(self):
        self.get_logger().info('Starting Pick and Place')
        self.future = self.cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        #self.get_logger().info('response: "%s"' % self.future.result())
        #self.get_logger().info('End Pick and Place')

def main(args=None):
    rclpy.init(args=args)

    monitor_subscriber = Monitor()

    rclpy.spin(monitor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
