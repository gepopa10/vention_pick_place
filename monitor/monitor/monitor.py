import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from example_interfaces.srv import Trigger

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
        
        self.cli = self.create_client(Trigger, '/mtc_node/start_pick_and_place')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def listener_callback(self, msg):
        try:
            t = self.tf_buffer.lookup_transform("world", "panda_hand", rclpy.time.Time())
            #self.get_logger().info('x transform: "%s"' % t.transform.translation.x)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return
                    
        if (len(msg.world.collision_objects)>0):
            self.get_logger().info('x position: "%s"' % msg.world.collision_objects[0].pose.position.x)
            self.get_logger().info('y position: "%s"' % msg.world.collision_objects[0].pose.position.y)
        if (len(msg.robot_state.attached_collision_objects)>0):
            object_in_world = tf2_geometry_msgs.do_transform_pose(msg.robot_state.attached_collision_objects[0].object.pose, t)
            self.get_logger().info('x position: "%s"' % object_in_world.position.x)
            self.get_logger().info('y position: "%s"' % object_in_world.position.y)
            #self.get_logger().info('x position: "%s"' % msg.robot_state.attached_collision_objects[0].object.pose.position.x)
            #self.get_logger().info('frame_id: "%s"' % msg.robot_state.attached_collision_objects[0].object.header.frame_id)

    def start_pick_and_place(self):
        self.get_logger().info('Starting Pick and Place')
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('response: "%s"' % self.future.result())
        self.get_logger().info('End Pick and Place')

def main(args=None):
    rclpy.init(args=args)

    monitor_subscriber = Monitor()

    monitor_subscriber.start_pick_and_place()
    rclpy.spin(monitor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
