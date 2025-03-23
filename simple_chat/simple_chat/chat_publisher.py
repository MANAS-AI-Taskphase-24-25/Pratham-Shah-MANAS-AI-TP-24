import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatPublisher(Node):
    # default constructor
    def __init__(self):
        super().__init__('chat_publisher')
        
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        
        self.get_logger().info('Type messages:')

    #method to publish
    def publish_message(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    #initialise rclpy
    rclpy.init(args=args)
    node = ChatPublisher()
    
    try:
        while True:
            # infinite message receiver
            user_input = input('Enter the message: ')
            node.publish_message(user_input) #publishes user input
            rclpy.spin_once(node, timeout_sec=0.1)  # process any callback
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
