import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # 获取 map 到 base_link 的变换
            trans = self.tf_buffer.lookup_transform(
                'map',  # 目标坐标系
                'base_link',  # 源坐标系
                rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.get_logger().info(f'机器人在map坐标系下的位置: x={x:.2f}, y={y:.2f}')
        except Exception as e:
            self.get_logger().warn(f'获取变换失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()