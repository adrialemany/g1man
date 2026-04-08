import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class LidarImuRelay(Node):
    def __init__(self):
        super().__init__('lidar_imu_relay')

        # QoS optimizado para alto tráfico (Best Effort + Buffer grande)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5, # Buffer pequeño para evitar latencia acumulada
            durability=DurabilityPolicy.VOLATILE
        )

        self.pc_pub = self.create_publisher(PointCloud2, '/livox/lidar', qos)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', qos)

        self.pc_sub = self.create_subscription(PointCloud2, '/utlidar/cloud_livox_mid360', self.pc_callback, qos)
        self.imu_sub = self.create_subscription(Imu, '/utlidar/imu_livox_mid360', self.imu_callback, qos)

        self.get_logger().info("✅ Relay Optimizado")

    def pc_callback(self, msg):
        # Forzamos que el frame_id sea livox_frame para que RViz no se pierda
        msg.header.frame_id = "livox_frame"
        self.pc_pub.publish(msg)

    def imu_callback(self, msg):
        msg.header.frame_id = "livox_frame"
        self.imu_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = LidarImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

