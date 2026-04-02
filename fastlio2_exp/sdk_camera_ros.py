import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

class CameraSDKNode(Node):
    def __init__(self):
        super().__init__('camera_sdk_node')

        self.cam_pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)

        # Inicialización del SDK
        # Asegúrate de que eth0 sea la interfaz correcta
        ChannelFactoryInitialize(0, "eth0") 
        self.video_client = VideoClient()
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()

        self.create_timer(0.055, self.timer_callback)
        self.get_logger().info("📸 Cámara a 18 FPS para estabilidad")

    def timer_callback(self):
        # El SDK de Unitree a veces bloquea el hilo. 
        # Usamos un timeout corto para no congelar el nodo.
        code, data = self.video_client.GetImageSample()
        if code == 0 and data is not None:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_face"
            msg.format = "jpeg"
            msg.data = bytes(data) 
            self.cam_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = CameraSDKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
