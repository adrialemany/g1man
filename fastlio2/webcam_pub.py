import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class USBWebcamNode(Node):
    def __init__(self):
        super().__init__('usb_webcam_node')

        # Publicador de imagen comprimida
        self.publisher_ = self.create_publisher(CompressedImage, '/webcam/image/compressed', 10)

        # Abrir la cámara USB (/dev/video6)
        self.cap = cv2.VideoCapture(6)

        # Configurar resolución (ajusta según tu cámara, 1280x720 es balanceado)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Timer para 20 FPS para no saturar la red
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("📷 Webcam USB 4K Iniciada en /webcam/image/compressed")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "webcam_link"
            msg.format = "jpeg"

            # Comprimimos la imagen al 70% de calidad (estilo profesor: equilibrio peso/calidad)
            success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])

            if success:
                msg.data = buffer.tobytes()
                self.publisher_.publish(msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = USBWebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
