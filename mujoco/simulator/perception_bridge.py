import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import zmq
import numpy as np
import threading

class PerceptionBridge(Node):
    def __init__(self):
        super().__init__('perception_bridge')
        
        # Publicador de Nube de Puntos para MoveIt
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
        
        # Conexión ZMQ al simulador
        context = zmq.Context()
        self.sub_depth = context.socket(zmq.SUB)
        self.sub_depth.setsockopt(zmq.CONFLATE, 1)
        self.sub_depth.connect("tcp://127.0.0.1:5556")
        self.sub_depth.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Parámetros de la cámara (Coinciden con el XML de MuJoCo)
        self.width = 640
        self.height = 480
        self.fov = 55.2 * (np.pi / 180.0) # FOV en radianes
        self.fx = (self.width / 2.0) / np.tan(self.fov / 2.0)
        self.fy = self.fx # Asumimos píxeles cuadrados
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        
        # Pre-calcular malla para vectorización ultra-rápida
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.u = u.flatten()
        self.v = v.flatten()

        self.get_logger().info("👁️ Puente de Percepción Activo. Esperando datos ZMQ...")
        
        # Iniciamos el hilo que procesará las imágenes
        threading.Thread(target=self.process_depth, daemon=True).start()

    def process_depth(self):
        while rclpy.ok():
            try:
                # Recibimos los bytes brutos
                buffer = self.sub_depth.recv(flags=zmq.NOBLOCK)
                
                # MuJoCo renderiza la profundidad como un array de floats (Float32)
                depth_map = np.frombuffer(buffer, dtype=np.float32)
                
                # Filtramos puntos: Quitamos el fondo infinito o errores (e.g., > 3 metros)
                valid_mask = (depth_map > 0.01) & (depth_map < 3.0)
                
                z = depth_map[valid_mask]
                u = self.u[valid_mask]
                v = self.v[valid_mask]
                
                # Fórmulas de proyección de cámara Pinhole (2D a 3D)
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy
                
                # Apilamos en formato [X, Y, Z]
                points_3d = np.stack((z, -x, -y), axis=-1) # Ejes ajustados para ROS (X al frente)
                
                # Crear el mensaje ROS 2
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                # MUY IMPORTANTE: Este nombre debe coincidir con el link que pusiste en el URDF
                header.frame_id = 'lidar_link' 
                
                pc2_msg = pc2.create_cloud_xyz32(header, points_3d.tolist())
                
                self.pc_pub.publish(pc2_msg)
                
            except zmq.Again:
                pass
            except Exception as e:
                self.get_logger().error(f"Error procesando nube: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
