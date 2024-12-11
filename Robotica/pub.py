import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(Vector3, 'aruco_pub', 10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

        # Crear el detector ArUco
        dictionary = cv2.aruco.DICT_4X4_250
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)

        # Parámetros de la cámara (aproximados para una webcam 1080p)
        self.camera_matrix = np.array([
            [1400, 0, 960],  # f_x, 0, c_x (suposición para 1080p)
            [0, 1400, 540],  # 0, f_y, c_y (suposición para 1080p)
            [0, 0, 1]        # 0, 0, 1
        ])
        self.dist_coeffs = np.zeros((4, 1))  # Suponiendo sin distorsión
        self.marker_size = 0.038  # Tamaño del marcador en metros (ajusta según tu marcador)

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image frame')
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convertir a escala de grises
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Detectar marcadores ArUco
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        # Procesar los marcadores detectados
        if ids is not None:
            for i in range(len(ids)):
                c = corners[i][0]

                # Calcular el centro del marcador
                x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
                y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4
                cv2.circle(current_frame, (int(x), int(y)), 4, (0, 0, 255), -1)

                # Calcular la distancia
                width = np.linalg.norm(c[0] - c[1])  # Ancho en píxeles
                focal_length = self.camera_matrix[0, 0]  # f_x de la matriz de cámara
                distance = (self.marker_size * focal_length) / width

                self.get_logger().info(f'ArUco ID={ids[i][0]}, Center=({x}, {y}), Distance={distance:.2f}m')

                # Publicar datos
                msg = Vector3()
                msg.x = x
                msg.y = float(ids[i][0])
                msg.z = distance
                self.publisher_.publish(msg)
        else:
            self.get_logger().info('ArUco marker not detected')

        # Mostrar la imagen
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
