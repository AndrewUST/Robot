# Robot
implementación de un sistema en ROS que permite al robot móvil realizar movimientos autónomos basados en deteccion de marcas visuales (aruco).

# Componentes

- Motor Encoder x2
- Esp32
- Raspberry pi 4
- Sensor ultrasonico
- Bateria LIPO
- Power Bank
- Camara
- Ruedas de Goma
- Rueda loca circular

# Modelo del motor 

![image](https://github.com/user-attachments/assets/fb8d675f-5bcd-4f41-ace7-9ae1b6d04012)

# Esquema Electrico de conexión

![image](https://github.com/user-attachments/assets/f2daaf5c-47eb-4df1-99ad-a54d7999ee25)

# Pasos para correr el programa

Primero en una terminal de Ubuntu hay que correr los siguientes codigos:

  python3 pub.py

Y en otra terminal:

  ros2 run usb_cam usb_cam_node_exe

Luego en una terminal de arduino se carga en la esp32 y se procede a correr el codigo sub5.ino.

# Packetes y/o Dependencias

Las librerias usadas en el codigo de arduino son:

- #include <micro_ros_arduino.h>
- #include <rcl/rcl.h>
- #include <rcl/error_handling.h>
- #include <rclc/rclc.h>
- #include <rclc/executor.h>
- #include <geometry_msgs/msg/vector3.h>
- Arduino.h: Incluido de forma implícita en el entorno de Arduino, proporciona funciones básicas como pinMode(), digitalWrite(), analogWrite(), attachInterrupt(), etc.

Y en el entorno de la raspberry, se estan usando packages y dependencias instaladas previamente en ros.

- rclpy: Para crear nodos ROS2 y suscribir/publicar mensajes.
- sensor_msgs: Para recibir mensajes de imagen.
- geometry_msgs: Para publicar mensajes de coordenadas de los marcadores ArUco.
- cv_bridge: Para convertir mensajes de imagen ROS a imágenes OpenCV.
- opencv-python (cv2): Para el procesamiento de imágenes y detección de marcadores ArUco.
- numpy: Para operaciones matemáticas y manipulación de matrices.
