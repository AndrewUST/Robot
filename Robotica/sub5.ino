#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

// Pines de control de motores
#define motor1Pin1 12
#define motor1Pin2 13
#define ENA 18
#define motor2Pin1 14
#define motor2Pin2 27
#define ENB 23

// Pines del sensor ultrasonido
#define TRIG 19
#define ECO 21
int DURACION;
int DISTANCIA;
int distance;

// Variables de ROS2
rcl_subscription_t subscription;
geometry_msgs__msg__Vector3 msg; // Ajuste en el tipo del mensaje
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Datos recibidos del marcador ArUco
int aruco_id = -1;  // ID del marcador
int flag = 1;

// Configurar pines de motores
void setup_motors() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Detener los motores inicialmente
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(ENB, 0);
}

// Configurar el sensor ultrasonido
void setup_ultrasonic() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECO, INPUT);
}

// Medir distancia con el sensor ultrasonido
float measure_distance() {
  digitalWrite(TRIG, HIGH);     // generacion del pulso a enviar
  delay(1);       // al pin conectado al trigger
  digitalWrite(TRIG, LOW);    // del sensor
  
  DURACION = pulseIn(ECO, HIGH);  // con funcion pulseIn se espera un pulso
            // alto en Echo
  DISTANCIA = DURACION / 58.2;    // distancia medida en centimetros
  Serial.println(DISTANCIA);    // envio de valor de distancia por monitor serial
  delay(10);     
  return DISTANCIA;
}

// Movimiento básico
void move_forward(int enable) {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(ENA, enable);

  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(ENB, enable);
}

void move_backward(int enable) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(ENA, enable);

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(ENB, enable);
}

void turn_left(int enable) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(ENA, enable);

  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(ENB, enable);
}

void turn_right(int enable) {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(ENA, enable);

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(ENB, enable);
}

void stop_motors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(ENB, 0);
}

// Callback para el suscriptor
void subscription_callback(const void *msg_in) {
  const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msg_in;

  // Extraer datos del mensaje
  aruco_id = (int)msg->y;  // ID del marcador
}

// Secuencia principal
void instruccion_ID() {
    if (aruco_id == 0) {
      turn_left(180);
      delay(500);  // Simular giro de 90 grados izquierda
      stop_motors();
      aruco_id = -1;
          
    } else if (aruco_id == 1) {
      turn_right(180);
      delay(500);  // Simular giro de 90 grados derecha
      stop_motors();
      aruco_id = -1;
    
    } else if (aruco_id == 2) {
      turn_right(180);
      delay(1000);  // Simular giro de 180 grados
      stop_motors();
      aruco_id = -1;

    } else if (aruco_id == 3) {
      turn_left(180);
      delay(500);  // Simular giro de 90 grados izquierda
      stop_motors();
      aruco_id = -1;
    
    } else if (aruco_id == 4) {
      turn_right(180);
      delay(1000);  // Simular giro de 80 grados final
      stop_motors();
      aruco_id = -1;
      flag = 0;
    }
}

void setup() {
  set_microros_wifi_transports("MATEO2", "12345789", "192.168.137.190", 8888);
  setup_motors();
  setup_ultrasonic();

  Serial.begin(115200);  		// inicializacion de comunicacion serial 

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "motores", "", &support);

  rclc_subscription_init_default(
      &subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "aruco_pub");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  distance = measure_distance();  // Usar medición del ultrasonido
  if(distance > 20 && flag == 1){
    move_forward(100);
  }
  else if (distance <= 19 && flag == 1) { 
    stop_motors();
    delay(1000);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    instruccion_ID();
    delay(100);
  } 
  else if(flag == 0){
    stop_motors();
  }
}
