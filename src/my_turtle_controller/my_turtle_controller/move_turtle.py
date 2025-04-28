#!/usr/bin/env python3

"""
keyboard_controller_shapes.py

Nodo ROS2 para controlar TurtleSim vía teclado:
 - Flechas: movimiento manual (avanzar/retroceder y girar)
 - '+' / '-' : aumentar/reducir velocidades lineal y angular
 - Teclas 'm', 'a', 'l', 'o', 'c', 'd', 's': dibujar letras
 - 'q' : salir

Capta las pulsaciones en modo raw de terminal y publica comandos Twist.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys
import select
import termios
import tty
import threading
import time

# --- Mensajes de ayuda y configuración de teclas ---
msg = """
Control Your Turtle!
---------------------------
Moving around:
       ^
       |
 <--   |   -->
       |
       v

+/- : Increase/decrease linear (10%) and angular (5%) speeds
m   : Draw 'M' (starts aligned)
a   : Draw 'A' (correct shape, likely initial turn)
l   : Draw 'L' (specific shape, likely initial turn/move back)
o   : Draw 'O' (starts aligned - appears rotated)
c   : Draw 'C' (starts aligned - appears rotated)
d   : Draw 'D' (starts aligned - appears rotated)
s   : Draw 'S' (approximate S composed of two arcs)
q   : quit

Arrow Up    : Move forward
Arrow Down  : Move backward
Arrow Left  : Turn left
Arrow Right : Turn right

IMPORTANT: This terminal must have focus.
Keystrokes will not be echoed.
"""

# Mapeo de secuencias ANSI para flechas a vectores (avance, giro)
# ESC [ A  → Flecha arriba
# ESC [ B  → Flecha abajo
# ESC [ D  → Flecha izquierda
# ESC [ C  → Flecha derecha
moveBindings = {
    '\x1b[A': (1.0, 0.0), '\x1b[B': (-1.0, 0.0),
    '\x1b[D': (0.0, 1.0), '\x1b[C': (0.0, -1.0),
}

# Factores de ajuste de velocidad y límites
SPEED_LINEAR_FACTOR = 1.10
SPEED_ANGULAR_FACTOR = 1.05
MIN_BASE_SPEED = 0.05

# Parámetros generales de control y simulador
DEFAULT_SPEED = 0.5
DEFAULT_TURN = 1.0
POSE_TIMEOUT_SEC = 1.0
CONTROL_LOOP_RATE = 10
GOAL_TOLERANCE_DIST = 0.1
GOAL_TOLERANCE_ANGLE = 0.05
KP_LINEAR = 1.5
KP_ANGULAR = 6.0
MAX_LINEAR_SPEED = 2.0
MAX_ANGULAR_SPEED = 2.0
TURTLE_MIN_X = 0.0
TURTLE_MAX_X = 11.0
TURTLE_MIN_Y = 0.0
TURTLE_MAX_Y = 11.0
BOUNDARY_PADDING = 0.1

# --- Generador de Formas y Definiciones de Letras ---
class ShapeGenerator:
"""Utilidades geométricas para generar y rotar trayectorias en 2D."""
    @staticmethod
    def rotate_point(point, angle):
       """
        Rota un punto (x,y) alrededor del origen.
        :param point: tupla (x, y)
        :param angle: ángulo en radianes
        :return: nuevo punto rotado (x', y')
        """
        x, y = point
        return (x * math.cos(angle) - y * math.sin(angle),
                x * math.sin(angle) + y * math.cos(angle))

    def generate_arc_segments(self, radius, start_angle, end_angle, num_segments):
       """
        Genera vectores diferenciales que aproximan un arco circular.
        :param radius: radio del arco
        :param start_angle: ángulo inicial (rad)
        :param end_angle: ángulo final (rad)
        :param num_segments: número de segmentos en los que dividir el arco
        :return: lista de tuplas (dx, dy)
        """
        segments = []
        if num_segments <= 0:
            return segments
        delta = (end_angle - start_angle) / num_segments
        last_x = radius * math.cos(start_angle)
        last_y = radius * math.sin(start_angle)
        for i in range(1, num_segments + 1):
            ang = start_angle + i * delta
            x = radius * math.cos(ang)
            y = radius * math.sin(ang)
            segments.append((x - last_x, y - last_y))
            last_x, last_y = x, y
        return segments

    def rotate_shape(self, points, angle):
        """
        Rota una lista de vectores relativos.
        :param points: lista de tuplas [(dx, dy), ...]
        :param angle: ángulo de rotación (rad)
        :return: lista de tuplas rotadas
        """
        return [self.rotate_point(p, angle) for p in points]

class LetterShapes:
    """
    Define las trayectorias relativas para dibujar las letras:
    M, A, L, O, C, D y S.
    """
    def __init__(self, base_scale=1.5, num_arc_full=32, num_arc_part=24):
        gen = ShapeGenerator()
        # M
        m_scale = base_scale
        self.m = [(1.0*m_scale, 0.0), (-0.5*m_scale, -0.5*m_scale), (0.5*m_scale, -0.5*m_scale), (-1.0*m_scale, 0.0)]
        # A
        a_h = 1.0 * base_scale; a_w = 1.0 * base_scale
        self.a = [(a_w/2, a_h), (a_w/2, -a_h), (-a_w/4, a_h/2), (-a_w/2, 0.0)]
        # L
        l_h = 1.0 * base_scale; l_w = 0.6 * base_scale
        self.l = [(-l_w, 0.0), (0.0, l_h)]
        # O
        o_r = 0.7 * base_scale
        o_orig = gen.generate_arc_segments(o_r, math.pi, 3*math.pi, num_arc_full)
        self.o = gen.rotate_shape(o_orig, -math.pi/2)
        # C
        c_r = 0.7 * base_scale
        c_orig = gen.generate_arc_segments(c_r, math.pi/4, 7*math.pi/4, num_arc_part)
        self.c = gen.rotate_shape(c_orig, -math.pi/2)
        # D
        d_h = 1.2 * base_scale; d_r = d_h / 2.0
        d_line = (0.0, d_h)
        d_arc = gen.generate_arc_segments(d_r, math.pi/2, -math.pi/2, num_arc_part)
        self.d = gen.rotate_shape([d_line] + d_arc, -math.pi/2)
        # S 
        s_r = 0.35 * base_scale
        num = num_arc_part
        # 1) Semicírculo superior (joroba derecha): de π a 0
        s_arc1 = gen.generate_arc_segments(s_r, math.pi, 0.0, num)
        # 2) Semicírculo inferior (joroba izquierda): de π a 2π
        s_arc2 = gen.generate_arc_segments(s_r, math.pi, 2 * math.pi, num)

        # 3) Combinar segmentos para continuidad sin saltos
        s_rel = s_arc1 + s_arc2

        # 4) Rotar cada segmento relativo para la orientación final
        s_reflected = [(-dx, dy) for dx, dy in s_rel]
        self.s = gen.rotate_shape(s_reflected, -math.pi/2)
        
        # Mapeo de tecla → forma
        self.shapes = {'m': self.m, 'a': self.a, 'l': self.l,
                       'o': self.o, 'c': self.c, 'd': self.d, 's': self.s}

# --- Nodo Controlador de Teclado ---
class KeyboardControllerNode(Node):
       """Nodo ROS que captura teclado y controla TurtleSim."""
    def __init__(self, settings):
           """
        Constructor:
        - Inicializa publicador y suscriptor
        - Arranca hilo para keyboard_loop()
        - Espera primera pose antes de mostrar ayuda
        """
        super().__init__('keyboard_controller_shapes')
        self.settings = settings
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.current_pose = None; self.pose_lock = threading.Lock()
        self.speed = DEFAULT_SPEED; self.turn = DEFAULT_TURN
        self.manual_x = 0.0; self.manual_th = 0.0
        self.drawing = False; self.interrupt = False
        self.thread = None
        shapes = LetterShapes().shapes
        self.shape_bindings = shapes
        self.key_thread = threading.Thread(target=self.keyboard_loop)
        self.key_thread.daemon = True; self.key_thread.start()
        self.get_logger().info('Initialized, waiting for pose...')
        self.wait_for_pose()

    def pose_cb(self, msg):
           """Callback de suscripción: guarda la última pose recibida."""
        with self.pose_lock:
            self.current_pose = msg

    def wait_for_pose(self):
           """Espera hasta recibir la pose inicial o hacer timeout."""
        start = self.get_clock().now()
        while self.current_pose is None and (self.get_clock().now() - start) < Duration(seconds=POSE_TIMEOUT_SEC):
            rclpy.spin_once(self, timeout_sec=0.1); time.sleep(0.1)
        if self.current_pose:
            self.get_logger().info(f"Pose: {self.current_pose.x:.2f},{self.current_pose.y:.2f},{self.current_pose.theta:.2f}")
            print_current_speeds(self.speed, self.turn); print(msg)
        else:
            self.get_logger().error('Pose timeout'); print_current_speeds(self.speed, self.turn); print(msg)

    def publish(self, lin, ang):
           """
        Publica un mensaje Twist con velocidades lineal y angular.
        :param lin: velocidad lineal
        :param ang: velocidad angular
        """
        t = Twist(); t.linear.x = float(lin); t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def keyboard_loop(self):
           """Hilo que lee el teclado en modo raw y gestiona comandos."""
        settings = self.settings
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
               # Leer sin bloqueo
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    k = sys.stdin.read(1)
                     # Detectar flechas (secuencia ESC [ X)
                    if k == '\x1b':
                        next1 = sys.stdin.read(1)
                        next2 = sys.stdin.read(1)
                        k = k + next1 + next2
                    with threading.Lock():
                     # Si estamos dibujando, solo escuchamos q o flechas para interrumpir
                        if self.drawing:
                            if k == 'q' or k in moveBindings:
                                self.interrupt = True; self.publish(0.0, 0.0)
                                if k in moveBindings:
                                    self.manual_x, self.manual_th = moveBindings[k]
                        else:
                            # Movimientos manuales
                            if k in moveBindings:
                                self.manual_x, self.manual_th = moveBindings[k]
                                self.publish(self.manual_x * self.speed, self.manual_th * self.turn)
                            # Ajuste de velocidad
                            elif k == '+':
                                self.speed = max(MIN_BASE_SPEED, self.speed * SPEED_LINEAR_FACTOR)
                                self.turn = max(MIN_BASE_SPEED, self.turn * SPEED_ANGULAR_FACTOR)
                                print_current_speeds(self.speed, self.turn)
                                if self.manual_x or self.manual_th:
                                    self.publish(self.manual_x * self.speed, self.manual_th * self.turn)
                            elif k == '-':
                                self.speed = max(MIN_BASE_SPEED, self.speed / SPEED_LINEAR_FACTOR)
                                self.turn = max(MIN_BASE_SPEED, self.turn / SPEED_ANGULAR_FACTOR)
                                print_current_speeds(self.speed, self.turn)
                                if self.manual_x or self.manual_th:
                                    self.publish(self.manual_x * self.speed, self.manual_th * self.turn)
                            # Dibujo de formas
                            elif k in self.shape_bindings:
                                shape = self.shape_bindings[k]
                                if self.current_pose:
                                    self.get_logger().info(f"Drawing {k.upper()}")
                                    self.drawing = True; self.interrupt = False
                                    self.publish(0.0, 0.0)
                                    self.thread = threading.Thread(target=self.execute_trajectory, args=(shape,))
                                    self.thread.daemon = True; self.thread.start()
                            # Salir
                            elif k == 'q':
                                break
                else:
                     # Si no hay input y había movimiento, detenerse
                    if not self.drawing and (self.manual_x or self.manual_th):
                        self.manual_x = 0.0; self.manual_th = 0.0; self.publish(0.0, 0.0)
        finally:
       # Restaurar terminal al cerrar
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.publish(0.0, 0.0)

    def execute_trajectory(self, rel_points):
           """
        Sigue una serie de puntos relativos para dibujar una forma.
        Rota y traslada cada vector al sistema global, luego llama a move_to_goal().
        """
        start = self.current_pose
        goals = []
        x, y, th = start.x, start.y, start.theta
       # Construir lista de metas absolutas
        for p in rel_points:
            dx, dy = ShapeGenerator.rotate_point(p, th)
            x += dx; y += dy
            if not (TURTLE_MIN_X + BOUNDARY_PADDING <= x <= TURTLE_MAX_X - BOUNDARY_PADDING and
                    TURTLE_MIN_Y + BOUNDARY_PADDING <= y <= TURTLE_MAX_Y - BOUNDARY_PADDING):
                self.get_logger().error('Point outside'); self.drawing = False; return
            goals.append((x, y))
       # Seguir cada meta
        for gx, gy in goals:
            if self.interrupt: break
            if not self.move_to_goal(gx, gy): break
        self.publish(0.0, 0.0)
        self.drawing = False; self.interrupt = False

    def move_to_goal(self, gx, gy):
       """
        Control proporcional para llegar a (gx, gy).
        :return: True si alcanzó la meta, False si se interrumpió o falló.
        """
        rate = self.create_rate(CONTROL_LOOP_RATE)
        while rclpy.ok() and not self.interrupt:
            cp = self.current_pose
            if cp is None:
                self.publish(0.0, 0.0); time.sleep(0.5); continue
            dx = gx - cp.x; dy = gy - cp.y
            dist = math.hypot(dx, dy)
           # Si está dentro de la tolerancia, parar
            if dist < GOAL_TOLERANCE_DIST:
                self.publish(0.0, 0.0); return True
            angle_to = math.atan2(dy, dx)
            err = normalize_angle(angle_to - cp.theta)
            # Control proporcional
            ang_vel = max(min(KP_ANGULAR * err, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
            lin_vel = KP_LINEAR * dist if abs(err) < GOAL_TOLERANCE_ANGLE else 0.0
              # Reducir velocidad angular al acercarse
            if dist < 2 * GOAL_TOLERANCE_DIST or abs(lin_vel) < 0.1:
                ang_vel = max(min(ang_vel, MAX_ANGULAR_SPEED * 0.6), -MAX_ANGULAR_SPEED * 0.6)
            self.publish(lin_vel, ang_vel)
            rate.sleep()
        self.publish(0.0, 0.0)
        return False

    def destroy(self):
           """Detiene la tortuga al destruir el nodo."""
        self.publish(0.0, 0.0)

# --- Funciones Auxiliares Globales ---
def normalize_angle(a):
       """
    Normaliza el ángulo a [-π, π].
    :param a: ángulo en radianes
    :return: ángulo normalizado
    """
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def print_current_speeds(spd, trn):
       """
    Muestra en consola las velocidades base actuales.
    :param spd: velocidad lineal
    :param trn: velocidad angular
    """
    print(f"Current base speeds: Linear={spd:.2f} Angular={trn:.2f}")

# --- Función Principal ---
def main(args=None):
       """
    Inicializa ROS, crea el nodo de teclado y mantiene vivo el spin
    hasta recibir 'q' o Ctrl+C. Al cerrar, restaura el terminal.
    """
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardControllerNode(settings)
    try:
        while rclpy.ok() and node.key_thread.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting.")
    finally:
        print("Shutting down...")
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Terminal restored.")

if __name__ == '__main__':
    main()
