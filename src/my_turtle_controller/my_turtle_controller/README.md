
# 🐢 ROS2 TurtleSim Shape Drawer

Este proyecto permite controlar una tortuga de **Turtlesim** en ROS2 usando el teclado, con funcionalidades adicionales para que la tortuga dibuje letras en pantalla como `M`, `A`, `L`, `O`, `C`, `D` y `S`. ¡Una manera divertida de interactuar con Turtlesim y practicar control de robots en ROS2!

---

## 📦 Requisitos

- Python 3
- ROS2 Humble
- Paquete `turtlesim` instalado
- Terminal compatible con entrada de teclas sin eco (ej. terminal de Linux o macOS)

---

## 🚀 Instalación y Ejecución

1. **Configura tu espacio de trabajo ROS2**

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Clona este repositorio** en tu carpeta `src` de tu workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone <URL-del-repositorio>
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. **Lanza el simulador de la tortuga** en un terminal:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

4. **Ejecuta el controlador** en otro terminal:

   ```bash
   ros2 run my_turtle_controller move_turtle
   ```

---

## 🎮 Controles de Teclado

### Movimiento Manual

| Tecla         | Acción                      |
|---------------|-----------------------------|
| ↑ (flecha arriba)    | Avanzar               |
| ↓ (flecha abajo)     | Retroceder            |
| ← (flecha izquierda) | Girar a la izquierda  |
| → (flecha derecha)   | Girar a la derecha    |
| `+`           | Aumentar velocidad lineal y angular |
| `-`           | Disminuir velocidad lineal y angular |

### Dibujo de Letras

Presiona una de las siguientes teclas para iniciar el trazado de la letra correspondiente. Durante el dibujo, puedes presionar `q` para interrumpir.

| Tecla | Letra | Descripción              |
|-------|-------|--------------------------|
| `m`   | M     | Dibuja la letra 'M' mediante segmentos rectos.|  
| `a`   | A     | Dibuja la letra 'A' con forma correcta y giro inicial.|  
| `l`   | L     | Dibuja la letra 'L' con movimiento vertical y horizontal.|  
| `o`   | O     | Dibuja la letra 'O' como un círculo completo.|  
| `c`   | C     | Dibuja la letra 'C' como un arco mayoritario.|  
| `d`   | D     | Dibuja la letra 'D' combinando línea vertical y arco.|
| `s`   | S     | Dibuja la letra 'S' con dos semicírculos concatenados.|
| `q`   | Q     | Salir del programa       |

---

## 🧩 Descripción de Clases y Módulos

### `ShapeGenerator`

- **Propósito**: Proporciona métodos para generar curvas y rotar puntos y formas.
- **Métodos clave**:
  - `rotate_point(point, angle)`: Rota un punto `(x, y)` por un ángulo dado.
  - `generate_arc_segments(radius, start_angle, end_angle, num_segments)`: Devuelve segmentos relativos que describen un arco de círculo.
  - `rotate_shape(points, angle)`: Rota una lista de puntos relativos por un ángulo.

### `LetterShapes`

- **Propósito**: Define las trayectorias relativas de cada letra que la tortuga debe seguir.
- **Constructor**:
  - `base_scale`: Escala general de las letras.
  - `num_arc_full` y `num_arc_part`: Número de segmentos para arcos completos y parciales.
- **Atributos**:
  - `self.shapes`: Diccionario que asocia cada letra (`'m'`, `'a'`, etc.) con una lista de segmentos relativos `(dx, dy)`.

### `KeyboardControllerNode` (subclase de `rclpy.node.Node`)

- **Propósito**: Nodo central que maneja la entrada de teclado, publica comandos de velocidad y controla el dibujo de formas.
- **Atributos importantes**:
  - `cmd_pub`: Publicador de mensajes `Twist` en `/turtle1/cmd_vel`.
  - `pose_sub`: Suscriptor a `/turtle1/pose` para obtener la posición actual.
  - `shape_bindings`: Mapeo de teclas a las formas definidas en `LetterShapes`.
  - `speed`, `turn`: Velocidades base lineal y angular.
  - `drawing`, `interrupt`: Flags para controlar la ejecución de dibujo y posibles interrupciones.
- **Métodos clave**:
  - `keyboard_loop()`: Bucle que captura y procesa pulsaciones de teclas.
  - `execute_trajectory(rel_points)`: Convierte puntos relativos en metas absolutas y las recorre.
  - `move_to_goal(gx, gy)`: Control proporcional (P-control) para mover la tortuga hasta `(gx, gy)`.
  - `pose_cb(msg)`: Callback para actualizar la posición actual de la tortuga.
  - `publish(lin, ang)`: Publica una velocidad lineal y angular.

### Funciones Auxiliares

- `normalize_angle(a)`: Normaliza un ángulo para que quede entre `-π` y `π`.
- `print_current_speeds(spd, trn)`: Imprime en consola las velocidades actuales.

---

## ⚙️ Flujo de Ejecución

1. `main()`: Guarda la configuración de terminal, inicializa ROS2 y crea la instancia de `KeyboardControllerNode`.
2. Se lanza ROS2 y se espera la primera pose de la tortuga.
3. El hilo `keyboard_loop` captura pulsaciones, envía comandos inmediatos o lanza hilos de dibujo.
4. Para cada letra, `execute_trajectory` calcula metas y usa `move_to_goal` para un trazado suave.
5. Se puede interrumpir el dibujo con `q` o usar flechas para control manual en cualquier momento.

---

## 📌 Notas y Consejos

- **Enfoque de la terminal**: Asegúrate de que la ventana tenga foco para recibir las teclas.
- **Límites de área**: Si un punto meta está fuera de `[0+0.1, 11-0.1]` en `x` o `y`, se detiene el dibujo.
- **Velocidades**: Ajustar con `+` y `-` en tiempo real según la precisión deseada.

---

## 🐛 Contribuciones y Feedback

¡Tu colaboración es bienvenida! Si encuentras errores o tienes sugerencias, abre un _issue_ o envía un _pull request_.

---

## 📄 Licencia

Este proyecto está bajo la licencia **MIT**. ¡Siéntete libre de usarlo y adaptarlo!

