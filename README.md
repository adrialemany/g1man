# G1 Humanoid — Simulación MuJoCo con percepción LiDAR, mapeo y navegación autónoma

## Descripción general

Este proyecto permite simular el robot humanoide **Unitree G1** (29 grados de libertad) dentro del motor de física **MuJoCo**, dotarlo de un **LiDAR 360°** simulado por ray-casting, y conectar toda la percepción al ecosistema **ROS 2 Humble** para visualización en **RViz2**, construcción de mapas de ocupación y **navegación autónoma con Nav2**.

El robot se mueve por un laberinto de 10 m × 10 m con pasillos de 2 m de ancho. Su locomoción está controlada por una red neuronal (modelo ONNX entrenado con SAC) que produce las posiciones articulares objetivo a 50 Hz. La planificación de rutas y evitación de obstáculos se delegan a la pila completa de Navigation2.

---

## Estructura del proyecto

```
g1man/
├── mujoco/
│   ├── simulator/
│   │   ├── config.py                  ← Configuración general de la simulación
│   │   ├── scene.xml                  ← Escena MuJoCo (laberinto + includes)
│   │   ├── g1_29dof.xml               ← Modelo MJCF del robot G1 (joints, meshes, sensores)
│   │   ├── unitree_mujoco.py          ← Simulador principal (física + LiDAR 360° + ZMQ)
│   │   ├── unitree_sdk2py_bridge.py   ← Bridge Unitree SDK2 ↔ MuJoCo
│   │   └── meshes/                    ← Mallas STL del robot
│   ├── run_sim_ai_g1.py               ← Lanzador (sim + policy IA + listener TCP/cmd_vel)
│   ├── fastsac_g1_29dof.onnx          ← Modelo ONNX de locomoción (FastSAC)
│   ├── mujoco_ros2_lidar_bridge.py    ← Bridge MuJoCo → ROS 2 (LiDAR + odom + TF)
│   ├── mujoco_slam_mapper.py          ← Mapeador de rejilla de ocupación
│   ├── maps/                          ← Mapas guardados (PGM + YAML, formato nav2)
│   └── g1_actions.py                  ← Utilidades de acciones del robot
├── nav2/
│   ├── nav2_params.yaml               ← Parámetros de Nav2 (AMCL, costmaps, DWB)
│   ├── g1_nav2.launch.py              ← Launch file que arranca la pila Nav2
│   └── nav2_cmd_vel_bridge.py         ← Puente /cmd_vel → TCP 6000 (Nav2 → policy)
├── rviz2/
│   └── lidar_maze.rviz                ← RViz2 (LiDAR + mapa + Nav2 + AMCL)
├── camera/
│   └── g1_client_mujoco.py            ← Cliente Tkinter de teleop (RGB + WASD)
└── Dockerfile                         ← Imagen Docker del entorno
```

---

## Descripción de cada componente

### 1. `config.py` — Configuración de la simulación

Define los parámetros globales del simulador: robot a usar (`g1`), escena (`scene.xml`), paso temporal de la física (`0.005 s`), frecuencia del visor (`50 fps`), ID de dominio DDS y interfaz de red para la comunicación Unitree SDK2.

### 2. `scene.xml` — Escena del laberinto

Archivo XML en formato MJCF que describe el entorno. Incluye el modelo del robot (`g1_29dof.xml`) y define:

- **Suelo** con material de tablero de ajedrez.
- **Muros exteriores** (4 paredes formando un recinto de 10 m × 10 m).
- **Muros interiores** (9 segmentos que forman el laberinto con pasillos de 2 m).

Todos los geoms del entorno pertenecen al **grupo 3** de MuJoCo. Esto permite que el LiDAR los detecte mediante el parámetro `geomgroup` sin interferir con el renderizado por defecto del visor (que muestra grupos 0–2). Para ver los muros en el visor de MuJoCo, se debe pulsar la tecla **3**.

### 3. `g1_29dof.xml` — Modelo del robot

Modelo MJCF completo del Unitree G1 con 29 articulaciones actuadas. Contiene:

- **Cuerpos y joints** de pelvis, cintura, torso, brazos y piernas.
- **Mallas STL** de cada eslabón (carpeta `meshes/`).
- **Sensores**: IMU en pelvis y torso, encoders en cada articulación.
- **Cámara** `realsense` montada en la cabeza.
- **Site `lidar_site`** en el torso, que define el punto de origen del LiDAR simulado.

La posición del LiDAR se controla con el atributo `pos` del site `lidar_site` (línea 222). El tercer valor es la coordenada Z relativa al torso: `0.3` lo sitúa en el pecho, `0.5` en los hombros.

### 4. `unitree_mujoco.py` — Simulador principal

Es el proceso central de la simulación. Ejecuta cuatro hilos en paralelo:

| Hilo | Función |
|------|---------|
| **SimulationThread** | Avanza la física de MuJoCo a 200 Hz (paso de 0.005 s). Recibe comandos articulares de la policy a través de Unitree SDK2. |
| **PhysicsViewerThread** | Sincroniza el visor 3D de MuJoCo a 50 fps. |
| **LidarThread** | Ejecuta el ray-casting 360° a 10 Hz desde el `lidar_site`. Publica la nube de puntos y las poses del robot vía ZMQ en el puerto 5556. |
| **RGBServerThread** | Renderiza la imagen de la cámara `realsense` y la publica vía ZMQ en el puerto 5555. |
| **ResetServerThread** | Escucha en el puerto UDP 6005. Al recibir `"reset"`, teletransporta al robot a la posición vertical inicial. |

**LiDAR 360° por ray-casting:** El hilo `LidarThread` utiliza la función `mj_ray` de MuJoCo para lanzar 360 rayos (1° de resolución) desde la posición del `lidar_site` en el plano horizontal. Cada rayo detecta el primer geom del grupo 3 que intercepta, hasta un alcance máximo de 12 m. Los puntos de impacto se convierten al frame local del LiDAR y se envían por ZMQ junto con las poses del pelvis y del site.

**Protocolo ZMQ** (puerto 5556, formato binario):

| Campo | Tipo | Descripción |
|-------|------|-------------|
| magic | uint32 | Valor fijo `0xDEAD1337` para validación |
| n_pts | uint32 | Número de puntos en la nube |
| pelvis_pose | 7 × float64 | Pose del pelvis: x, y, z, qw, qx, qy, qz |
| lidar_pose | 7 × float64 | Pose del lidar_site: x, y, z, qw, qx, qy, qz |
| timestamp | float64 | Marca temporal (epoch) |
| points | n_pts × 3 × float32 | Coordenadas XYZ en frame local del LiDAR |

### 5. `run_sim_ai_g1.py` — Lanzador y controlador de locomoción

Es el punto de entrada principal. Realiza cuatro funciones:

1. **Lanza el simulador** (`unitree_mujoco.py`) como subproceso.
2. **Ejecuta la policy de locomoción** cargando el modelo ONNX (`fastsac_g1_29dof.onnx`) y enviando comandos articulares a 50 Hz a través de Unitree SDK2.
3. **Escucha comandos en el puerto TCP 6000**. Acepta caracteres simples (`'w'`, `'a'`, `'s'`, `'d'`, `'q'`, `'e'`, `'stop'`) que se mapean a las velocidades `vx`, `vy` y `yaw` del policy. Este protocolo lo usan tanto el cliente Tkinter de teleop manual como el puente de Nav2 (ver sección 11).

Incluye detección automática de caídas: si el vector de gravedad proyectado indica que el robot no está vertical, envía un comando de reset al simulador para teletransportarlo a la posición erguida.

### 6. `mujoco_ros2_lidar_bridge.py` — Bridge MuJoCo → ROS 2

Nodo de ROS 2 que traduce los datos ZMQ del simulador al ecosistema ROS 2. Publica:

| Topic | Tipo de mensaje | QoS | Descripción |
|-------|----------------|-----|-------------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | RELIABLE | Nube de puntos 3D en frame `lidar_link` |
| `/scan` | `sensor_msgs/LaserScan` | RELIABLE | Escaneo 2D (360 rangos) en frame `lidar_link`. Lo consumen el mapper y los costmaps de Nav2. |
| `/odom` | `nav_msgs/Odometry` | RELIABLE | Odometría del robot (posición, orientación y velocidades en `base_link`). |
| `/tf` | TF dinámicos | — | `odom → base_link` y `base_link → lidar_link` |

**QoS RELIABLE:** Todos los topics son RELIABLE en lugar de BEST_EFFORT porque Nav2 (especialmente los costmaps) suscribe los sensores con perfil RELIABLE por defecto. Si los QoS no coinciden, los mensajes no llegan aunque el topic esté publicándose.

**Árbol de transforms (TF):**

```
[map]              ← lo añade AMCL cuando Nav2 está activo
   ↓
 odom              ← raíz del bridge
   ↓ (dinámico)
base_link
   ↓ (dinámico)
lidar_link
```

El bridge **no publica el frame `world`**. Cuando solo está corriendo el mapper, `odom` es la raíz del árbol TF. Cuando arranca Nav2, AMCL añade el frame `map` por encima de `odom` mediante su propio scan matching contra el mapa estático. Así no hay árboles desconectados.

**Plano de navegación estable (Z = 0):** Como el robot es bípedo, la pelvis del G1 oscila en altura al caminar. Para que la navegación 2D no se vea afectada por estas oscilaciones verticales, el bridge fuerza la coordenada Z del TF `odom → base_link` a `0.0`. La altura real del LiDAR se compensa en el TF `base_link → lidar_link`, de modo que los rayos siguen partiendo del punto físico correcto pero el plano de costmaps queda perfectamente plano. Sin este truco, el local costmap (frame `odom`) y el global costmap (frame `map`) aparecerían a alturas distintas en RViz y los planificadores se confundirían.

La conversión de `PointCloud2` a `LaserScan` se realiza calculando el ángulo y la distancia de cada punto en el plano XY local, y asignándolos al bin angular correspondiente (360 bins de 1°).

Las velocidades lineales y angulares del mensaje `Odometry` se calculan por diferencia finita entre frames consecutivos de la pose del pelvis.

### 7. `mujoco_slam_mapper.py` — Mapeador de rejilla de ocupación

Nodo de ROS 2 que construye un mapa 2D del entorno a partir del LiDAR y la odometría. Implementa un algoritmo de **mapping con poses conocidas** (la odometría del simulador es exacta, sin drift).

**Funcionamiento:**

1. Recibe cada escaneo del topic `/scan` y la pose actual del topic `/odom`.
2. Para cada rayo del escaneo, traza una línea desde la posición del robot hasta el punto de impacto usando el **algoritmo de Bresenham**.
3. Las celdas atravesadas por el rayo se actualizan como **libres** (log-odds −0.4).
4. La celda del punto de impacto se actualiza como **ocupada** (log-odds +0.85).
5. Publica el mapa resultante en `/map` como `nav_msgs/OccupancyGrid` a 2 Hz.

**Parámetros del mapa:**

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| Resolución | 0.05 m | Tamaño de cada celda (5 cm) |
| Dimensiones | 400 × 400 celdas | Cubre 20 m × 20 m |
| Origen | (−10, −10) | Esquina inferior izquierda del mapa |
| Clamp log-odds | [−5, +5] | Evita saturación excesiva |

**Comandos de teclado** (en la terminal donde se ejecuta el mapper):

| Tecla | Acción |
|-------|--------|
| `m` | Guarda el mapa en `mujoco/maps/` en formato PGM + YAML |
| `r` | Resetea el mapa (borra todo y empieza de cero) |
| `q` | Cierra el nodo de forma limpia |

**Formato de guardado:** El mapa se guarda como un par de archivos compatibles con `nav2_map_server`:

- **`.pgm`** — Imagen en escala de grises (P5): 254 = libre, 0 = ocupado, 205 = desconocido.
- **`.yaml`** — Metadatos: ruta de la imagen, resolución, origen y umbrales de ocupación.

### 8. `lidar_maze.rviz` — Configuración de RViz2

Preconfiguración de RViz2 que muestra simultáneamente:

- **Grid** de referencia (20 × 20 m, celdas de 1 m).
- **TF** de todos los frames (map, odom, base_link, lidar_link).
- **LiDAR Points** — nube de puntos instantánea (verde, topic `/lidar/points`).
- **PointCloud2** — nube acumulada con persistencia de 30 s (coloreada por intensidad).
- **Odometry** — flechas azules mostrando la trayectoria (hasta 500 poses).
- **Map** — mapa de ocupación (publicado por el mapper o por `map_server` de Nav2).
- **LaserScan** — escaneo 2D instantáneo (amarillo, topic `/scan`).
- **Global Plan** — ruta global calculada por Nav2 (verde).
- **Local Plan** — trayectoria local del DWB controller (naranja).
- **Global/Local Costmap** — capas de coste de Nav2 (deshabilitadas por defecto, activar si depuras).
- **AMCL Particles** — nube de partículas de localización (`/particle_cloud`).

El **Fixed Frame** está configurado a `map`, que es el frame raíz cuando AMCL está activo. También incluye las herramientas **2D Pose Estimate** (publica en `/initialpose` para inicializar AMCL) y **Nav2 Goal** (publica en `/goal_pose`).

### 9. `nav2/nav2_params.yaml` — Configuración de Nav2

Archivo YAML con todos los parámetros de la pila Navigation2 adaptados al G1. Cada bloque configura un nodo distinto:

#### AMCL — Localización por scan matching

`AMCL` (Adaptive Monte Carlo Localization) es el localizador. Mantiene una nube de partículas (entre 500 y 2000) que representan posibles poses del robot dentro del mapa. En cada escaneo del LiDAR:

1. Compara el `LaserScan` recibido con lo que vería desde cada partícula si el mapa fuera correcto.
2. Las partículas que mejor encajan ganan peso.
3. Se hace un resample dando preferencia a las partículas con más peso.
4. La pose estimada (media ponderada) se publica como TF `map → odom`.

Configurado con:
- **Modelo de movimiento**: `nav2_amcl::OmniMotionModel` — el G1 puede strafear lateralmente (no es solo "diferencial"), así que necesita un modelo omnidireccional.
- **Pose inicial**: `(0, 0, 0)` con `set_initial_pose: true`. Esto evita tener que dar manualmente "2D Pose Estimate" si el robot arranca cerca del origen.
- **Likelihood field**: modelo de sensor que usa una transformada de distancia precomputada del mapa para acelerar el matching.

#### Costmaps (global y local)

Los costmaps son rejillas que asignan a cada celda un coste de transitarla (0 = libre, 254 = ocupada). Hay dos:

- **Global costmap** (frame `map`): cubre todo el mapa estático cargado por `map_server`. Usado por el planner global.
  - **Capas:** `static_layer` (lee `/map` del map_server), `obstacle_layer` (añade obstáculos detectados en vivo por el LiDAR), `inflation_layer` (infla los obstáculos según el radio del robot).

- **Local costmap** (frame `odom`): ventana móvil de 4 × 4 m centrada en el robot. Usado por el controller local.
  - **Capas:** `obstacle_layer` + `inflation_layer`. Sin static_layer porque solo le interesa lo que ve el LiDAR ahora mismo.
  - **Importante:** se usa `obstacle_layer` (2D) y NO `voxel_layer` (3D), porque el LiDAR del G1 está montado a ~1.2 m de altura y un voxel layer rechazaría los rayos al considerarlos fuera del rango Z.

Ambos comparten:
- `robot_radius: 0.30` — el G1 se aproxima por un círculo de 30 cm.
- `inflation_radius: 0.55` — los obstáculos se inflan 55 cm para que el planner deje margen.
- `resolution: 0.05` — celdas de 5 cm.

#### Planner global — `NavfnPlanner`

Implementa **A\*** sobre el global costmap. Cuando recibe un goal, calcula la ruta más corta desde la pose actual hasta el destino, evitando celdas con coste alto. Permite ruta a través de zonas desconocidas (`allow_unknown: true`).

#### Controller local — `DWBLocalPlanner`

Implementa **DWA** (Dynamic Window Approach). En cada ciclo de control (10 Hz):

1. Genera una "ventana" de velocidades alcanzables a partir de la velocidad actual y los límites de aceleración.
2. Para cada combinación `(vx, vy, vθ)` de la ventana, simula la trayectoria 1.5 segundos hacia adelante.
3. Cada trayectoria se puntúa con varios "critics": distancia a obstáculos (`BaseObstacle`), alineación con la ruta global (`PathAlign`), distancia al goal (`GoalDist`), penalización por oscilar (`Oscillation`), etc.
4. Se elige la trayectoria con mejor score y se publica su primer comando como `Twist` en `/cmd_vel_nav`.

Límites de velocidad acordes a lo que el policy del G1 puede ejecutar de forma estable:
- `vx ∈ [-0.4, 0.6]` m/s
- `vy ∈ [-0.3, 0.3]` m/s
- `vθ ∈ [-0.8, 0.8]` rad/s

#### Velocity smoother

Recibe el `cmd_vel_nav` del controller y lo suaviza aplicando límites de aceleración (1.5 m/s² lineal, 2.0 rad/s² angular) antes de publicarlo en `/cmd_vel`. Esto evita que el robot reciba saltos bruscos de velocidad que harían perder el equilibrio al policy de locomoción.

#### Behavior server

Implementa los comportamientos de recovery: `spin` (girar 360° para reorientar), `backup` (retroceder un poco), `drive_on_heading`, `wait`. Se activan automáticamente cuando el controller no puede progresar (atascado contra un obstáculo o sin ruta).

### 10. `nav2/g1_nav2.launch.py` — Launch file de Nav2

Lanza la pila completa de Nav2 con un solo comando. Arranca los siguientes nodos en este orden:

1. **`map_server`** — carga el mapa PGM/YAML y lo publica en `/map`.
2. **`amcl`** — localizador.
3. **`planner_server`** — planner global (NavFn).
4. **`controller_server`** — controller local (DWB).
5. **`velocity_smoother`** — suavizador con remapeo `cmd_vel_nav` → `cmd_vel`.
6. **`behavior_server`** — recoveries.
7. **`bt_navigator`** — Behavior Tree que orquesta el flujo: recibe goal → planner → controller → recovery si falla.
8. **`waypoint_follower`** — para misiones multi-waypoint (no usado en flujo básico).
9. **`lifecycle_manager_navigation`** — activa todos los nodos lifecycle en orden.

Por defecto carga el mapa más reciente de `mujoco/maps/`. Se puede sobreescribir con:
```bash
ros2 launch g1_nav2.launch.py map:=/ruta/absoluta/mapa.yaml
```

### 11. `nav2/nav2_cmd_vel_bridge.py` — Puente Nav2 → Policy de locomoción

Pieza clave que conecta el mundo ROS 2 (Nav2) con el mundo del policy (`run_sim_ai_g1.py`). Su función es traducir los `Twist` de `/cmd_vel` en los comandos TCP que ya entiende el simulador.

**¿Por qué es necesario?** Porque `run_sim_ai_g1.py` se ejecuta en un entorno Python que normalmente no tiene `rclpy` disponible (usa el SDK de Unitree con dependencias propias). Hacer que `run_sim_ai_g1.py` suscriba directamente `/cmd_vel` requeriría sourcing manual de ROS 2 cada vez. La solución más robusta es un puente independiente que vive en el lado de ROS 2 y reenvía las velocidades al puerto TCP 6000 que `run_sim_ai_g1.py` ya escucha (es exactamente el mismo protocolo que usa el cliente Tkinter `g1_client_mujoco.py`).

**Cómo decide qué tecla mandar:** En cada `Twist` recibido, calcula los pesos de cada eje:
- `score_x = |linear.x|`
- `score_y = |linear.y|`
- `score_θ = |angular.z| × 0.5`  (factor para que rad/s pueda competir con m/s)

Y elige la tecla del eje dominante:
- `vx > 0` → `'w'` (avanzar) | `vx < 0` → `'s'` (retroceder)
- `vy > 0` → `'a'` (strafe izquierda) | `vy < 0` → `'d'` (strafe derecha)
- `vθ > 0` → `'q'` (rotar izquierda) | `vθ < 0` → `'e'` (rotar derecha)

Si las tres componentes están por debajo de la dead zone (`0.05 m/s` lineal, `0.10 rad/s` angular), envía `'stop'`.

**Optimizaciones:**

- **Rate limiting** a 20 Hz para no saturar el TCP.
- **Solo envía cuando cambia el comando**, evitando inundar el puerto con la misma tecla repetida.
- **Stop al cerrar**: si se mata el bridge con Ctrl+C, envía un último `'stop'` para que el robot no siga caminando solo.

Esta arquitectura preserva todo el código original y permite que el cliente Tkinter siga funcionando en paralelo: cuando teleoperas con WASD desde el cliente, los comandos llegan directamente al TCP y machacan a los de Nav2 (lo que llegue último gana). Esto te da una vía manual de intervención durante la navegación autónoma.

---

## ¿Cómo se obtiene la odometría y la localización?

Una de las partes más importantes (y confusas) del sistema es entender la cadena completa de transformadas que permite saber dónde está el robot. Aquí va el desglose paso a paso:

### Paso 1 — Pose ground truth desde MuJoCo

El simulador MuJoCo conoce exactamente la pose de cada cuerpo del robot porque la calcula él mismo a partir de la integración de las ecuaciones físicas. En cada paso de simulación (200 Hz), el hilo `LidarThread` lee la pose del `body name="pelvis"` mediante la API `mj_data.xpos[body_id]` y `mj_data.xmat[body_id]`. Esto da una pose perfecta, sin ruido y sin drift, en el frame world de MuJoCo.

Es **ground truth**: en simulación tenemos información que en un robot real solo podríamos estimar con sensores ruidosos (encoders + IMU + visual odometry). Esta ventaja la usamos a fondo.

### Paso 2 — Envío por ZMQ

La pose del pelvis se serializa junto con la nube de puntos del LiDAR y la pose del `lidar_site`, y se envía por ZMQ al puerto 5556. El protocolo binario incluye un magic word (`0xDEAD1337`), el número de puntos y las dos poses (pelvis y lidar) como vectores de 7 floats `(x, y, z, qw, qx, qy, qz)`.

### Paso 3 — Bridge ROS 2: publicación de `/odom` y TF

El nodo `mujoco_ros2_lidar_bridge.py` recibe el mensaje ZMQ y hace tres cosas con la pose del pelvis:

1. **Publica `nav_msgs/Odometry`** en `/odom`. Este mensaje contiene la posición y orientación del robot, además de las velocidades lineales y angulares calculadas por diferencia finita entre frames consecutivos.

2. **Publica el TF dinámico `odom → base_link`**. Este es el transform que cualquier nodo puede consultar para saber dónde está el robot relativo al frame de odometría. **Forzamos `Z = 0`** en este TF para mantener el plano de navegación estable, ya que el bípedo oscila verticalmente al caminar.

3. **Publica el TF dinámico `base_link → lidar_link`**. Aquí sí se incluye la altura real del LiDAR (≈1.2 m) para que los rayos del escaneo se proyecten desde el punto físico correcto.

En este punto, el árbol TF queda así:

```
odom (raíz)
   ↓
base_link (Z=0, plano)
   ↓
lidar_link (~1.2 m de altura)
```

### Paso 4 — Mapper o Nav2 cierran el bucle

A partir de aquí hay dos modos de funcionamiento:

#### Modo A — Mapeo

El nodo `mujoco_slam_mapper.py` suscribe `/odom` y `/scan`. Como la odometría es ground truth (sin drift), no necesita scan matching: integra cada escaneo directamente en el grid usando la pose del pelvis como verdad absoluta. Esto se llama **"mapping with known poses"**, el escenario más simple del SLAM. El resultado es un mapa publicado en `/map` que el usuario puede guardar pulsando `m`.

En este modo, el frame raíz sigue siendo `odom`. RViz debe tener `Fixed Frame = odom`.

#### Modo B — Navegación

Cuando se carga el mapa guardado y arranca Nav2:

1. **`map_server`** publica el mapa estático en `/map` con QoS `transient_local` (para que los suscriptores tardíos también lo reciban).

2. **`AMCL`** suscribe `/scan`, `/map` y la nube de partículas. En cada escaneo:
   - Compara el `LaserScan` con lo que vería desde cada partícula si esa fuera la pose real del robot.
   - Reajusta los pesos de las partículas y hace resampling.
   - Calcula la pose media ponderada → esa es la estimación de la pose del robot en el frame `map`.
   - **Publica el TF `map → odom`** que corrige cualquier drift de la odometría.

3. El árbol TF completo queda así:

```
map (raíz, lo añade AMCL)
   ↓ (lo publica AMCL en función de su scan matching)
odom
   ↓ (lo publica el bridge desde la pose ground truth)
base_link
   ↓ (lo publica el bridge)
lidar_link
```

En este modo, RViz debe tener `Fixed Frame = map`.

### Paso 5 — Cierre del bucle de control

Con el robot localizado en `map`, el flujo de navegación es:

1. El usuario publica un goal en `/goal_pose` desde el botón **Nav2 Goal** de RViz.
2. **`bt_navigator`** ejecuta el behavior tree: invoca al **`planner_server`** con el goal.
3. **`planner_server`** (NavFn) calcula una ruta A\* sobre el global costmap y la publica en `/plan`.
4. **`bt_navigator`** invoca al **`controller_server`** pasándole esa ruta.
5. **`controller_server`** (DWB) muestrea trayectorias en su ventana dinámica, evalúa críticos sobre el local costmap, elige la mejor y publica el `Twist` resultante en `/cmd_vel_nav`.
6. **`velocity_smoother`** lo suaviza con límites de aceleración y lo republica en `/cmd_vel`.
7. **`nav2_cmd_vel_bridge.py`** suscribe `/cmd_vel`, decide la tecla dominante (`w`/`a`/`s`/`d`/`q`/`e`/`stop`) y la envía por TCP al puerto 6000.
8. **`run_sim_ai_g1.py`** recibe el comando TCP y actualiza las variables `vx`, `vy`, `yaw` que alimentan al policy ONNX.
9. El **policy de locomoción** genera las posiciones articulares objetivo cada 20 ms y las publica vía Unitree SDK2.
10. El **simulador MuJoCo** integra la física y el robot da un paso.
11. La nueva pose se vuelve a leer en el Paso 1 y el ciclo se repite.

Todo este bucle corre en tiempo real con varios procesos y dos middlewares distintos (ZMQ y ROS 2 / DDS) coexistiendo sin interferirse.

---

## Dependencias

| Paquete | Versión | Propósito |
|---------|---------|-----------|
| Ubuntu | 22.04 | Sistema operativo |
| ROS 2 | Humble | Middleware de robótica |
| MuJoCo | ≥ 3.0 | Motor de física |
| Python | 3.10 | Lenguaje de ejecución |
| numpy | ≥ 1.23 | Operaciones numéricas |
| zmq (pyzmq) | ≥ 25.0 | Comunicación entre procesos |
| opencv-python | ≥ 4.7 | Procesamiento de imagen |
| onnxruntime | ≥ 1.15 | Inferencia del modelo de locomoción |
| unitree_sdk2py | — | SDK de comunicación Unitree |
| tf2_ros | (ROS 2) | Publicación de transforms |
| nav_msgs | (ROS 2) | Mensajes de odometría y mapa |
| sensor_msgs | (ROS 2) | Mensajes de LiDAR y cámara |

---

## Flujo de trabajo completo (mapeo + navegación)

> **Nota importante:** Antes de cualquier comando que use `ros2`, asegúrate de tener ROS 2 sourced en esa terminal:
> ```bash
> source /opt/ros/humble/setup.bash
> ```
> Conviene añadirlo al `~/.bashrc` para evitar olvidos. El simulador `run_sim_ai_g1.py` NO lo necesita (no usa rclpy directamente).

El proyecto se usa en dos fases bien diferenciadas:

### Fase A — Mapeo del entorno

El objetivo es construir un mapa del laberinto que luego servirá para navegar.

1. **Terminal 1** — simulación + policy:
   ```bash
   cd g1man/mujoco && python3 run_sim_ai_g1.py
   ```

2. **Terminal 2** — bridge ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   cd g1man/mujoco && python3 mujoco_ros2_lidar_bridge.py
   ```

3. **Terminal 3** — mapper:
   ```bash
   source /opt/ros/humble/setup.bash
   cd g1man/mujoco && python3 mujoco_slam_mapper.py
   ```

4. **Terminal 4** — RViz2:
   ```bash
   source /opt/ros/humble/setup.bash
   cd g1man && rviz2 -d rviz2/lidar_maze.rviz
   ```
   En RViz, asegúrate de que **Fixed Frame = `odom`** durante el mapeo.

5. **Terminal 5** — cliente de teleop manual:
   ```bash
   cd g1man/camera && python3 g1_client_mujoco.py
   ```

6. **Pasea al robot** por todo el laberinto con WASD/QE hasta que el mapa esté completo. Verás cómo se rellena en RViz.

7. **Pulsa `m`** en la terminal del mapper. Se guarda un par `maze_map_TIMESTAMP.pgm` + `.yaml` en `mujoco/maps/`.

8. Cuando estés satisfecho, **mata el mapper** (`q` o `Ctrl+C`).

### Fase B — Navegación con Nav2

Ahora cargas el mapa guardado y dejas que Nav2 se encargue de la navegación.

1. Mantén abiertas la **Terminal 1** (simulación), **Terminal 2** (bridge) y **Terminal 4** (RViz2). Cierra el mapper.

2. **Cambia el Fixed Frame de RViz a `map`** (esquina superior izquierda del panel Displays).

3. **Terminal 3** — Nav2:
   ```bash
   source /opt/ros/humble/setup.bash
   cd g1man/nav2 && ros2 launch g1_nav2.launch.py
   ```
   Por defecto carga el mapa más reciente de `mujoco/maps/`. Para usar uno específico:
   ```bash
   ros2 launch g1_nav2.launch.py map:=/home/usuario/g1man/mujoco/maps/maze_map_TIMESTAMP.yaml
   ```

4. **Terminal 5** — puente cmd_vel → TCP:
   ```bash
   source /opt/ros/humble/setup.bash
   cd g1man/nav2 && python3 nav2_cmd_vel_bridge.py
   ```
   Sin este puente, las velocidades de Nav2 no llegan al simulador y el robot no se moverá aunque la ruta aparezca en RViz.

5. **En RViz2**:
   - Verás el mapa cargado por `map_server` y la nube de partículas roja de AMCL alrededor del robot.
   - (Opcional) Pulsa **2D Pose Estimate** y haz clic donde está el robot, arrastrando para indicar su orientación. AMCL reinicializa sus partículas. Si el robot arrancó cerca de (0, 0) este paso es opcional porque AMCL ya tiene `set_initial_pose: true`.
   - Pulsa **Nav2 Goal** y haz clic donde quieres que vaya el robot, arrastrando para indicar la orientación final.
   - Verás aparecer:
     - **Global Plan** (verde) — la ruta calculada por NavFn.
     - **Local Plan** (naranja) — la trayectoria del DWB controller en tiempo real.
   - El robot empezará a caminar autónomamente hacia el destino.

### Resolución de problemas comunes

- **Nav2 publica `/cmd_vel` pero el robot no se mueve**: comprueba que `nav2_cmd_vel_bridge.py` está corriendo. Sin él, las velocidades no llegan al simulador. Puedes verificar con `ros2 topic info /cmd_vel`: deben aparecer al menos 1 publisher (Nav2) y 1 subscriber (el bridge).
- **AMCL pierde la localización**: usa "2D Pose Estimate" para reinicializar.
- **RViz dice "No transform from [world] to [map]"**: el bridge ya no publica el frame `world`. Cambia el Fixed Frame de RViz a `map` (con Nav2 activo) o a `odom` (sin Nav2).
- **Costmaps a alturas distintas**: el bridge ya fuerza Z=0 en `odom → base_link`. Si los sigues viendo desfasados, asegúrate de tener la versión actualizada del bridge.
- **El planner no encuentra ruta**: el footprint del robot (radio 30 cm) más el inflation radius (55 cm) puede ser demasiado para tus pasillos de 2 m si el mapa tiene ruido. Reduce `inflation_radius` en `nav2_params.yaml` a 0.35 m.
- **El robot oscila o se atasca**: aumenta `BaseObstacle.scale` en `nav2_params.yaml` o reduce `vx_samples`/`vtheta_samples` para que el DWB explore menos trayectorias.

---

## Comandos rápidos (copiar y pegar)

### Mapeo (5 terminales)

```bash
# Terminal 1 — Sim + IA (no necesita ROS sourced)
cd g1man/mujoco && python3 run_sim_ai_g1.py
```

```bash
# Terminal 2 — Bridge MuJoCo → ROS 2
source /opt/ros/humble/setup.bash
cd g1man/mujoco && python3 mujoco_ros2_lidar_bridge.py
```

```bash
# Terminal 3 — Mapper
source /opt/ros/humble/setup.bash
cd g1man/mujoco && python3 mujoco_slam_mapper.py
```

```bash
# Terminal 4 — RViz2 (Fixed Frame: odom)
source /opt/ros/humble/setup.bash
cd g1man && rviz2 -d rviz2/lidar_maze.rviz
```

```bash
# Terminal 5 — Cliente teleop manual
cd g1man/camera && python3 g1_client_mujoco.py
```

### Navegación (5 terminales)

```bash
# Terminal 1 — Sim + IA
cd g1man/mujoco && python3 run_sim_ai_g1.py
```

```bash
# Terminal 2 — Bridge MuJoCo → ROS 2
source /opt/ros/humble/setup.bash
cd g1man/mujoco && python3 mujoco_ros2_lidar_bridge.py
```

```bash
# Terminal 3 — Nav2 (carga el mapa más reciente automáticamente)
source /opt/ros/humble/setup.bash
cd g1man/nav2 && ros2 launch g1_nav2.launch.py
```

```bash
# Terminal 4 — Puente cmd_vel → TCP (CLAVE para que el robot se mueva)
source /opt/ros/humble/setup.bash
cd g1man/nav2 && python3 nav2_cmd_vel_bridge.py
```

```bash
# Terminal 5 — RViz2 (Fixed Frame: map)
source /opt/ros/humble/setup.bash
cd g1man && rviz2 -d rviz2/lidar_maze.rviz
```

---

## Tareas pendientes

### Navegación autónoma

- [x] Integrar **Nav2** (Navigation2) para planificación de rutas sobre el mapa generado.
- [x] Configurar `nav2_map_server` para cargar los mapas guardados en `mujoco/maps/`.
- [x] Implementar el nodo `nav2_amcl` para localización con scan matching.
- [x] Configurar los costmaps (global y local) adaptados al tamaño del robot G1.
- [x] Definir el footprint del robot para evitar colisiones con los muros.
- [x] Publicar goals de navegación desde RViz2.
- [ ] Crear cliente Python que envíe goals programáticamente (`nav2_simple_commander`).
- [ ] Implementar **NavigateThroughPoses** para misiones multi-waypoint.
- [ ] Tunear los pesos del DWB controller para movimiento más fluido del humanoide.

### Mejoras en percepción

- [ ] Añadir más canales verticales al LiDAR (pasar `LIDAR_N_ELEVATION` de 1 a 16) para obtener un LiDAR 3D tipo Velodyne.
- [ ] Integrar la cámara RGB (`realsense`) con ROS 2 para publicar `sensor_msgs/Image`.
- [ ] Explorar SLAM 3D con los datos del LiDAR multicanal.

### Mejoras en el mapper

- [ ] Implementar **scan matching** (ICP o correlative scan matching) para corregir la pose antes de integrar cada escaneo, haciendo el mapper funcional con odometría ruidosa.
- [ ] Añadir detección de **loop closure** para cerrar bucles en el laberinto.
- [ ] Optimizar el rendimiento del ray-tracing con NumPy vectorizado en lugar del bucle Python actual.
- [ ] Añadir guardado automático del mapa cada N escaneos.

### Exploración

- [ ] Implementar un planificador de exploración tipo **frontier-based** que identifique las zonas desconocidas del mapa y envíe goals de navegación hacia ellas.
- [ ] Integrar con Nav2 para exploración completamente autónoma del laberinto.

### Infraestructura

- [ ] Crear un **launch file** de ROS 2 (`.launch.py`) que arranque bridge, mapper y RViz2 con un solo comando.
- [ ] Parametrizar la configuración del LiDAR (resolución, rango, grupos) como parámetros de ROS 2 en lugar de constantes en el código.
- [ ] Documentar el módulo de cámara (`camera/`).
- [ ] Añadir tests unitarios para las funciones de conversión de quaterniones y el protocolo ZMQ.
