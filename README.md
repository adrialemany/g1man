# G1 Humanoid — Simulación MuJoCo con percepción LiDAR y mapeo

## Descripción general

Este proyecto permite simular el robot humanoide **Unitree G1** (29 grados de libertad) dentro del motor de física **MuJoCo**, dotarlo de un **LiDAR 360°** simulado por ray-casting, y conectar toda la percepción al ecosistema **ROS 2 Humble** para visualización en **RViz2** y construcción de mapas de ocupación.

El robot se mueve por un laberinto de 10 m × 10 m con pasillos de 2 m de ancho. Su locomoción está controlada por una red neuronal (modelo ONNX entrenado con SAC) que produce las posiciones articulares objetivo a 50 Hz.

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
│   ├── run_sim_ai_g1.py               ← Lanzador principal (arranca simulador + policy IA)
│   ├── fastsac_g1_29dof.onnx          ← Modelo ONNX de locomoción (FastSAC)
│   ├── mujoco_ros2_lidar_bridge.py    ← Bridge MuJoCo → ROS 2 (LiDAR + odom + TF)
│   ├── mujoco_slam_mapper.py          ← Mapeador de rejilla de ocupación
│   ├── maps/                          ← Mapas guardados (PGM + YAML, formato nav2)
│   └── g1_actions.py                  ← Utilidades de acciones del robot
├── rviz2/
│   └── lidar_maze.rviz                ← Configuración de visualización para RViz2
├── camera/                            ← Módulo de visión (cámara RGB, no cubierto aquí)
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

Es el punto de entrada principal. Realiza tres funciones:

1. **Lanza el simulador** (`unitree_mujoco.py`) como subproceso.
2. **Ejecuta la policy de locomoción** cargando el modelo ONNX (`fastsac_g1_29dof.onnx`) y enviando comandos articulares a 50 Hz a través de Unitree SDK2.
3. **Escucha comandos de movimiento** en el puerto TCP 6000 (teclas WASD para velocidad lineal y QE para rotación).

Incluye detección automática de caídas: si el vector de gravedad proyectado indica que el robot no está vertical, envía un comando de reset al simulador.

### 6. `mujoco_ros2_lidar_bridge.py` — Bridge MuJoCo → ROS 2

Nodo de ROS 2 que traduce los datos ZMQ del simulador al ecosistema ROS 2. Publica:

| Topic | Tipo de mensaje | Descripción |
|-------|----------------|-------------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | Nube de puntos 3D en frame `lidar_link` |
| `/scan` | `sensor_msgs/LaserScan` | Escaneo 2D (360 rangos) en frame `lidar_link`. Necesario para algoritmos de SLAM. |
| `/odom` | `nav_msgs/Odometry` | Odometría del robot (posición, orientación y velocidades). |
| `/tf` | TF dinámicos | `odom → base_link` y `base_link → lidar_link` |
| `/tf_static` | TF estático | `world → odom` (identidad, dado que la odometría es ground truth) |

**Árbol de transforms (TF):**

```
world → odom → base_link → lidar_link
  (estático)  (dinámico)    (dinámico)
```

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
- **TF** de todos los frames (world, odom, base_link, lidar_link).
- **LiDAR Points** — nube de puntos instantánea (verde, topic `/lidar/points`).
- **PointCloud2** — nube acumulada con persistencia de 30 s (coloreada por intensidad).
- **Odometry** — flechas azules mostrando la trayectoria (hasta 500 poses).
- **Map** — mapa de ocupación del mapper (topic `/map`).
- **LaserScan** — escaneo 2D instantáneo (amarillo, topic `/scan`).

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

## Guía de ejecución

### Paso 1 — Simulación + locomoción IA

Abre una terminal y ejecuta:

```bash
cd g1man/mujoco
python3 run_sim_ai_g1.py
```

Este comando lanza el simulador de MuJoCo (incluyendo el LiDAR 360° y la cámara RGB) y la policy de locomoción del robot. Se abrirá el visor 3D de MuJoCo. Para ver los muros del laberinto en el visor, pulsa la tecla **3**.

Para mover al robot, usa un cliente TCP al puerto 6000 enviando: `w` (avanzar), `s` (retroceder), `a` (izquierda), `d` (derecha), `q` (rotar izquierda), `e` (rotar derecha), `stop` (detenerse).

### Paso 2 — Bridge ROS 2

En una segunda terminal:

```bash
cd g1man/mujoco
python3 mujoco_ros2_lidar_bridge.py
```

Traduce los datos del simulador (ZMQ) a topics de ROS 2. A partir de este momento, los topics `/scan`, `/odom`, `/lidar/points` y los transforms `/tf` están disponibles en la red ROS 2.

### Paso 3 — Mapeador

En una tercera terminal:

```bash
cd g1man/mujoco
python3 mujoco_slam_mapper.py
```

Comienza a construir el mapa de ocupación. Mientras el robot se mueve, el mapa se va completando. Pulsa **m** en esta terminal para guardar el mapa en `mujoco/maps/`.

### Paso 4 — Visualización

En una cuarta terminal:

```bash
cd g1man
rviz2 -d rviz2/lidar_maze.rviz
```

Abre RViz2 con la configuración preestablecida. Se mostrará la nube de puntos, la odometría, el mapa de ocupación y los transforms del robot en tiempo real.

---

## Comandos checkpoint.

Para ponerlo todo en marcha desde cero, ejecuta cada bloque en una terminal distinta:

```bash
# Terminal 1 — Simulación + IA
cd g1man/mujoco && python3 run_sim_ai_g1.py
```

```bash
# Terminal 2 — Bridge ROS 2
cd g1man/mujoco && python3 mujoco_ros2_lidar_bridge.py
```

```bash
# Terminal 3 — Mapper
cd g1man/mujoco && python3 mujoco_slam_mapper.py
```

```bash
# Terminal 4 — RViz2
cd g1man && rviz2 -d rviz2/lidar_maze.rviz
```

---

## Tareas pendientes

### Navegación autónoma

- [ ] Integrar **Nav2** (Navigation2) para planificación de rutas sobre el mapa generado.
- [ ] Configurar `nav2_map_server` para cargar los mapas guardados en `mujoco/maps/`.
- [ ] Implementar el nodo `nav2_amcl` para localización (o usar la odometría ground truth como localización directa).
- [ ] Definir el footprint del robot para evitar colisiones con los muros.
- [ ] Publicar goals de navegación desde RViz2 o programáticamente.

### Mejoras en percepción.

- [ ] Añadir más canales verticales al LiDAR (pasar `LIDAR_N_ELEVATION` de 1 a 16) para obtener un LiDAR 3D tipo Velodyne.
- [ ] Explorar SLAM 3D con los datos del LiDAR multicanal.

### Mejoras en el mapper

- [ ] Implementar **scan matching** (ICP o correlative scan matching) para corregir la pose antes de integrar cada escaneo, haciendo el mapper funcional con odometría ruidosa.
- [ ] Añadir detección de **loop closure** para cerrar bucles en el laberinto.
- [ ] Añadir guardado automático del mapa cada N escaneos.

### Exploración

- [ ] Implementar un planificador de exploración tipo **frontier-based** que identifique las zonas desconocidas del mapa y envíe goals de navegación hacia ellas.
- [ ] Integrar con Nav2 para exploración completamente autónoma del laberinto.

### Infraestructura

- [ ] Crear un **launch file** de ROS 2 (`.launch.py`) que arranque bridge, mapper y RViz2 con un solo comando.
- [ ] Parametrizar la configuración del LiDAR (resolución, rango, grupos) como parámetros de ROS 2 en lugar de constantes en el código.
- [ ] Documentar el módulo de cámara (`camera/`).
- [ ] Añadir tests unitarios para las funciones de conversión de quaterniones y el protocolo ZMQ.
