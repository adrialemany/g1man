# G1Man — Stack de Navegación Autónoma para Unitree G1

Proyecto TFG: stack completo de navegación autónoma para el robot humanoide **Unitree G1** simulado en **MuJoCo**, con integración **ROS 2 Humble** y soporte para dos pilas de navegación: **Nav2** y **EasyNavigation (URJC)**.

---

## Estructura del proyecto

```
g1man/
├── teleop/                         # Teleoperación manual del robot
│   └── g1_client_mujoco.py         # Cliente Tkinter: vídeo + control WASD multi-tecla (TCP:6000)
│
├── navegacion/                     # Todo lo relativo a navegación autónoma
│   ├── cmd_vel_bridge.py           # Puente genérico /cmd_vel → TCP:6000 (obsoleto, ver nota)
│   ├── nav2/                       # Stack Nav2 (ROS 2 Humble)
│   │   ├── g1_nav2.launch.py       # Launch: AMCL + costmaps + planner + BT Navigator
│   │   ├── nav2_params.yaml        # Parámetros de Nav2 (footprint poligonal del G1)
│   │   └── nav2_cmd_vel_bridge.py  # Bridge Nav2 (obsoleto, integrado en run_sim_ai_g1.py)
│   └── easynav/                    # Stack EasyNavigation (ROS 2 Jazzy, Docker)
│       ├── g1_easynav.launch.py    # Launch para ejecutar dentro del contenedor
│       ├── config/
│       │   └── easynav_params.yaml # Parámetros del SimpleStack de EasyNav
│       ├── docker/
│       │   ├── Dockerfile          # Imagen g1-easynav:jazzy
│       │   ├── entrypoint.sh
│       │   ├── build_easynav_docker.sh
│       │   └── run_easynav_docker.sh
│       ├── BUG_REPORT.md           # Bug conocido: std::runtime_error time sources
│       └── README.md               # Documentación específica de EasyNav
│
├── mujoco/                         # Simulación MuJoCo
│   ├── simulacion/                 # Motor de simulación y código de control
│   │   ├── run_sim_ai_g1.py        # PUNTO DE ENTRADA: lanza simulador + policy IA + bridge Nav2
│   │   ├── unitree_mujoco.py       # Visor MuJoCo + LiDAR ZMQ + cámara ZMQ + cámara 3ª persona (C)
│   │   ├── unitree_sdk2py_bridge.py# Bridge SDK2 ↔ MuJoCo
│   │   ├── config.py               # Configuración del simulador (robot, scene, DDS)
│   │   ├── fastsac_g1_29dof.onnx  # Modelo de locomoción (política SAC)
│   │   ├── mujoco_ros2_lidar_bridge.py  # Publica /scan, /odom, /tf desde ZMQ
│   │   ├── mujoco_slam_mapper.py   # Mapper de ocupación casero → guarda en maps/
│   │   ├── g1_actions.py           # Control de brazos vía UDP:9876
│   │   ├── vision.py               # Utilidades de visión
│   │   ├── scene.xml               # Escena MuJoCo (laberinto simple)
│   │   ├── scene_from_sdf_centered.xml  # Escena MuJoCo (edificio TI)
│   │   ├── g1_29dof.xml            # Modelo MJCF del robot G1
│   │   └── meshes/                 # Mallas STL del robot
│   ├── worlds/                     # Entornos de simulación
│   │   └── tibuilding/             # Edificio TI (mallas OBJ/DAE + colisiones)
│   └── creator_editor/             # Herramientas de creación de mundos
│       ├── image_to_mujoco.py      # Convierte imagen de plano → XML MuJoCo
│       ├── limpiar_plano.py        # Limpieza/preprocesado de planos PNG
│       └── salida-1_walls.xml      # XML de paredes generado para escena de prueba
│
├── maps/                           # Mapas generados y planos de referencia
│   ├── maze_map_20260406_200946.pgm/.yaml  # Mapa de ejemplo (laberinto)
│   ├── salida-1.png                # Plano PNG de la planta (preprocesado)
│   ├── salida.pdf / TI_n1.pdf      # Planos originales del edificio TI
│   └── TI_1/                       # Modelo SDF del edificio TI (Gazebo/RViz)
│
└── rviz2/                          # Configuraciones RViz2
    ├── g1_teleop.rviz              # Vista para teleoperación
    ├── g1_mapping.rviz             # Vista para mapeo SLAM
    ├── navigation.rviz             # Vista para navegación Nav2
    └── g1_easynav.rviz             # Vista para EasyNavigation
```

---

## Requisito previo — source de ROS 2

**Cada terminal** que ejecute cualquier componente ROS 2 o Python con `rclpy` necesita tener el entorno de ROS 2 cargado. Ejecutar antes de cualquier comando:

```bash
source /opt/ros/humble/setup.bash
```

Para no tener que hacerlo en cada terminal, añadirlo al `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **⚠️ Importante:** Si `run_sim_ai_g1.py` se lanza sin el source, el bridge interno de Nav2 mostrará `[WARN] rclpy no disponible — /cmd_vel deshabilitado` y el robot no responderá a los goals de Nav2.

---

## Flujo de trabajo

### 1 · Simulación + control de locomoción

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
cd mujoco/simulacion
python3 run_sim_ai_g1.py
```

`run_sim_ai_g1.py` lanza internamente `unitree_mujoco.py` y arranca tres servicios en paralelo:
- Servidor de teleop TCP en el puerto 6000 (multi-tecla JSON, retrocompatible)
- Suscriptor ROS 2 a `/cmd_vel` con **prioridad absoluta sobre la teleop**
- Listener de brazos UDP en el puerto 9876

### 2 · Bridge LiDAR → ROS 2

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
cd mujoco/simulacion
python3 mujoco_ros2_lidar_bridge.py
```

### 3A · Teleoperación manual

```bash
# Terminal 3
cd teleop
python3 g1_client_mujoco.py
```

El cliente soporta **múltiples teclas simultáneas**: `W+D` avanza en diagonal, `W+Q` avanza girando, etc. Si Nav2 está activo, el cliente muestra el overlay **"NAV2 ACTIVE — TELEOP BLOCKED"** y los comandos de teclado se ignoran hasta que Nav2 deje de publicar en `/cmd_vel`.

```bash
rviz2 -d rviz2/g1_teleop.rviz
```

### 3B · Mapeo (SLAM casero)

```bash
# Terminal 3
source /opt/ros/humble/setup.bash
cd mujoco/simulacion
python3 mujoco_slam_mapper.py
# Pulsa 'm' para guardar el mapa en maps/
```

```bash
rviz2 -d rviz2/g1_mapping.rviz
```

### 3C · Navegación autónoma con Nav2

> **Nota:** `cmd_vel_bridge.py` y `nav2_cmd_vel_bridge.py` ya **no son necesarios**. El bridge de `/cmd_vel` está integrado directamente en `run_sim_ai_g1.py`.

```bash
# Terminal 3 — stack Nav2 (usa el mapa más reciente de maps/ por defecto)
source /opt/ros/humble/setup.bash
cd navegacion/nav2
ros2 launch g1_nav2.launch.py

# O con mapa explícito:
ros2 launch g1_nav2.launch.py map:=/ruta/absoluta/al/mapa.yaml
```

```bash
rviz2 -d rviz2/navigation.rviz
```

Para mandar un goal desde línea de comandos:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Cuando Nav2 empieza a publicar en `/cmd_vel`, la Terminal 1 mostrará:
```
[🟢 NAV2] Control ACTIVO — teleop bloqueada
```
Y al parar:
```
[🔴 NAV2] Control INACTIVO — teleop libre
```

### 3D · Navegación autónoma con EasyNavigation (Docker, Jazzy)

```bash
# Terminal 3 — bridge /cmd_vel → TCP:6000
source /opt/ros/humble/setup.bash
cd navegacion
python3 cmd_vel_bridge.py

# Terminal 4 — construir imagen Docker (solo la primera vez)
cd navegacion/easynav/docker
./build_easynav_docker.sh

# Terminal 5 — lanzar EasyNav en el contenedor
./run_easynav_docker.sh launch
# O con mapa específico (ruta dentro del contenedor):
./run_easynav_docker.sh launch /g1_maps/maze_map_20260406_200946.yaml
```

```bash
rviz2 -d rviz2/g1_easynav.rviz
```

> **⚠️ Bug conocido:** EasyNav arranca y activa sus nodos correctamente pero lanza un `std::runtime_error: can't compare times with different time sources` en el momento en que intenta procesar el primer scan. Ver `navegacion/easynav/BUG_REPORT.md`.

---

## Cámara en tercera persona

El visor de MuJoCo incluye un modo de cámara en tercera persona que sigue al robot desde detrás.

- **Tecla `C`** (con la ventana de MuJoCo enfocada) → activa/desactiva el seguimiento
- En modo libre la cámara se controla manualmente como siempre
- En modo seguimiento la cámara se mantiene fija detrás del robot y orbita con él al girar

Parámetros ajustables en `unitree_mujoco.py` dentro de `PhysicsViewerThread`:

| Variable | Valor | Descripción |
|---|---|---|
| `CAM_DISTANCE` | `3.5` | Distancia al robot (m) |
| `CAM_ELEVATION` | `-20.0` | Ángulo vertical (grados) |
| `CAM_HEIGHT` | `0.3` | Offset vertical sobre el pelvis (m) |
| `CAM_SMOOTHING` | `0.08` | Suavizado del giro (0 = fija, 1 = instantáneo) |

---

## Crear un nuevo mundo desde un plano

```bash
# 1. Limpiar/binarizar el plano PNG
cd mujoco/creator_editor
python3 limpiar_plano.py

# 2. Convertir a XML MuJoCo
python3 image_to_mujoco.py

# 3. Copiar el XML generado a mujoco/worlds/ o referenciarlo desde scene.xml
```

---

## Configuración clave (`mujoco/simulacion/config.py`)

| Variable | Valor por defecto | Descripción |
|---|---|---|
| `ROBOT` | `"g1"` | Modelo de robot |
| `ROBOT_SCENE` | `"scene_from_sdf_centered.xml"` | Escena activa |
| `DOMAIN_ID` | `1` | ROS 2 Domain ID |
| `INTERFACE` | `"lo"` | Interfaz de red DDS |
| `SIMULATE_DT` | `0.005` | Paso de simulación (s) |

---

## Parámetros de navegación (`navegacion/nav2/nav2_params.yaml`)

El footprint del robot está definido como un **polígono rectangular** que representa la silueta real del G1 con los brazos:

```yaml
footprint: "[[0.14, 0.20], [0.14, -0.20], [-0.14, -0.20], [-0.14, 0.20]]"
```

0.40 m de ancho (eje Y, de hombro a hombro) × 0.28 m de largo (eje X, frente-espalda).

| Parámetro | Valor | Descripción |
|---|---|---|
| `footprint` | polígono 0.40×0.28 m | Silueta real del G1 con brazos |
| `inflation_radius` | `0.22` | Margen de seguridad alrededor de obstáculos |
| `cost_scaling_factor` | `6.0` | Decaimiento del coste de inflación |

Para ajustar el acceso a pasillos estrechos, modificar `inflation_radius` (bajar = más permisivo) y los vértices del footprint.

---

## Puertos y protocolos

| Puerto | Protocolo | Dirección | Descripción |
|---|---|---|---|
| `6000` | TCP | → `run_sim_ai_g1.py` | Comandos de locomoción (JSON multi-tecla o strings clásicos) |
| `5555` | ZMQ PUB | ← `unitree_mujoco.py` | Stream de vídeo RealSense |
| `5556` | ZMQ PUB | ← `unitree_mujoco.py` | Stream LiDAR (puntos + poses) |
| `6005` | UDP | → `unitree_mujoco.py` | Reset de posición del robot |
| `9876` | UDP | → `run_sim_ai_g1.py` | Targets de brazos externos |
