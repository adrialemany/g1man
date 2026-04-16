# G1Man — Stack de Navegación Autónoma para Unitree G1

Proyecto TFG: stack completo de navegación autónoma para el robot humanoide **Unitree G1** simulado en **MuJoCo**, con integración **ROS 2 Humble** y soporte para dos pilas de navegación: **Nav2** y **EasyNavigation (URJC)**.

---

## Estructura del proyecto

```
g1man/
├── teleop/                         # Teleoperación manual del robot
│   └── g1_client_mujoco.py         # Cliente Tkinter: vídeo + control WASD (TCP:6000)
│
├── navegacion/                     # Todo lo relativo a navegación autónoma
│   ├── cmd_vel_bridge.py           # Puente genérico /cmd_vel → TCP:6000 (Nav2 y EasyNav)
│   ├── nav2/                       # Stack Nav2 (ROS 2 Humble)
│   │   ├── g1_nav2.launch.py       # Launch: AMCL + costmaps + planner + BT Navigator
│   │   ├── nav2_params.yaml        # Parámetros de Nav2
│   │   └── nav2_cmd_vel_bridge.py  # Bridge específico Nav2 (alternativo a cmd_vel_bridge.py)
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
│   │   ├── run_sim_ai_g1.py        # PUNTO DE ENTRADA: lanza simulador + policy IA
│   │   ├── unitree_mujoco.py       # Visor MuJoCo + LiDAR ZMQ + cámara ZMQ
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

## Flujo de trabajo

### 1 · Simulación + control de locomoción

```bash
# Terminal 1 — arranca MuJoCo + policy IA (también lanza unitree_mujoco.py internamente)
cd mujoco/simulacion
python3 run_sim_ai_g1.py
```

### 2 · Bridge LiDAR → ROS 2

```bash
# Terminal 2 — publica /scan, /odom, /tf
cd mujoco/simulacion
python3 mujoco_ros2_lidar_bridge.py
```

### 3A · Teleoperación manual

```bash
# Terminal 3 — cliente visual con WASD
cd teleop
python3 g1_client_mujoco.py
```

```bash
# RViz2
rviz2 -d rviz2/g1_teleop.rviz
```

### 3B · Mapeo (SLAM casero)

```bash
# Terminal 3 — mapper de ocupación
cd mujoco/simulacion
python3 mujoco_slam_mapper.py
# Pulsa 'm' para guardar el mapa en maps/
```

```bash
rviz2 -d rviz2/g1_mapping.rviz
```

### 3C · Navegación autónoma con Nav2

```bash
# Terminal 3 — bridge /cmd_vel → TCP:6000
cd navegacion
python3 cmd_vel_bridge.py

# Terminal 4 — stack Nav2 (usa el mapa más reciente de maps/ por defecto)
cd navegacion/nav2
ros2 launch g1_nav2.launch.py
# O con mapa explícito:
ros2 launch g1_nav2.launch.py map:=/ruta/absoluta/al/mapa.yaml
```

```bash
rviz2 -d rviz2/navigation.rviz
```

### 3D · Navegación autónoma con EasyNavigation (Docker, Jazzy)

```bash
# Terminal 3 — bridge /cmd_vel → TCP:6000 (misma ventana que Nav2, no usar ambos)
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

## Puertos y protocolos

| Puerto | Protocolo | Dirección | Descripción |
|---|---|---|---|
| `6000` | TCP | → `run_sim_ai_g1.py` | Comandos de locomoción (`w/s/a/d/q/e/stop`) |
| `5555` | ZMQ PUB | ← `unitree_mujoco.py` | Stream de vídeo RealSense |
| `5556` | ZMQ PUB | ← `unitree_mujoco.py` | Stream LiDAR (puntos + poses) |
| `6005` | UDP | → `unitree_mujoco.py` | Reset de posición del robot |
| `9876` | UDP | → `run_sim_ai_g1.py` | Targets de brazos externos |
