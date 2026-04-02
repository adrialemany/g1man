FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 1. Instalación de paquetes del sistema y dependencias de ROS 2
RUN apt-get update && apt-get install -y \
    # Herramientas base y compilación
    git build-essential cmake wget curl nano iproute2 net-tools terminator tar \
    python3-pip python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool \
    # Herramientas de Audio, Vídeo y Gráficos
    alsa-utils ffmpeg v4l-utils freeglut3-dev libglfw3-dev libglu1-mesa-dev \
    # Librerías matemáticas y Python
    python3-opencv python3-numpy \
    # Dependencias base de ROS 2
    ros-humble-rviz2 \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-generator-dds-idl \
    # Paquetes ROS 2 de Visión y Cámaras
    ros-humble-v4l2-camera \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-vision-msgs \
    # Almacenamiento y Rosbags
    ros-humble-rosbag2-storage-mcap \
    # Middleware y Red DDS
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp \
    # Navegación, Planificación y Diagnóstico
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-moveit \
    ros-humble-diagnostic-updater \
    ros-humble-pinocchio \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# 2. Inicializar rosdep
RUN rosdep init || true \
    && rosdep update

# 3. Instalación de paquetes de Python adicionales (Pip)
# Instalamos la librería de Unitree directamente desde GitHub como tenías en tu entorno
RUN pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git@ab0d8ae2fee8e3ec337ba941d5e7fd49de5fd30a

# 4. Instalación de Livox-SDK2 (Dependencia C++ a nivel de sistema)
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/Livox-SDK2

# 5. Configuración del Workspace de ROS 2
WORKDIR /root/ros2_ws/src

# Solución al problema de Git "dubious ownership"
RUN git config --global --add safe.directory '*'

# Clonamos todos los repositorios en la carpeta src
RUN git clone https://github.com/CDonosoK/astroviz_interfaces.git \
    && git clone https://github.com/RobInLabUJI/g1pilot.git \
    && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
    && git clone https://github.com/unitreerobotics/unitree_ros2.git \
    && git clone https://github.com/hku-mars/FAST_LIO_ROS2.git

# 6. Compilación del Workspace
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble"

# 7. Configuración del entorno
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]
