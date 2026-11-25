FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV QT_QPA_PLATFORM=xcb

WORKDIR /app

# Instalace systémových a Python knihoven
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-zmq \
    python3-psycopg2 \
    python3-h5py \
    python3-pandas \
    libgl1 \
    libegl1 \
    libdbus-1-3 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-xkb1 \
    polkitd \
    pkexec \
    && rm -rf /var/lib/apt/lists/*

# Virtuální prostředí s přístupem k systémovým ROS2 knihovnám
RUN /usr/bin/python3 -m venv --system-site-packages /opt/diplomka-venv

# Instalace PySide6
RUN /opt/diplomka-venv/bin/python -m pip install --no-cache-dir PySide6

# Kopírování ROS2 balíčků
COPY src/ /app/src/

# EtherCAT driver nebude součástí Docker image
RUN rm -f \
    /app/src/measurement_system/measurement_system/drivers/leika_driver/leica_connect.py

# Sestavení ROS2 projektu
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Spouštění ROS2 aplikace
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash && exec /opt/diplomka-venv/bin/python /app/install/measurement_system/lib/measurement_system/measurement_ui"]