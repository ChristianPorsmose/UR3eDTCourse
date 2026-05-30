ARG BASE_IMAGE=communication-base
FROM --platform=linux/amd64 ${BASE_IMAGE}

USER root

RUN apt-get update && apt-get install -y \
    wget unzip curl \
    libxcursor1 libxinerama1 libxrandr2 libxi6 libgl1-mesa-dri libglu1-mesa \
    libxkbcommon0 libdbus-1-3 \
    xvfb x11vnc websockify novnc fluxbox procps \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/godotengine/godot/releases/download/4.6.2-stable/Godot_v4.6.2-stable_linux.x86_64.zip \
    && unzip Godot_v4.6.2-stable_linux.x86_64.zip -d /usr/local/bin/ \
    && mv /usr/local/bin/Godot_v4.6.2-stable_linux.x86_64 /usr/local/bin/godot \
    && chmod +x /usr/local/bin/godot

WORKDIR /app

RUN mkdir -p addons && \
    wget https://github.com/niklas2902/py4godot/releases/download/4.6-alpha17/py4godot.zip -O addon.zip && \
    unzip addon.zip -d addons/ && \
    rm addon.zip

# Install pure-Python deps into /app/lib — version-agnostic, works with any Python runtime
RUN pip install --target /app/lib --no-binary :all: pika pyyaml

COPY dt_solution/dt_services/GODOT_Visualization_service/ur-3e-visualization/ .

RUN PYTHONPATH=/app:/app/lib timeout 120 xvfb-run -a godot --headless --editor --quit; exit 0

EXPOSE 6080

CMD bash -c "\
    Xvfb :99 -screen 0 1920x1080x24 & \
    sleep 2; \
    export DISPLAY=:99; \
    fluxbox & \
    sleep 1; \
    x11vnc -display :99 -nopw -forever -shared -bg -rfbport 5900 -noxdamage; \
    sleep 2; \
    /usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 & \
    sleep 2; \
    PYTHONPATH=/app:/app/lib godot --audio-driver Dummy World.tscn"
