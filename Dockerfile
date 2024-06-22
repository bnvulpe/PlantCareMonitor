# build ros image
FROM ros:humble

# install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rosbag2 \
    python3-setuptools \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# configure environment
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"
RUN mkdir -p /ros_ws/src

# Copia el código fuente a la imagen
COPY . /ros_ws/src/plant_monitor

# on terminal 
# 1. docker build -t plant_monitor .       
# 2. docker run -it --rm plant_monitor bash
# 3. cd /ros_ws && colcon build
# 4. source /ros_ws/install/local_setup.bash
# 5. ros2 launch plant_monitor plant_monitor.launch.py
# ctrl c in bash to stop the program and exit to exit the container

CMD ["/bin/bash"]