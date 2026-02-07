Steps:
  1. Open docker with this command (mounts workspace as well)
  ```
  sudo docker run -it --rm \
      --net=host \
      --privileged \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /dev:/dev \
      --mount type=bind,src=$(pwd),target=/workspace \
      -w /workspace \
      ros-fastlio:jetson
  ```
2. Run ```sudo chown -R $(id -u):$(id -g) .``` (gives root permissions)
3. Enter top-lvl directory and run ```colcon build```
