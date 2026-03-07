Steps:

  1. Before opening docker, run on jetson terminal. this allows it to get access to display so rviz can work.
  2. '''
     xhost +local:

     '''

  3. Open docker with this command (mounts workspace as well)
  ```
  sudo docker run -it --rm \
      --net=host \
      --privileged \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /dev:/dev \
      --mount type=bind,src=/home/agrobot,target=/workspace \
      -w /workspace \
      ros-fastlio:jetson
  ```
4. Run ```sudo chown -R $(id -u):$(id -g) .``` (gives root permissions)
5. Enter top-lvl directory and run ```colcon build``` to rebuild all packages if changes have been made
6. Notes:
   - Currently, in fastlio_config, blind is set to 0.1m, so points closer than that are removed. We may want to modify this after testing

