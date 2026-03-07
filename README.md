Steps:

  1. Before opening docker, run on jetson terminal. this allows it to get access to display so rviz can work.
     ```
     xhost +local:

     ```

  2. Open docker with this command (mounts workspace as well)
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
3. Run ```sudo chown -R $(id -u):$(id -g) .``` (gives root permissions)
4. Enter top-lvl directory and run ```colcon build``` to rebuild all packages if changes have been made
5. Notes:
   - Currently, in fastlio_config, blind is set to 0.1m, so points closer than that are removed. We may want to modify this after testing

