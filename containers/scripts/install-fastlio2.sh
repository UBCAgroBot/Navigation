    #!/usr/bin/env bash
    set -euox pipefail
    echo "Installing dependencies for FAST_LIO2 / livox_ros_driver2 / Livox-SDK2"
    export DEBIAN_FRONTEND=noninteractive

    # Install a compatible CMake version
    rm -f /usr/local/bin/cmake /usr/local/bin/ctest /usr/local/bin/cpack # remove old CMake environment to prevent conflicts

    apt-get update
    apt-get install -y \
        cmake \
        libpcl-dev \
        libeigen3-dev \
        ros-humble-pcl-ros \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        python3-rosdep \
        python3-dev

    cmake --version

    # Source ROS 2
    if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
        set +u
        source /opt/ros/humble/setup.bash
        set -u
    else
        echo "ERROR: /opt/ros/humble/setup.bash not found (ROS2 Humble not installed?)"
        exit 1
    fi

    # Workspace
    cd "$HOME"
    mkdir -p fastlio2-build/src
    cd fastlio2-build/src

    # Livox-SDK2
    if [ ! -d Livox-SDK2 ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    fi
    echo "Building Livox-SDK2"
    cd Livox-SDK2
    cmake --version
    
    # FORCE CMAKE VERSION COMPATIBILITY
    find . -name "CMakeLists.txt" -exec sed -i -E 's/VERSION ([0-2]\.[0-9]+|3\.[0-4])/VERSION 3.5/g' {} + # Find all CMakeLists.txt files and update any version below 3.5 to 3.5
    mkdir -p build
    cd build
    cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    make VERBOSE=1 -j$(nproc)
    make install

    # livox_ros_driver2
    cd "$HOME/fastlio2-build/src"
    if [ ! -d livox_ros_driver2 ]; then
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    fi

    cd livox_ros_driver2

    # Manually prepare the ROS 2 package manifest
    if [ -f "package_ROS2.xml" ]; then
        echo "Preparing package.xml for ROS 2..."
        cp package_ROS2.xml package.xml
    else
        echo "ERROR: package_ROS2.xml not found! Check repo structure."
        exit 1
    fi

    # Patch the CMakeLists.txt to ignore the NOTFOUND variable 
    sed -i '/LIVOX_INTERFACES_INCLUDE_DIRECTORIES/s/^/#/' CMakeLists.txt # Clear build errors related to missing Livox interfaces

    echo "Building livox_ros_driver2"

    # Direct colcon build command
    colcon build \
        --packages-select livox_ros_driver2 \
        --symlink-install \
        --cmake-args \
            -DROS_EDITION=ROS2 \
            -DHUMBLE_ROS=humble \
            -DCMAKE_BUILD_TYPE=Release \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_EXECUTABLE=/usr/bin/python3

    # FAST_LIO_ROS2
    cd "$HOME/fastlio2-build/src"
    if [ ! -d FAST_LIO_ROS2 ]; then
        git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
    fi

    cd "$HOME/fastlio2-build"

    rosdep init || true
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y

    colcon build \
        --parallel-workers $(nproc) \
        --symlink-install \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    echo "FASTLIO done"
