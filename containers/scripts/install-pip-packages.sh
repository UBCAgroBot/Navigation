#!/usr/bin/env bash
set -euox pipefail

echo 'Installing CUDA supported python packages'

export PIP_DEFAULT_TIMEOUT=100

python3 -m pip install --upgrade pip
pip3 install --no-cache-dir --verbose --ignore-installed blinker

echo 'export CPATH=$CPATH:/usr/local/cuda-12.2/targets/aarch64-linux/include' >> ~/.bashrc
echo 'export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/cuda-12.2/targets/aarch64-linux/lib' >> ~/.bashrc
echo 'export PATH=$PATH:/usr/local/cuda-12.2/bin' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-12.2/lib64' >> ~/.bashrc
# echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo 'export PYTHONPATH=/usr/local/lib/python3.10/site-packages/:$PYTHONPATH' >> ~/.bashrc

wget http://jetson.webredirect.org/jp6/cu122/+f/8c0/114b6c62bfa3d/torchvision-0.19.0a0+48b1edf-cp310-cp310-linux_aarch64.whl#sha256=8c0114b6c62bfa3d60d08b51f1467e0ea1ee4916e5b4b1084db50c2c1f345d93

pip3 install --trusted-host jetson.webredirect.org --verbose \
    torch --index-url http://jetson.webredirect.org/jp6/cu122 \
    torchvision-0.19.0a0+48b1edf-cp310-cp310-linux_aarch64.whl

pip3 install --no-cache-dir --verbose \
    numpy \
    scipy \
    scikit-learn \
    matplotlib \
    onnx

pip3 install --trusted-host jetson.webredirect.org --verbose \
    onnxruntime-gpu --index-url http://jetson.webredirect.org/jp6/cu122 \
    cupy --index-url http://jetson.webredirect.org/jp6/cu122 \
    pycuda --index-url http://jetson.webredirect.org/jp6/cu122

pip3 install --no-cache-dir --verbose \
    ultralytics \
    flask \
    tqdm \
    argparse

apt-get update && apt-get upgrade -y
apt-get clean
rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*