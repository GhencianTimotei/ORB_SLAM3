
## Prerequisite

Please make sure you have the following already installed before running build.sh

```
sudo apt-get install -y \
  git cmake build-essential pkg-config \
  libeigen3-dev libopencv-dev \
  libboost-serialization-dev libssl-dev \
  libgl1-mesa-dev libglew-dev libpython3-dev \
  libepoxy-dev  ros-jazzy-cv-bridge

git clone --depth 1 --branch v0.9.1 https://github.com/stevenlovegrove/Pangolin.git "$HOME/Pangolin"
cmake -S "$HOME/Pangolin" -B "$HOME/Pangolin/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/Pangolin/build" -j4
sudo cmake --install "$HOME/Pangolin/build"
sudo ldconfig
