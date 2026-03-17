# QROS Package

This package provides a library for creating QT-based GUIs for ROS2 programs.

## Installing Package

This package depends on the QT6 library and several QML modules. Install the following packages:

```bash
sudo apt install qt6-base-dev \
                 qt6-declarative-dev \
                 qml6-module-qtquick \
                 qml6-module-qtquick-controls \
                 qml6-module-qtquick-templates \
                 qml6-module-qtquick-window \
                 qml6-module-qtqml-workerscript
```

Install the RoShip Interfaces packages to your workspace `src/` folder:

```bash
cd ~/ros2_ws/src
git clone https://github.com/k2oceanic/roship_interfaces.git
git clone https://github.com/k2oceanic/roship_io.git
```

Next, clone this repository and install the ROS dependencies:
```bash
# Clone the repository into your workspace src/
cd ~/ros2_ws/src
git clone https://github.com/YOUR_ORG/qros.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build your workspace.


## Configuring the package


## Using QROS
