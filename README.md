# QROS Package

This package provides a library for creating QT-based GUIs for ROS2 programs.

## Installing Package

This package depends on the QT6 library. Install QT6:

```bash
sudo apt install qt6-base-dev qt6-declarative-dev
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

TODO: clone/build interface package dependencies?? package cannot be built without them


## Configuring the package
