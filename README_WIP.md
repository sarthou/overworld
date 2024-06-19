# <img src="docs/images/overworld.png" width="150">

## Installation (dev branch)

### Quick start guide on Ubuntu 22.04

Clone the dev git repository into your ros2 workspace
```
git clone -b dev https://github.com/jevans-laas/overworld-indev-engine.git
```

Go into the directory
```
cd overworld-indev-engine/
```

Make sure your system is up-to-date
```
sudo apt update
sudo apt upgrade
```

Download some system-wide dependencies
```
sudo apt install libglm-dev    # For math stuff, vectors, matrices, quaternions..
sudo apt install libassimp-dev # For loading 3D models
sudo apt install libglfw3-dev  # For creating & managing windows
sudo apt install libbullet-dev # Bullet3 physics engine
```
You could also do this all-in-one go:
```
sudo apt install libglm-dev libassimp-dev libglfw3-dev libbullet-dev
```

If you have an NVidia GPU AND plan on using the PhysX physics engine, please follow the following guide https://docs.nvidia.com/cuda/cuda-installation-guide-linux/ since it relies on the CUDA runtime, the instructions shouldn't change that much since the time of writing this, so you can just skip reading the entire thing and just do what I did:
```
# Add keyring
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

# Update the package repository
sudo apt update

# Install the CUDA toolkit
sudo apt install cuda-toolkit
```

In order to be able to build PhysX, you must also install clang tools

```
sudo apt install clang
```

Then reboot the system
```
sudo reboot
```

todo... basically just get ontologenius and run colcon build at this point