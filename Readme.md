# <img src="docs/images/overworld.png" width="150">

[![Release][Release-Image]][Release-Url]

[![Dependency Status][Ontologenius-Dependency-Image]][Ontologenius-Dependency-Url]

[![Build Status][Build-Status-Image]][Build-Status-Url]

## Installation

### Overworld

First of all, if you don't yet have the required dependencies, install them.

```
sudo apt install -y libglm-dev libglfw3-dev
```

To install Overworld, you just have to clone it and Ontologenius in the src folder of your ROS workspace and compile it. Enjoy !

```
cd src
git clone https://github.com/sarthou/ontologenius.git
git clone https://github.com/RIS-WITH/common_ground_ontology.git
git clone https://github.com/sarthou/overworld.git
cd ..
catkin_make # or catkin build
```

## Usage

Overworld has only one executable being `overworld_node` which takes only one required argument: a configuration. A lot of optional arguments are available, take a look to (Overworld's Website)[https://sarthou.github.io/overworld/overview/launchers.html].

The configuration file is used to define the used perception modules and to configure them.

Because Overworld is strongly linked to the semantic knowledge base Ontologenius, you have to launch both:

Terminal 1:
```
roslaunch overworld tuto_ontologenius.launch
```

Terminal 2:
```
roslaunch overworld tuto_overworld.launch
```

## Ontologenius

The knowledge base Ontologenius can be used as a source of information for Overworld to spawn the objects. It can provide the visual mesh, the collision mesh, or the color of an object. In addition, it is used to store the symbolic facts generated by Overworld.

More information about this link and tutorials to show its use will come soon.

[Release-Url]: https://github.com/sarthou/overworld/releases/tag/v0.2.0
[Release-Image]: http://img.shields.io/badge/release-v0.2.0-blue

[Ontologenius-Dependency-Image]: https://img.shields.io/badge/dependencies-ontologenius-yellow
[Ontologenius-Dependency-Url]: https://github.com/sarthou/ontologenius

[Build-Status-Image]: https://github.com/sarthou/overworld/actions/workflows/overworld.yml/badge.svg
[Build-Status-Url]: https://github.com/sarthou/overworld/actions
