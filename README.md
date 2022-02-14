# AI Robotics pwd by Turtlebot3 + nVidia Jetson + OpenManipulator
 
[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

Autonomous robots, computer vision and AI controlled applications are starting to have higher and higher influence in our everyday life. It is the same case for precision agriculture, where the help of these robots could conduct a more efficient but also greener way of food production. 

This project is supposed is supposed to be my Bachelor's License (BSc) project in Computer Science & Automation at the Technical University of Cluj-Napoca, where I intend to ... 


1. [The specification](spec.md) for how a standard README should look.
2. A link to [a linter](https://github.com/RichardLitt/standard-readme-preset) you can use to keep your README maintained ([work in progress](https://github.com/RichardLitt/standard-readme/issues/5)).
3. A link to [a generator](https://github.com/RichardLitt/generator-standard-readme) you can use to create standard READMEs.
4. [A badge](#badge) to point to this spec.
5. [Examples of standard READMEs](example-readmes/) - such as this file you are reading.

Standard Readme is designed for open source libraries. Although it’s [historically](#background) made for Node and npm projects, it also applies to libraries in other languages and package managers.


## Table of Contents

- [Background](#background)
- [Prerequisites](#prerequisites)
- [Install](#install)
- [Usage](#usage)
	- [Generator](#generator)
- [Badge](#badge)
- [Example Readmes](#example-readmes)
- [Related Efforts](#related-efforts)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Background

Standard Readme started with the issue originally posed by [@maxogden](https://github.com/maxogden) over at [feross/standard](https://github.com/feross/standard) in [this issue](https://github.com/feross/standard/issues/141), about whether or not a tool to standardize readmes would be useful. A lot of that discussion ended up in [zcei's standard-readme](https://github.com/zcei/standard-readme/issues/1) repository. While working on maintaining the [IPFS](https://github.com/ipfs) repositories, I needed a way to standardize Readmes across that organization. This specification started as a result of that.

> Your documentation is complete when someone can use your module without ever
having to look at its code. This is very important. This makes it possible for
you to separate your module's documented interface from its internal
implementation (guts). This is good because it means that you are free to
change the module's internals as long as the interface remains the same.

> Remember: the documentation, not the code, defines what a module does.

~ [Ken Williams, Perl Hackers](http://mathforum.org/ken/perl_modules.html#document)

Writing READMEs is way too hard, and keeping them maintained is difficult. By offloading this process - making writing easier, making editing easier, making it clear whether or not an edit is up to spec or not - you can spend less time worrying about whether or not your initial documentation is good, and spend more time writing and using code.

By having a standard, users can spend less time searching for the information they want. They can also build tools to gather search terms from descriptions, to automatically run example code, to check licensing, and so on.

The goals for this repository are:

1. A well defined **specification**. This can be found in the [Spec document](spec.md). It is a constant work in progress; please open issues to discuss changes.
2. **An example README**. This Readme is fully standard-readme compliant, and there are more examples in the `example-readmes` folder.
3. A **linter** that can be used to look at errors in a given Readme. Please refer to the [tracking issue](https://github.com/RichardLitt/standard-readme/issues/5).
4. A **generator** that can be used to quickly scaffold out new READMEs. See [generator-standard-readme](https://github.com/RichardLitt/generator-standard-readme).
5. A **compliant badge** for users. See [the badge](#badge).

## Prerequisites

The project is based on the [Robot Operaing System's](https://www.ros.org/)  [Melodic Distro](http://wiki.ros.org/melodic). One should start with the installation steps provided by the ROS platform. The recommended OS is [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) for the Master PC and [JetPack 4.6] for the Jetson Nano since it has been tested only in this configuration, but it should be working fine on other Linux-based systems too.

On the Hardware side, I was working with a [Jetson Nano Dev Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) used for controlling the robotic platform, which is a [Turtlebot3 Waffle](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) completed by the [OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/) robotic arm. In addition, I use an [Intel Realsense D435i Depth Camera](https://www.intelrealsense.com/depth-camera-d435i/) for customized object detection and getting the object's coordinates in 3D space.




## Install

### D435i Depth Camera

For using the camera one should install the realsense2 SDK's [ROS wrapper](https://github.com/IntelRealSense/realsense-ros) by typing:

```sh
$ sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

Unfortunately, in case of the ARM arhictectures it does not work, so one has to build both the ROS package from source:

```sh
THIS PART HAS TO BE UPDATED...
```

Moreover, the [Python wrapper](https://pypi.org/project/pyrealsense2/) for the Intel RealSense SDK2.0 is also needed, which provides C++ to Python binding (maybe at some point I am going to use C++ instead of Python)

```sh
$ pip install pyrealsense2
```

In case of the SDK's Python wrapper the situation is the same (if it has not been resolved in the meantime) it does not work with the ARM processors, therefore we need to build it from source:

```sh
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense
$ mkdir build
$ cd build
$ cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DBUILD_WITH_CUDA:bool=true
$ make -j4
$ sudo make install

$ cd ~/librealsense
$ ./scripts/setup_udev_rules.sh

export PATH=$PATH:~/.local/bin
export PYTHONPATH=$PYTHONPATH:/usr/local/lib
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2
```



## Usage

This is only a documentation package. You can print out [spec.md](spec.md) to your console:

```sh
$ standard-readme-spec
# Prints out the standard-readme spec
```

### Generator

To use the generator, look at [generator-standard-readme](https://github.com/RichardLitt/generator-standard-readme). There is a global executable to run the generator in that package, aliased as `standard-readme`.

## Badge

If your README is compliant with Standard-Readme and you're on GitHub, it would be great if you could add the badge. This allows people to link back to this Spec, and helps adoption of the README. The badge is **not required**.

[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

To add in Markdown format, use this code:

```
[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)
```

## Example Readmes

To see how the specification has been applied, see the [example-readmes](example-readmes/).

## Related Efforts

- [Art of Readme](https://github.com/noffle/art-of-readme) - 💌 Learn the art of writing quality READMEs.
- [open-source-template](https://github.com/davidbgk/open-source-template/) - A README template to encourage open-source contributions.

## Maintainers



## Contributing

Feel free to dive in! [Open an issue](https://github.com/RichardLitt/standard-readme/issues/new) or submit PRs.

Standard Readme follows the [Contributor Covenant](http://contributor-covenant.org/version/1/3/0/) Code of Conduct.

### Contributors



## License

[MIT](LICENSE) © Richard Littauer
