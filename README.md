# Rbot

Rakuten Robotics Base Toolkit (RBot) is a collection of packages aimed at
improving productivity, increasing code reuse and promoting best-practises.

The code is mostly C++14 header only, except
* where it needs to be a shared library (such as Gazebo plugins)
* where C++17 functionality is needed

## Introduction
RBot aims at
* Filling holes in existing libraries
* Provinding a more uniform interface
    * Simple conversion from Node <-> Nodelet
    * Simulated sensors have similar inputs
    * `std::chrono` and `ros::Time`
* Reducing copy-paste code by providing convenient functions
    * `<algorithm>` style functions for messages
    * Easy assignment from Gazebo <-> ROS <-> Ignition <-> Others
* Encapsulate "best practises" without forcing a framework
    * Most classes and functions are stand-alone
    * Provides nodelets and nodes for simple OpenCV operations
* Provide an easy to use "framework" if needed afterall
    * WIP: CMake to go along with aforementioned classes

RBot is a mono-repo with 4 major components:
* **Messages**: Schema of robot-agnostic messages for allowing higher
flexibility in nodes
* **Utilities**: Algorithms and structures needed to quickly create
maintainable MVP (Minimum Viable Product) nodes/nodelets
* **Computer Vision Pipeline**: Basic building blocks for creating a fast and
responsive compute elements to reduce load, latency and increase throughput
while increasing ease of debugging vision pipelines
* **Simulator Integration**: Allows better integration between ROS and Gazebo
for testing code in simulation. Better interface allows larger code sharing
between various components for parsing XML input and reducing errors due to
spelling differences

## License

All code is licensed as MPL2. Please note that the license is not reproduced
in each file or each package for brewity. For the complete license, please
look at [LICENSE]
