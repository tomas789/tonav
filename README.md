# Tonav

[![Travis-CI Tonav][2]][1]

  [1]: https://travis-ci.org/tomas789/tonav/branches
  [2]: https://travis-ci.org/tomas789/tonav.svg?branch=master (Travis-CI Tonav)

**NEW**: Check out [my wiki page](https://github.com/tomas789/tonav/wiki). I'll add some additional resources there.

Implementation of Multi-State Constraint Kalman Filter (MSCKF) for Vision-aided Inertial Navigation. This is my master's thesis.

## Status

This is work-in-progress. It is not able to work out-of-the-box currently. Expected time when it will be able to work out-of-the-box: Jan 2017.

## Goals

As a goal of this work I want to create complete navigation stack without using global position such as GPS. For local navigation it uses Multi-State Constraint Kalman Filter which is at the time of writing state-of-the-art method. It also has a great computation power to accuracy ratio. Drawback of this approach is that still accumulates (relatively small) drift during time. To compensate for this I want to use mechanism that uses loop closures. It will be based on principles used in ORB-SLAM. I have quite a bit experience with it and it works great.

By combining these two approaches I want to create navigation stack that will be able to perform life-long navigation using very cheap hardware and with low energy demands. It should be able to run on battery. It should also be able to run on CPU only.

Goal list:
 - Accurate navigation
 - Low-cost hardware
 - Life-long navigation
 - Low-energy demand (battery)
 - Global drift compensation (loop closure)

## Datasets

For development purpose, I use [MIT Stata Center Data Set](http://projects.csail.mit.edu/stata/index.php). It contains rosbag files recorded from PR2 robot. 

Each bag file is quite large because it contains laser scans. They are not needed for the purpose of this work, so I created a filtered version of them using command 

`$ rosbag filter 2011-01-18-06-37-58.bag pr2.bag 'topic in ("/wide_stereo/left/image_rect", "/wide_stereo/left/camera_info", "/torso_lift_imu/data", "/tf", "/robot_pose_ekf/odom_combined")'`

Another dataset I use is [The KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/). It has accurate IMU sensor and data are provided at relatively low frequency which simplyfies development a lot. Because I was not able to find good tool for replaying this dataset in ROS, I've created one. It is called [kitti2bag](https://github.com/tomas789/kitti2bag). It is written in Python and it is very simple to use. It converts raw dataset files into ROS bag files which you can play any time at any speed. 

By the way. Can you believe how hard it is to find publically available bagfile that is recorded using some cheap hardware? C'mon!

## Live data from iOS

For development purpose I've created data streaming app that streams IMU events (and video in the future) form iDevice (iPad, iPhone, ...) to RabbitMQ and then another app that picks them up from RabbitMQ and publishes them as ROS topic. Those apps are called [iOSmsg](https://github.com/tomas789/iOSmsg) and [iOSmsg_client](https://github.com/tomas789/iOSmsg_client). You can use them for any purpose you want. And if you like them, remember to give me a start, fork it and tell your friends ;)

## Installation

To install this you need to have installed and working ROS. Then it should be fairly easy to build and run.

```
git clone https://github.com/tomas789/tonav.git
cd tonav
mkdir build
cd build
cmake ..
make
```

## Run

I currently don't provide any roslaunch file. Just run `roscore` and then run Tonav

```
./tonav --image <image_topic> --camerainfo <camerainfo_topic> --imu <imu_topic>
```

For [MIT Stata Center Data Set](http://projects.csail.mit.edu/stata/index.php) I run it using

```
./tonav --image /wide_stereo/left/image_rect --camerainfo /wide_stereo/left/camera_info --imu /torso_lift_imu/data
```

## Documentation

At the time of writing there is no good documentation. Actually the best one is this readme. You can also find some useful information in my in-source Doxygen documentation. If you have installed Doxygen in version at least 1.8.8 you can generate it. Just run `make doc` and it will be generated in the folder `build/doc`.

## Bug reporting and support

This is something as alpha-dev-buggy piece of work. But stay tuned. I do my best. If you want to report a bug or if you want to know something about it just contact me at tomas789@gmail.com or simply use [Issue tracker of GitHub](https://github.com/tomas789/tonav/issues).

## What does Tonav mean?

Its Tom's Navigation.

## License

This work is currently distributed under LGPL v3 license. In the future, it will switch to GPL.
