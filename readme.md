# HARMONIC Playback

The HARMONIC playback package is designed to visualize the data available in the [HARMONIC](http://harp.ri.cmu.edu/harmonic) dataset. The latest version is 0.1.0, which corresponds to HARMONIC dataset version 0.2.0.

## Build

The project depends on ROS; specific dependencies are declared in the `package.xml` file. All packages should be standard, with the exception of two:

* `or_urdf`: available at https://github.com/personalrobotics/or_urdf
* `or_rviz`: available at https://github.com/personalrobotics/or_rviz
* `ros_myo`: available at https://github.com/uts-magic-lab/ros_myo

The package currently depends on `openrave`; it has been tested against version 0.9.

With the dependencies installed, it can be built as a standard ROS package. It has been tested on Ubuntu 14.04 / ROS Indigo and Ubuntu 16.04 / ROS Kinetic.

## Usage

First, the data to be visualized must be compiled into a visualization package. This performs the following steps:

* Creates a `viz_data.bag` file with messages to be played back in ROS, currently including joystick, robot position, and Myo data
* Downsamples the appropriate videos (currently `gaze_overlay`, `eye0`, `eye1`, and `zed_left`) to a specified frame rate for ease of playback

To run this process, call `rosrun harmonic_playback build_viz_bag /path/to/data/dir`. (A data directory is a directory that contains files named `text_data`, `stats`, etc., corresponding to a single run.) If you download the complete HARMONIC dataset, these files have been pre-generated. If you pass in a higher-level directory, it will process all data directories that fall below (though it currently has no capacity for parallel processing).

To visualize the data, run `roslaunch harmonic_playback visualizer.launch data_directory:=/path/to/data/dir`. Once the data loads, press spacebar to start playback. Several additional options are also available.


