# ZED Workspace Setup
> The following summarizes the steps involved in cloning and installing zed workspace that is essential for launching zed series cameras. This documentation has been referenced from StereoLabs zed camera setup.

Official Link to Stereolabs ZED Camera setup [ZED ROS SETUP](https://www.stereolabs.com/docs/ros)

Our Rover has a vision from ZED-2i camera.

#### The following are the services provided by the ZED-2i Camera:

<li>Left and right rectified/unrectified images</li>
<li>Depth map</li>
<li>Colored 3D point cloud</li>
<li>Visual odometry: Position and orientation of the camera</li>
<li>Pose tracking: Position and orientation of the camera fixed and fused with IMU data</li>
<li>Spatial mapping: Fused 3d point cloud</li>
<li>Sensors data: accelerometer, gyroscope, barometer, magnetometer, internal temperature sensors</li>


Dependencies
### Prerequisites

| Dependency  | Version  |
|-------------|----------|
| Ubuntu      | 20.04    |
| ZED SDK     | ≥ 3.5    |
| CUDA        | Required |
| ROS         | Noetic   |

<span style="font-size: 1.2em;">[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange)](https://releases.ubuntu.com/20.04/)</span>
<span style="font-size: 1.2em;">[![ZED SDK](https://img.shields.io/badge/ZED%20SDK-3.5+-blue)](https://www.stereolabs.com/zed/)</span>
<span style="font-size: 1.2em;">[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-green)](http://wiki.ros.org/noetic)</span>

### IMPORTANT CUDA Installation

By default after the linux kernel update the default Nvidia driver metapackage should be Nvidia Proprietory 535 if not it is required to downgrade to that version if a more later has been installed by default.

> Since the default tf(tensorflow) and CUDA version required by the Nvidia Driver is not supported by Ubuntu 20.04 we will follow a method of downgrading installation which is discussed in the following document of [CUDA]()

Once the CUDA Toolkit has been installed, we proceed with installation of ZED SDK which is inherently dependent on the CUDA version. The following link redirects to ZED SDK Official installation and one can install the required SDK for their respective CUDA version. [ZED SDK](https://www.stereolabs.com/en-in/developers/release)

After Downloading the file navigate to the folder containing the .zstd.run file.

Make sure to install zstd for extracting

```
sudo apt install zstd
```

```
cd ~/path_to_folder/file.zstd.run
chmod +x file.zstd.run
./file.zstd.run
```

After this there will be a series of installtion inshell prompts. Install the requied features or services of ZED SDK. For to use the entire potential of the SDK it is better to install all the services provided by the ZED SDK.


Now the system is ready to clone the <b>zed_workspace</b> for running the camera from ROS API.

The following are the instructions to clone and use the <b>zed_ros_workspace</b>.

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash

```

Next the following two packages are optional
<li><b>zed_ros_interfaces</b></li>
<li><b>zed_ros_examples</b></li>
















