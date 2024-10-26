The ROS 1 that forms the core of the system of the bot is supported under ubuntu 20.04 LTS as native operating system. The version of CUDA supported by Ubuntu 20.04 LTS is 11.8, but the default Nvidia driver of the latest kernel of 20.04 LTS is NVIDIA:535 and it requires a CUDA 12.2. 

> So we opt the method of downgrading instllation in which we use the compiler drivers of 11.8 but yet the CUDA 12.2 will be used for Graphics Rendering.

Here are the steps for the Downgrading Installation:


> 1) Downloading the CUDA run file.[Download](https://developer.nvidia.com/cuda-11-8-0-download-archive)<br>
> 2) Choose Operating Sytem -> LINUX ; Architecture -> X86_64 ; Version -> 20.04 ; Installer Type -> runfile(local)
> 3) After downloading the run file run the following command to start the installer
     ```
  sudo sh cuda_11.8.0_520.61.05_linux.run
    ```
> 4) In the installer menu choose the following options

  
