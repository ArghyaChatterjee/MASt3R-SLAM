<p align="center">
  <h1 align="center">MASt3R-SLAM-ROS2</h1>
  <p align="center">
  <a href="https://www.youtube.com/watch?v=iqkj2m7uGEQ" target="_blank">
    <img src="./media/ihmc_onr_demo.gif" alt="Video Thumbnail" width="400">
  </a>
  <a href="https://www.youtube.com/watch?v=7QrIU78EMaM" target="_blank">
    <img src="./media/ihmc_zedmini_demo.gif" alt="Video Thumbnail" width="400">
  </a>
  </p>
</p>

## Requirements
This experiment was performed on Ubuntu 22.04 with cuda 12.1 and python 3.10 and ROS2 humble.

## Setup

Clone the repo and install the dependencies.
```
git clone https://github.com/ArghyaChatterjee/MASt3R-SLAM-ROS2.git
cd MASt3R-SLAM-ROS2/
```
Export the `libstdc++` dll before running the demo:
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
```
Create a conda environment and install required libraries with conda.
```
source ~/miniconda3/bin/activate
conda create -n mast3r-slam python=3.10
conda activate mast3r-slam
conda install numpy==1.26.4
conda install -c conda-forge spdlog=1.9.2 libtiff=4.3.0
conda install pytorch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 pytorch-cuda=12.1 -c pytorch -c nvidia
pip3 install --upgrade pip setuptools wheel
pip3 install -r requirements.txt
pip3 install --no-build-isolation -e thirdparty/mast3r
pip3 install -e thirdparty/in3d
pip3 install --no-build-isolation -e .
```

Setup the checkpoints for MASt3R and retrieval.  The license for the checkpoints and more information on the datasets used is written [here](https://github.com/naver/mast3r/blob/mast3r_sfm/CHECKPOINTS_NOTICE).
```
mkdir -p checkpoints/
wget https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth -P checkpoints/
wget https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_trainingfree.pth -P checkpoints/
wget https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_codebook.pkl -P checkpoints/
```

## Online Demo
### Input ROS topic -> Output ModernGL Viewer with ImGUI
Connect the camera and publish zed ros2 topic in the following way:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```
The zed ros2 topic it will publish:
```bash
image_topic='/zed/zed_node/left/image_rect_color'
info_topic='/zed/zed_node/left/camera_info'
```
If the calibration parameters are unknown:
```bash
python3 main.py --dataset ros2 --config config/base.yaml
```
If the calibration parameters are known, you can specify them in intrinsics.yaml:
```bash
python3 main.py --dataset ros2 --config config/base.yaml --calib config/intrinsics_zed_demo.yaml
```
This produces 9 fps SLAM rate on 4070 RTX GPU with 16 GB VRAM. 

### Input ROS topic -> Output ROS topic with Rviz
For output topic remapping as ros topics, we are working on it. Stay Tuned !!

<p align="center">
  <a href="https://www.youtube.com/watch?v=6snX6V5s2Pc" target="_blank">
    <img src="media/mast3r_slam_ros2.gif" alt="Video Thumbnail" width="80%">
  </a>
</p>





