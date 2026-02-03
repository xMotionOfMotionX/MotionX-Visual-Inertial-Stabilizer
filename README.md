# MotionX: Visual-Inertial Anti-Motion Sickness AI

MotionX is an AI-powered solution designed to mitigate motion sickness in autonomous vehicles by synchronizing visual and inertial data.

## 🚀 Overview
This project focuses on real-time visual motion compensation. By analyzing the vehicle's movement through an Intel RealSense camera, our algorithm calculates the reverse motion vector to stabilize the visual flow for passengers.

## 🛠 Tech Stack
- **Hardware**: NVIDIA Jetson Orin Nano, Intel RealSense D435i
- **Software**: Python, OpenCV, ROS 2, pyrealsense2

## 📺 Demo
[Check out our data acquisition demo on YouTube](https://youtu.be/Dv3aUmfnNwM?si=ZvAskxh2yRYUGfJl)

## 📁 Project Structure
- `main.py`: Real-time motion analysis and compensation algorithm.
- `requirements.txt`: List of dependencies.
- data/: Raw motion logs (CSV files with $dx, dy$ values).
- graphs/: Visualization graphs 
<img width="578" height="270" alt="image" src="https://github.com/user-attachments/assets/76e31050-5f63-43dd-85ad-53dcfc3de2a1" />
