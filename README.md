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



[Winning Point]
- Universal Middleware: With Jetson, it can be connected to any simulator or car data

- Edge AI Optimization: With the Use of GPU, Optical Flow can be calculated in real time, which allows accurate compensation.

- Safety Layer: Not only anti motion sickness but also detecting dangerous motion to stop the car for safety.

"MotionX는 단순히 멀미를 줄이는 도구가 아닙니다. 차량의 물리적 관성(Inertia)과 승객의 시각적 경험(Visual)을 연결하는 표준 운영 체제를 지향합니다."

