# MotionX: Visual-Inertial Anti-Motion Integrated Simulator

MotionX is an AI-powered solution designed to mitigate motion sickness and prevent dangerous collision in autonomous vehicles by synchronizing visual and inertial data between various simulators.

## 🚀 First Test Overview
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


[Simulator / Main Area / Motion Data Structure]
-F1tenth	Gym / Autonomous Racing / ROS 2 Topics (/imu, /odom)	
-MuJoCo / Contact Physics, Walking Robot / Sensor XML + mjData Struct (XML로 센서를 정의하면 Python/C++ API의 mjData 구조체로 가속도, 힘 데이터가 나옵니다.)
-Isaac Sim / NVIDIA Omniverse AI / ROS2 Bridge, Python API (NVIDIA 전용. USD 기반 데이터가 ROS 2나 Python API로 바로 꽂힙니다.)
-Donkey Sim / Beginner Autonomous Driving / Socker, JSON (유니티 기반이며, 보통 소켓 통신을 통해 JSON 형식으로 차량의 속도와 각도를 쏴줍니다.)
-CARLA / City Autonomous Driving / Python API, ROS2 Bridge (가장 방대한 센서군(IMU, GNSS 등)을 Python 객체나 ROS 토픽으로 제공합니다.) 


[Strategy]: 
입력 (Input): 각 시뮬레이터의 특성에 맞는 '통역사' 파일을 만듭니다. (carla_adapter.py, donkey_adapter.py 등)

규격화 (Standardize): 형식이 뭐든 간에 우리 미들웨어 내부에서는 똑같은 MotionX_Inertia 메시지로 변환합니다.

판단 (Core): 젯슨 오린 나노에 있는 MotionX Core는 이 데이터가 어디서 왔는지 상관하지 않고 똑같은 '안티 모션' 알고리즘과 '위험 감지' 로직을 돌립니다.
