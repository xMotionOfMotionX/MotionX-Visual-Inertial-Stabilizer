import os

# 영상이 들어있는 폴더 경로
video_folder = "Jetson/Windows"

# 폴더 내 파일 목록 가져오기
files = os.listdir(video_folder)

# .mp4 파일만 골라내서 출력해보기
for file in files:
    if file.endswith(".mp4"):
        print(f"file found: {file}")