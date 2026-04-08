#!/usr/bin/env python3
import functions
import time

# Functions 객체 생성
fuc = functions.Functions()
fuc.setup()
fuc.start()

print("거리 유지 모드 시작...")
fuc.keepDistance()

# 10초 동안 실행
time.sleep(20)

print("정지")
fuc.pause()