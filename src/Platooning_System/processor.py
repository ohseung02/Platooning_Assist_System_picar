#!/usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from line import Line

class ImageProcessor():
    """
        Implements the computer vision algorithms for detecting lanes in an image.
        (Canny + ROI + Hough + 선 필터링 강화 버전)
    """

    def __init__(self, frameDimensions, frameRate):
        # Define camera dimensions
        self.frameDimensions = frameDimensions
        self.frameRate = frameRate
        self.w = self.frameDimensions[0]
        self.h = self.frameDimensions[1]

        # ROI dimensions in percentage
        self.roiY = (0.55, 1.0)   # 하단 절반만 사용
        self.roiX = (0.25, 0.85)    # 좌우 10%는 잘라냄

        # Initialize lane containers
        self.left = Line(self.frameDimensions, (0, 0, 255))
        self.right = Line(self.frameDimensions, (255, 0, 0))
        
        # Camera calibration (optional, 그대로 유지)
        self.calibrationResolution = (1280, 720)
        kx = self.w / self.calibrationResolution[0]
        ky = self.h / self.calibrationResolution[1]
        cameraMatrix = np.array([
            [1.00612323e+03, 0.0, 6.31540281e+02],
            [0.0, 1.00551440e+03, 3.48207362e+02],
            [0.0, 0.0, 1.0]
        ])
        self.cameraMatrix = np.multiply(cameraMatrix, [
            [kx, 1, kx],
            [1, ky, ky],
            [1, 1, 1]
        ])
        self.distortionCoefficients = np.array([[0.18541226, -0.32660915, 0.00088513, -0.00038131, -0.02052374]])
        self.newCameraMatrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.cameraMatrix,
            self.distortionCoefficients,
            self.frameDimensions,
            1,
            self.frameDimensions
        )
        self.rectifyMapX, self.rectifyMapY = cv2.initUndistortRectifyMap(
            self.cameraMatrix,
            self.distortionCoefficients,
            None,
            self.newCameraMatrix,
            self.frameDimensions,
            5
        )

    def doBlur(self, frame, iterations, kernelSize):
        """ Performs a gaussian blur with the set number of iterations """
        blured = frame.copy()
        while iterations > 0:
            blured = cv2.GaussianBlur(blured, (kernelSize, kernelSize), sigmaX=0, sigmaY=0)
            iterations -= 1
        return blured

    def doRegionOfInterest(self, frame):
        """
            Applies trapezoidal ROI mask to keep only lane area
        """
        y0Px = int(self.h * self.roiY[0])
        y1Px = int(self.h * self.roiY[1])
        x0Px = int(self.w * self.roiX[0])
        x1Px = int(self.w * self.roiX[1])

        vertices = np.array([[
            (x0Px, y0Px),
            (x1Px, y0Px),
            (self.w, y1Px),
            (0, y1Px)
        ]], dtype=np.int32)

        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(frame, mask)

    def findLanes(self, frame, lines, minAngle=10, drawAll=False):
        """
            Filter and classify Hough lines into left and right lanes
        """
        self.left.clear()
        self.right.clear()

        if lines is not None:
            cx = self.w // 2
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x1 == x2:   # vertical line skip
                        continue
                    slope = (y2 - y1) / (x2 - x1)
                    length = np.hypot(x2 - x1, y2 - y1)

                    # 필터링: 너무 짧은 선 제거
                    if length < 40:
                        continue
                    # 필터링: 수평선 제거
                    if abs(slope) < 0.3:
                        continue

                    # 왼쪽 차선
                    if slope < 0 and x1 < cx and x2 < cx:
                        self.left.add(x1, y1, x2, y2)
                        if drawAll:
                            cv2.line(frame, (x1, y1), (x2, y2), self.left.color, 2)
                    # 오른쪽 차선
                    elif slope > 0 and x1 > cx and x2 > cx:
                        self.right.add(x1, y1, x2, y2)
                        if drawAll:
                            cv2.line(frame, (x1, y1), (x2, y2), self.right.color, 2)

        self.left.fit()
        self.right.fit()
        return frame
        
    def drawPoly(self, frame, poly, color, width=3):
        """
            Draws fitted line polynomial on frame
        """
        y0 = int(self.h * self.roiY[0])
        y1 = int(self.h * self.roiY[1])
        if poly is not None:
            x0 = int(poly(y0))
            x1 = int(poly(y1))
            cv2.line(frame, (x0, y0), (x1, y1), color, width)

    def process(self, frame):
        """
            Main lane detection pipeline
        """
        undistort = cv2.remap(frame, self.rectifyMapX, self.rectifyMapY, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(undistort, cv2.COLOR_BGR2GRAY)
        blured = self.doBlur(gray, iterations=2, kernelSize=5)
        edges = cv2.Canny(blured, 50, 150)
        roi = self.doRegionOfInterest(edges)

        lines = cv2.HoughLinesP(
            roi,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=40,
            maxLineGap=80
        )

        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.findLanes(vis, lines, minAngle=10, drawAll=True)
        return vis
