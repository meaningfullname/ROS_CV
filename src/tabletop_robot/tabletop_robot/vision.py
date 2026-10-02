import cv2
import numpy as np


COLOR_RANGES = {
    'red': [((0, 110, 70), (10, 255, 255)), ((170, 110, 70), (179, 255, 255))],
    'green': [((40, 100, 60), (85, 255, 255))],
    'blue': [((95, 110, 60), (135, 255, 255))],
}


def detect_objects(rgb, depth, focal_length):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    height, width = depth.shape
    objects = []
    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros((height, width), np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        for index in range(1, count):
            if stats[index, cv2.CC_STAT_AREA] < 20:
                continue
            u, v = centers[index]
            distance = float(np.median(depth[labels == index]))
            x = 0.45 + (u - (width - 1) / 2) * distance / focal_length
            y = -(v - (height - 1) / 2) * distance / focal_length
            z = 1.2 - distance
            if 0.30 < x < 0.65 and abs(y) < 0.30 and 0.015 < z < 0.09:
                objects.append((color, (x, y, 0.025)))
    return objects
