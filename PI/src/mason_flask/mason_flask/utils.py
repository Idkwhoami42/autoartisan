import numpy as np
from numpy.typing import NDArray
import cv2
from scipy.ndimage import maximum_filter, minimum_filter
import time
from collections import deque


def add_red_tint(color):
    return [color[0], color[1], 200]


def add_green_tint(color):
    return [color[0], 200, color[2]]


def normalize_depth_frame(depth_frame: NDArray[np.uint16]) -> NDArray[np.uint16]:
    height, width = depth_frame.shape
    for x in range(width):
        for y in range(height):
            if depth_frame[y, x] == 0:
                # get sum of non zero neighbors
                sum_neighbors = 0
                count = 0
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        if i == 0 and j == 0:
                            continue
                        if (
                            x + i >= 0
                            and x + i < width
                            and y + j >= 0
                            and y + j < height
                        ):
                            if depth_frame[y + j, x + i] != 0:
                                sum_neighbors += depth_frame[y + j, x + i]
                                count += 1
                if count > 0:
                    depth_frame[y, x] = sum_neighbors // count
    return depth_frame


def preprocess_depth_frame(depth_frame: NDArray[np.uint32]) -> NDArray[np.uint32]:

    height, width = depth_frame.shape
    kernel_size = 2
    new_height = height // kernel_size
    new_width = width // kernel_size

    reshaped = depth_frame.reshape(new_height, kernel_size, new_width, kernel_size)
    subsampled_image = reshaped.max(axis=(1, 3))

    return subsampled_image


def preprocess_color_frame(color_frame: NDArray[np.uint16]) -> NDArray[np.uint16]:
    height, width, channels = color_frame.shape
    kernel_size = 2
    new_height = height // kernel_size
    new_width = width // kernel_size

    reshaped = color_frame.reshape(
        new_height, kernel_size, new_width, kernel_size, channels
    )
    subsampled_image = reshaped.mean(axis=(1, 3)).astype(np.uint8)

    return subsampled_image


def process_frames(
    depth_frame: NDArray[np.uint16], color_frame: NDArray[np.uint16]
) -> None:

    # startTime = time.perf_counter()

    INVALID_DEPTH_BAND = 72
    depth_frame = preprocess_depth_frame(
        depth_frame[36:, INVALID_DEPTH_BAND:].astype(np.uint16)
    )
    color_frame = preprocess_color_frame(color_frame[36:, INVALID_DEPTH_BAND:])

    height, width = depth_frame.shape

    depth_frame = depth_frame[
        height // 4 : height // 4 * 3, width // 4 : width // 4 * 3
    ]
    color_frame = color_frame[
        height // 4 : height // 4 * 3, width // 4 : width // 4 * 3
    ]

    window_size = 80

    height, width = depth_frame.shape

    # max_depth_arr = maximum_filter(depth_frame, size=window_size)
    # min_depth_arr = minimum_filter(np.where(depth_frame == 0, 1000, depth_frame), size=window_size)

    # min_depth_arr, max_depth_arr = getMinMax(depth_frame, window_size)
    # print(depth_frame.dtype)

    # count of 0s in the depth frame
    zero_count = np.count_nonzero(depth_frame == 0)
    # print("Zero count: ", zero_count)

    min_depth_arr = cv2.erode(
        np.where(depth_frame == 0, 255, depth_frame),
        cv2.getStructuringElement(cv2.MORPH_RECT, (80, 80)),
    )
    max_depth_arr = cv2.dilate(
        depth_frame, cv2.getStructuringElement(cv2.MORPH_RECT, (80, 80))
    )

    # print(min_depth_arr.shape)
    # print(max_depth_arr.shape)

    for x in range(width):
        for y in range(height):

            # get the max and min value around 40x40 area

            # top = max(0, y - window_size // 2)
            # bottom = min(height, y + window_size // 2)
            # left = max(0, x - window_size // 2)
            # right = min(width, x + window_size // 2)

            # # Extract the valid depth window
            # depth_window = depth_frame[top:bottom, left:right]

            # max_depth = int(np.max(depth_window))
            # min_depth = int(np.min(depth_window[depth_window != 0]))
            if depth_frame[y, x] == 0 or max_depth_arr[y, x] - min_depth_arr[y, x] <= 7:
                color_frame[y, x] = [*add_green_tint(color_frame[y, x])]
                continue

            ratio = (depth_frame[y, x] - min_depth_arr[y, x]) / (
                max_depth_arr[y, x] - min_depth_arr[y, x]
            )
            if ratio > 0.4:
                color_frame[y, x] = [*add_red_tint(color_frame[y, x])]
            else:
                color_frame[y, x] = [*add_green_tint(color_frame[y, x])]

    # final_color_frames.append((color_frame, frame_no))

    # endTime = time.perf_counter()

    # print(f"Processed frame {frame_no} in {endTime - startTime} seconds")

    return color_frame
