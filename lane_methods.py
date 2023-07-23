import cv2
import numpy as np
import math
from lane_detection import average_slope_intercept

# Masking
def mask_img(img, show=False):  # H  S  V
    lower_thr = np.array([0, 0, 0])
    upper_thr = np.array([179, 255, 87])
    img_masked = cv2.inRange(img, lower_thr, upper_thr)
    if show:
        cv2.imshow("Masked Frame", img_masked)
    return img_masked


# Canny Edge Detection
def detect_edges(img, show=False):
    # img_bgr = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    img_gre = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(img_gre, (5, 5), 0)
    img_canny = cv2.Canny(blur, 200, 400)
    if show:
        cv2.imshow("Canny Filter", img_canny)
    return img_canny


# Cropping , Region of Interest
def crop_roi(img, show=False):
    height = img.shape[0]
    width = img.shape[1]
    # print(height, width)
    mask = np.zeros_like(img)
    cv2.rectangle(mask, (0, height // 2), (width, height), 255, -1)  # -1 -> fill
    roi = cv2.bitwise_and(img, mask)
    if show:
        cv2.imshow("mask", mask)
        cv2.imshow("roi", roi)
    return roi


# Hough Transform
def detect_lines(img, show=False):
    # function that we use is HoughLinesP
    '''
    Arguments for HoughLinesP

    rho : Distance Precision
    The hough Line Transform algorithm represents line in polar coordinates -> origin (rho) and angle (theta)
    rho specifies distance resolution in pixels.
    rho of 1 means two lines that are very close to each other but differ by a singe pixel will be considered different lines

    theta : Angular Precision
    It defines angular precision of hough transform. Means precision with which algo detects lines at different angle
    It is defined in radians
    If theta is np.Pi / 180 , that gives precision of 1 degree

    min_threshold : Minimum number of votes required for line to be considered

    lines : np.array([]) Empty array to store detected line segments

    min_line_length : Minimum length of a line to be considered

    max_line_gap : Max gap in segment to be considered as same line

    '''

    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    min_line_length = 20
    max_line_gap = 4

    lines = cv2.HoughLinesP(img, rho, theta, min_threshold, np.array([]), min_line_length, max_line_gap)

    return lines


# Draw Lines
def draw_lines(img, lines):
    line_color = (0, 255, 82)
    line_width = 2

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), line_color, line_width)
    cv2.imshow("Lines", img)


# Visually see grouping ares , [for debugging]
def lane_search_area(img, boundary = 1/2):
    height = img.shape[0]
    width = img.shape[1]
    left_lane_area_width = int(width * (1 - boundary))
    right_lane_area_width = int(width * boundary)
    # left_region = np.zeros_like(img)
    # right_region = np.zeros_like(img)

    cv2.rectangle(img, (0, 0), (left_lane_area_width, height), (0, 244, 233), 5)  # -1 -> fill
    cv2.rectangle(img, (right_lane_area_width, 0), (width, height), (0, 0, 255), 5)  # -1 -> fill

    cv2.imshow("left and right region", img)


# Idea is to group positive slop and negative slop lines , which will define left and right lane markings
def group_lines(img, lines):
    height = img.shape[0]
    width = img.shape[1]

    lane_lines = []

    # No lines found
    if lines is None:
        return lane_lines

    left_lane = []
    right_lane = []

    boundary = 1 / 3
    left_lane_area_width = width * (1 - boundary)
    right_lane_area_width = width * boundary



    for line in lines:
        for x1, y1, x2, y2 in line:
            # skip vertical lines as they have infinite slope
            if x1 == x2:
                continue

            # np.polyfit can be used to get slop and intercept from two points on the line
            coff = np.polyfit((x1, x2), (y1, y2), 1)
            slope = coff[0]
            intercept = coff[1]

            # note that y axis is inverted in matrix of images. 
            # so as x (width) increases, y(height) values decreases
            # this is reason why slope of right nane is positive and left name is negative

            # positive slop -> right lane marking  \
            #                                       \
            #                                        \
            #                                         \
            if slope > 0:
                # search area check
                if x1 > right_lane_area_width and x2 > right_lane_area_width:
                    right_lane.append((slope, intercept))


            # negative slop -> left lane marking  /
            #                                    /
            #                                   /
            #                                  /
            else:
                if x1 < left_lane_area_width and x2 < left_lane_area_width:
                    left_lane.append((slope, intercept))

    # averaging all the lines in each group to get a single line out of them
    left_avg = np.average(left_lane, axis=0)
    right_avg = np.average(right_lane, axis=0)

    # if got left lane, convert to point form from intercept form
    if len(left_lane) > 0:
        lane_lines.append(line_to_point(img, left_avg))
    if len(right_lane) > 0:
        lane_lines.append((line_to_point(img, right_avg)))

    return lane_lines


# Create points from the lane lines with slop and intercept
def line_to_point(img, line):
    slop = line[0]
    intercept = line[1]
    height = img.shape[0]
    width = img.shape[1]

    #
    #
    #      left      right
    #     x1,y1      x1,y1
    #
    #
    #
    # x2,y2              x2,y2

    # y = mx + c
    # x = (y - c) / m

    y1 = int(height / 2)  # middle
    x1 = int((y1 - intercept) / slop)
    if x1 < 0:
        x1 = 0
    if x1 > width:
        x1 = width

    y2 = int(height)  # bottom
    x2 = int((y2 - intercept) / slop)
    if x2 < 0:
        x2 = 0
    if x2 > width:
        x2 = width
    print(x1, y1, x2, y2)
    return [[x1, y1, x2, y2]]


# steering angle
      
             # IMAGE
#             
#             
#
#
#
#
#
#
#             |-------- X_div       ^
#             |        /            |
#             |       /             |
#             |      /              |
#             |     /               |
#             | th /               y/2
#             |   /                 |
#             |  /                  |
#             | /                   |
#             |/                    _


# We can calculate th (theta) deviation angle by tan = (opposite side) / (adjacent side) = x_div / y/2
def steering_angle(img, lane, show=False):

    height = img.shape[0]
    width = img.shape[1]

    # if there is only one lane , we will set deviation to slope of the lane
    if len(lane) == 1:
        x1, y1, x2, y2 = lane[0][0]
        slope = x2 - x1
        x_deviation = slope

    # if two lines, get average of far end points
    else:
        l1x1, l1y1, l1x2, l1y2 = lane[0][0]
        l2x1, l2y1, l2x2, l2y2 = lane[1][0]
        average_point = int(l1x2 + l2x2 / 2)
        x_deviation = int(average_point) - int(width / 2)

        if show:
            steering_img = cv2.circle(img, (average_point, (int(height/2))), radius=3, color=(0, 0, 255), thickness=-1)
            steering_img = cv2.line(steering_img, (int(width / 2) , 0), (int(width / 2), height), (0, 255, 0), 1)
            cv2.imshow("Deviation", steering_img)

    line_length = int(height / 2)

    angle_to_middle_vertical_rad = math.atan(x_deviation / line_length)
    angle_to_middle_vertical_deg = int(angle_to_middle_vertical_rad * 180.0 / math.pi)

    return angle_to_middle_vertical_deg





# frame = cv2.imread('lane.jpg')
# frame = cv2.resize(frame, (640, 480))

# # frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # frame_masked = mask_img(frame_hsv)


# frame_edge = detect_edges(frame, show=True)
# region_of_interest = crop_roi(frame_edge)
# all_lines = detect_lines(region_of_interest)

# draw_lines(frame, all_lines)
# #lane_search_area(frame, boundary=1/3)

# lane_markings = group_lines(frame, all_lines)
# print("LANE : " , lane_markings)

# steering = steering_angle(frame, lane_markings)
# print(steering)

# draw_lines(frame, lane_markings)




# #print(all_lines)
# # cv2.imshow("lane image", region_of_interest)
# cv2.waitKey(0)


