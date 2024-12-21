from pathlib import Path
import numpy as np
import cv2
from math import cos, sin, pi
import os
import glob

def get_centroid_and_corner(img: np.array, timeout: int = 0, show_process = False, save_result = False) -> list:
    """

    :param img: the input image in OpenCV format.
    :param timeout: the timeout to close the window.
                If timeout=0, the window will close when pressing ESC. Default: 0.
    :return: a list of centroids which represent as [Cx, Cy, principal_angle].
    """
    # img_path = os.path.join(input_path, file_name)
    # img = cv2.imread(img_path)
    h, w = img.shape[:2]
    cv2.imshow("Original", img)

    # 灰階 and 二值化
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    if show_process:
        cv2.imshow("gray", gray)
        cv2.imshow("binary", binary)

    # 開運算去除雜訊
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    if show_process:
        cv2.imshow("opening", opening)
    
    # 高斯濾波
    blur_img = cv2.GaussianBlur(opening, (5, 5), 2)
    if show_process:
        cv2.imshow("blur_img", blur_img)

    blur_img = np.float32(blur_img)
    dst = cv2.cornerHarris(blur_img,4,3,0.04)

    
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    cv2.imshow('cornerHarris',dst)
    dst_list = np.array(np.argwhere((dst>0.01).T))
    
    # 用Canny找到contours
    low_threshold = 50
    high_threshold = 100
    canny = cv2.Canny(opening, low_threshold, high_threshold)
    # canny = cv2.Canny(blur_img, low_threshold, high_threshold)
    contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # Visualize contours
    img_contour = cv2.drawContours(img.copy(), contours, -1, (0, 255, 0), 2)
    if show_process:
        cv2.imshow("canny", canny)
        cv2.imshow("img_contour", img_contour)

    centroid_list = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        if area > 1000 and area < 150000:
            M = cv2.moments(cnt) # Calculate moments
            if M['m00'] != 0:
                # 計算形心 (Cx, Cy)
                Cx = M['m10'] / M['m00']
                Cy = M['m01'] / M['m00']
            else:
                Cx, Cy = 0, 0
            # 畫形心
            cv2.circle(img, (int(Cx), int(Cy)), 5, (0, 0, 255), -1) # Draw the Centroid in red
            center_np = np.array([int(Cx), int(Cy)])
            dst_r = np.sqrt(np.sum((dst_list-center_np)**2, axis=-1))
            dst_r_mean = np.mean(dst_r)
            temp_list = dst_list[dst_r <= dst_r_mean/4]
            img_temp = img.copy()
            for i in range(len(temp_list)):
                img_temp[temp_list[i][1], temp_list[i][0]] = (0, 0, 255)
            cv2.imshow("test_img", img_temp)
            max_arg = (-1, -1)
            max_r = 0
            for i in range(len(temp_list)):
                for j in range(i, len(temp_list)):
                    temp_r = np.sum((temp_list[i] - temp_list[j])**2)
                    if temp_r > max_r:
                        max_arg = (i, j)
                        max_r = temp_r
            
            temp_v = temp_list[max_arg[0]] - temp_list[max_arg[1]]
            cv2.circle(img, (int(temp_list[max_arg[0]][0]), int(temp_list[max_arg[0]][1])), 5, (0, 255, 0), -1) # Draw the Centroid in red
            cv2.circle(img, (int(temp_list[max_arg[1]][0]), int(temp_list[max_arg[1]][1])), 5, (0, 255, 0), -1) # Draw the Centroid in red
            print(temp_v[0], temp_v[1])
            principal_angle = np.rad2deg(np.arctan2(temp_v[1], temp_v[0]))
            temp_v_len = np.sqrt(temp_v[0]**2+temp_v[1]**2)
            temp_v = temp_v/np.sqrt(temp_v[0]**2+temp_v[1]**2)

            def rotate_2d(vector, angle):
                rotation_matrix = np.array([
                    [cos(angle), -sin(angle)],
                    [sin(angle), cos(angle)]
                ])
                return np.dot(rotation_matrix, vector)
            
            temp_v = rotate_2d(temp_v, pi/4)
            principal_angle = np.rad2deg(np.arctan2(temp_v[1], temp_v[0]))

            # # Compute Principal Component Analysis (PCA)
            # data_pts = np.array(cnt, dtype=np.float64).reshape(-1, 2)
            # mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
            # principal_axis = eigenvectors[0] # Extract the principal axis

            # 計算正向和反向延伸的點，確保線段延伸到圖像邊框外
            t = max(h, w)  # 大概的延伸係數，確保延伸到邊界
            # p1 = (int(Cx + t * principal_axis[0]), int(Cy + t * principal_axis[1]))
            # p2 = (int(Cx - t * principal_axis[0]), int(Cy - t * principal_axis[1]))
            p1 = (int(Cx + t * temp_v[0]), int(Cy + t * temp_v[1]))
            p2 = (int(Cx - t * temp_v[0]), int(Cy - t * temp_v[1]))

            cv2.line(img, p1, p2, (255, 0, 0), 2) # Draw the principal line in blue

            # principal_angle = np.rad2deg(np.arctan2(principal_axis[1], principal_axis[0]))

            # Text to be displayed above the centroid
            text_centroid = f"Centroid = ({Cx:.4f}, {Cy:.4f})"
            text_angle = f"Principal Angle = {principal_angle:.4f} degrees"
            print(text_centroid)
            print(text_angle)
            # Calculate the position for the text (slightly above the centroid)
            text_pos = (int(Cx)-90, int(Cy))
            # Display the centroid and angle information above the centroid point
            cv2.putText(img, text_centroid, (text_pos[0], text_pos[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(img, text_angle, (text_pos[0], text_pos[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            centroid_list.append([Cx, Cy, principal_angle])

            

    cv2.imshow("Result", img)
    if save_result:
        cv2.imwrite("result.jpg", img)
    cv2.waitKey(timeout)
    cv2.destroyAllWindows()
    return centroid_list

if __name__ == "__main__":
    img = cv2.imread("hw4_blocks.jpg")
    get_centroid_and_corner(img, show_process=True)