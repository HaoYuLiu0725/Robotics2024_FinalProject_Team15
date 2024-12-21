from pathlib import Path
import numpy as np
import cv2
import os
import glob

__all__ = ['Intrinsic', 'Extrinsic', 'Camera']

def eul2rotm_zyx(theta_z, theta_y, theta_x):
    """
    Converts Euler angles (ZYX order: yaw, pitch, roll) in degrees to a 3x3 rotation matrix.

    :param theta_z: Rotation angle around the Z-axis (yaw) in degrees
    :param theta_y: Rotation angle around the Y-axis (pitch) in degrees
    :param theta_x: Rotation angle around the X-axis (roll) in degrees
    :return: 3x3 numpy array representing the rotation matrix
    """
    # Convert degrees to radians
    theta_z = np.deg2rad(theta_z)
    theta_y = np.deg2rad(theta_y)
    theta_x = np.deg2rad(theta_x)

    # Rotation matrix around the Z-axis
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z),  np.cos(theta_z), 0],
                    [0,               0,                1]])

    # Rotation matrix around the Y-axis
    Ry = np.array([[np.cos(theta_y),  0, np.sin(theta_y)],
                    [0,                1, 0],
                    [-np.sin(theta_y), 0, np.cos(theta_y)]])

    # Rotation matrix around the X-axis
    Rx = np.array([[1, 0,                0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x),  np.cos(theta_x)]])

    # Combined rotation matrix (ZYX order)
    R = Rz @ Ry @ Rx
    return R
            
def arm_point_to_rotm_tevc(arm_point: str):
    """
    Convert arm pose string to rotation vector and translation vector.

    :param arm_point: string in format "X, Y, Z, Rx, Ry, Rz"
    :return: rotm (rotation matrix), tvec (translation vector)
    """
    pos = [float(value) for value in arm_point.split(",")]
    tvec = np.array(pos[:3], dtype=np.float64).reshape(3, 1)  # Translation vector
    rotm = eul2rotm_zyx(pos[-1], pos[-2], pos[-3])
    # rvec = cv2.Rodrigues(rotm)
    return rotm, tvec

class Intrinsic:
    """
    Intrinsic Information
    """

    def __init__(self, path_calib_info: str, width: int, height: int):
        """

        :param path_calib_info: path that contain intrinsic information.
        :param width: the image width.
        :param height: the image height.
        """
        self.width = width
        self.height = height
        self.path_calib_info = Path(path_calib_info)

        self.mat_cam = None
        self.mat_cam_new = None
        self.mat_cam_new_inv = None
        self.dist_coeff = None
        self.roi = None
        self.fx, self.fy = 0.0, 0.0
        self.cx, self.cy = 0.0, 0.0

    def run_calibration(self, img_path:str, sqr_size:int, sqr_num:tuple=(8, 6), show_img = True):
        """
        Run camera calibration using OpenCV.  
        Only use **.jpg** file to run calibration

        :param img_path: path of folder have images(.jpg only) of chessboard to perform calibration
        :param sqr_size: square size in [mm] of chessboard
        :param sqr_num: number of chessboard corners eg. 8 X 6 = (8, 6)
        """
        # define termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        obj_idx = np.zeros((sqr_num[0] * sqr_num[1], 3), np.float32)
        # add SQR_SIZE to account for SQR_SIZE mm per square in grid
        obj_idx[:, :2] = np.mgrid[0: sqr_num[0], 0: sqr_num[1]].T.reshape(-1, 2) * sqr_size

        # arrays to store object points and image points from all images
        obj_points = [] # 3d point in the real world space
        img_points = [] # 2d points in the image plane

        # define visualization window
        window_name = 'Image'
        cv2.namedWindow(window_name, cv2.WINDOW_FULLSCREEN)

        try:          
            print('>>>> Reading calibration images...')
            find_img_path = os.path.join(img_path, '*.jpg')
            img_names = glob.glob(find_img_path)

            # Define a sorting key to extract the numeric part of the filename
            def _extract_number(file_path):
                base_name = os.path.basename(file_path)  # Get the file name (e.g., cap_0.jpg)
                number = os.path.splitext(base_name)[0].split('_')[-1]  # Extract the numeric part
                return int(number)  # Convert to integer for numerical sorting

            # Sort the list
            img_names_sorted = sorted(img_names, key=_extract_number)
            print(img_names_sorted)

            found = 0
            for fname in img_names_sorted:
                img = cv2.imread(str(fname))
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, (sqr_num[0], sqr_num[1]), None)

                # if found, add object points, image points, (after refining them)
                if ret:
                    found += 1

                    obj_points.append(obj_idx)

                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    img_points.append(corners2)

                    if show_img:
                        # Draw and display the corners
                        cv2.drawChessboardCorners(img, (sqr_num[0], sqr_num[1]), corners2, ret)
                        cv2.imshow(window_name, img)
                        cv2.waitKey(500)
                else:
                    print(f"{fname} image mot found chessboard")

            print(f'{found} images used for calibration')
            cv2.destroyAllWindows()

            print('>>>> Starting calibration...')
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

            print('>>>> Verifying calibration...')
            # estimate re-projection error
            mean_error = 0
            for i in range(len(obj_points)):
                imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
            
            print("Total Re-projection Error: {}".format(mean_error/len(obj_points)))

            print('>>>> Saving calibration information...')
            # create save directory if not exists
            os.makedirs(self.path_calib_info, exist_ok=True)
            
            print(f'Camera Matrix:\n{mtx}')
            mtx_path = os.path.join(self.path_calib_info, "camera_matrix.npy")
            np.save(mtx_path, mtx)
            print(f'Distortion Coefficients:\n{dist}')
            dist_path = os.path.join(self.path_calib_info, "distortion_coeff.npy")
            np.save(dist_path, dist)

            # load into self's parameter
            self.mat_cam = mtx
            self.dist_coeff = dist

            self.mat_cam_new, self.roi = cv2.getOptimalNewCameraMatrix(self.mat_cam, self.dist_coeff, (self.width, self.height), 1, (self.width, self.height))
            self.mat_cam_new_inv = np.linalg.inv(self.mat_cam_new)

            self.fx, self.fy = self.mat_cam_new[0, 0], self.mat_cam_new[1, 1]
            self.cx, self.cy = self.mat_cam_new[0, 2], self.mat_cam_new[1, 2]

        except Exception as e:
            print(e)       

    def load_param(self):
        assert self.path_calib_info.exists()
        try:
            self.mat_cam = np.load(str(self.path_calib_info / 'camera_matrix.npy'))
            self.dist_coeff = np.load(str(self.path_calib_info / 'distortion_coeff.npy'))

            # if using Alpha = 0, so we discard the black pixels from the distortion.
            # This helps make the entire region of interest is the full dimensions of the image (after `undistort`)
            # If using Alpha = 1, we retain the black pixels, and obtain the region of interest as the valid pixels for the matrix.
            # I will use Alpha = 1, so that I don't have to run undistort and can just calculate my real world x,y
            self.mat_cam_new, self.roi = cv2.getOptimalNewCameraMatrix(self.mat_cam, self.dist_coeff, (self.width, self.height), 1, (self.width, self.height))
            self.mat_cam_new_inv = np.linalg.inv(self.mat_cam_new)

            self.fx, self.fy = self.mat_cam_new[0, 0], self.mat_cam_new[1, 1]
            self.cx, self.cy = self.mat_cam_new[0, 2], self.mat_cam_new[1, 2]
        except Exception as e:
            print(e)


class Extrinsic:
    def __init__(self, path_calib_info: str):
        """

        :param path_calib_info: path that contain intrinsic information.
        """

        self.path_calib_info = Path(path_calib_info)

        # for eye-in-end
        self.HTM_c2g = None

        # for eye-to-hand
        self.tvec = None
        self.rvec = None
        self.scale = None
        self.mat_R = None
        self.mat_R_inv = None
        self.mat_Rt = None

    def run_calibration_eye_in_hand(self, arm_points, mtx, dist, img_path:str, sqr_size:int, sqr_num:tuple=(8, 6), show_img = False):
        """
        Run eye-in-hand calibration using OpenCV calibrateHandEye().  
        Only use **.jpg** file to run calibration

        :param arm_points: list of string describing arm pose: "X, Y, Z, Rx, Ry, Rz"
        :param mtx: camera matrix (Intrinsic parameter)
        :param dist: distortion coefficients (Intrinsic parameter)
        :param img_path: path of folder have images(.jpg only) of chessboard to perform calibration
        :param sqr_size: square size in [mm] of chessboard
        :param sqr_num: number of chessboard corners eg. 8 X 6 = (8, 6)
        """

        # ---------- Get R_gripper2base and t_gripper2base -----------------------------------------------------       
        R_g2b_all = []
        t_g2b_all = []
        for pt in arm_points:
            R_g2b, t_g2b = arm_point_to_rotm_tevc(pt)
            R_g2b_all.append(R_g2b)
            t_g2b_all.append(t_g2b)
            # print(f'Arm pose: {pt}')
            # print(f'Rotation matrix: {R_g2b}')
            # print(f'Translation vector: {t_g2b}')

        # ---------- Get R_target2cam and t_target2cam -----------------------------------------------------
        # define termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        obj_pts = np.zeros((sqr_num[0] * sqr_num[1], 3), np.float32)
        # add SQR_SIZE to account for SQR_SIZE mm per square in grid
        obj_pts[:, :2] = np.mgrid[0: sqr_num[0], 0: sqr_num[1]].T.reshape(-1, 2) * sqr_size

        # print(obj_pts)

        # define visualization window
        window_name = 'Image'
        cv2.namedWindow(window_name, cv2.WINDOW_FULLSCREEN)

        try:          
            print('>>>> Reading calibration images...')
            find_img_path = os.path.join(img_path, '*.jpg')
            img_names = glob.glob(find_img_path)

            # Define a sorting key to extract the numeric part of the filename
            def _extract_number(file_path):
                base_name = os.path.basename(file_path)  # Get the file name (e.g., cap_0.jpg)
                number = os.path.splitext(base_name)[0].split('_')[-1]  # Extract the numeric part
                return int(number)  # Convert to integer for numerical sorting

            # Sort the list
            img_names_sorted = sorted(img_names, key=_extract_number)
            print(img_names_sorted)

            R_t2c_all = []
            t_t2c_all = []
            found = 0
            for fname in img_names_sorted:
                img = cv2.imread(str(fname))
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, (sqr_num[0], sqr_num[1]), None)

                # if found, add object points, image points, (after refining them)
                if ret:
                    found += 1
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    img_pts = corners2
                    print(img_pts)
                    _, R_t2c, t_t2c = cv2.solvePnP(obj_pts, img_pts, mtx, dist)
                    R_t2c_all.append(R_t2c)
                    t_t2c_all.append(t_t2c)
                    if show_img:
                        # Draw and display the corners
                        cv2.drawChessboardCorners(img, (sqr_num[0], sqr_num[1]), corners2, ret)
                        cv2.imshow(window_name, img)
                        cv2.waitKey(500)

            print(f'{found} images used for calibration')
            cv2.destroyAllWindows()

        # ---------- calibrateHandEye() ---------------------------------------------------------------------------------
            print('>>>> Starting calibrateHandEye()...')
            R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b_all, t_g2b_all, R_t2c_all, t_t2c_all)

            print(f"R_cam2gripper:\n{R_c2g}")
            print(f"t_cam2gripper:\n{t_c2g}")

            HTM_c2g = np.column_stack((R_c2g, t_c2g))
            HTM_c2g = np.row_stack((HTM_c2g, np.array([0, 0, 0, 1])))
            print(f"HTM_c2g:\n{HTM_c2g}")

            for i in range(len(arm_points)):
                HTM_g2b = np.column_stack((R_g2b_all[i], t_g2b_all[i]))
                HTM_g2b = np.row_stack((HTM_g2b, np.array([0, 0, 0, 1])))
                # print(f"HTM_g2b:\n{HTM_g2b}")

                HTM_t2c = np.column_stack((cv2.Rodrigues(R_t2c_all[i])[0], t_t2c_all[i]))
                HTM_t2c = np.row_stack((HTM_t2c, np.array([0, 0, 0, 1])))
                # print(f"HTM_t2c:\n{HTM_t2c}")

                HTM_t2b = HTM_t2c @ HTM_c2g @ HTM_g2b
                print(f"HTM_t2b_{i}:\n{HTM_t2b}")

            np.save(str(self.path_calib_info / 'HTM_c2g.npy'), HTM_c2g)
            self.HTM_c2g = HTM_c2g

        except Exception as e:
            print(e)

    def run_calibration_eye_to_hand(self, world_points, image_points, mtx, dist, mtx_new_inv):
        """
        Run eye-to-hand calibration using OpenCV solvePnP().

        :param world_points: 3D points in world coordinate (mm)
        :param image_points: 2D points in image coordinate (pixels)
        :param mtx: camera matrix (Intrinsic parameter)
        :param dist: distortion coefficients (Intrinsic parameter)
        :param mtx_new_inv: new camera matrix (Intrinsic parameter)
        """
        print('>>>> Solving PnP...')
        ret, rvec, tvec = cv2.solvePnP(world_points, image_points, mtx, dist)
        # compute rotation matrix
        mat_R, mat_jacob = cv2.Rodrigues(rvec)
        # compute Rt matrix
        mat_Rt = np.column_stack((mat_R, tvec))
        # compute projection matrix
        mat_P = mtx_new_inv.dot(mat_Rt)

        print('>>>> Estimate Scaling Factor...')
        s_arr = np.array([0], dtype=np.float32)
        s_describe = np.array([0] * len(world_points), dtype=np.float32)

        for i in range(0, len(world_points)):
            print(f'POINT #{i}')
            print("Forward: From World Points, Find Image Pixel...")
            XYZ1 = np.array(
                [[world_points[i, 0], world_points[i, 1], world_points[i, 2], 1]],
                dtype=np.float32
            ).T
            suv1 = mat_P.dot(XYZ1)
            s = suv1[2, 0]
            uv1 = suv1 / s
            print(f'Scaling Factor: {s}')
            s_arr = np.array([s / len(world_points) + s_arr[0]], dtype=np.float32)
            s_describe[i] = s

            print("Solve: From Image Pixels, find World Points...")
            uv1_ = np.array(
                [[image_points[i, 0], image_points[i, 1], 1]],
                dtype=np.float32
            ).T
            suv1_ = s * uv1_
            print("Get camera coordinates, multiply by inverse Camera Matrix, subtract tvec1")
            xyz_c = mtx_new_inv.dot(suv1_) - tvec
            mat_R_inv = np.linalg.inv(mat_R)
            XYZ_ = mat_R_inv.dot(xyz_c)
            print(f'XYZ: {XYZ_}')

        s_mean, s_std = np.mean(s_describe), np.std(s_describe)
        print('Result')
        print(f'Mean: {s_mean}')
        print(f'Std: {s_std}')
        print(">>>>>> S Error by Point")
        for i in range(len(world_points)):
            print(f'POINT #{i}')
            print(f'S: {s_describe[i]} Mean: {s_mean} Error: {s_describe[i] - s_mean}')

        print('>>>> Saving to file....')

        np.save(str(Path(self.path_calib_info) / 'tvec.npy'), tvec)
        np.save(str(Path(self.path_calib_info) / 'rvec.npy'), rvec)
        np.save(str(Path(self.path_calib_info) / 'rotation_matrix.npy'), mat_R)
        np.save(str(Path(self.path_calib_info) / 'extrinsic_matrix.npy'), mat_Rt)
        np.save(str(Path(self.path_calib_info) / 'projection_matrix.npy'), mat_P)
        np.save(str(Path(self.path_calib_info) / 'scaling_factor.npy'), s_arr)
        
    def load_param(self, mode: str = "EyeToHand"):
        """
        Load calibration parameters for different modes (EyeToHand or EyeInHand).
        
        :param mode: str, optional
            Mode of calibration, "EyeToHand" or "EyeInHand" (default: "EyeToHand")
        """
        assert self.path_calib_info.exists()

        if mode == "EyeToHand":
            self.rvec = np.load(str(self.path_calib_info / 'rvec.npy'))
            self.tvec = np.load(str(self.path_calib_info / 'tvec.npy'))
            self.scale = np.load(str(self.path_calib_info / 'scaling_factor.npy'))[0]

            self.mat_R, _ = cv2.Rodrigues(self.rvec)
            self.mat_R_inv = np.linalg.inv(self.mat_R)
            self.mat_Rt = np.column_stack((self.mat_R, self.tvec))

        elif mode == "EyeInHand":
            self.HTM_c2g = np.load(str(self.path_calib_info / 'HTM_c2g.npy'))
        
        else:
            raise ValueError("Invalid mode. Please choose 'EyeToHand' or 'EyeInHand'.")

class Camera:
    """
    Class to handle camera calibration, intrinsic and extrinsic parameters.
    1. If camera calibration not done yet, please run **intrinsic.run_calibration()** and **extrinsic.run_calibration()** first to get calibration info.  
    1.1 For intrinsic.run_calibration(), please take more than 20 photos of chessboard using camera first, and store as .jpg file in a folder. Then pass img_path, sqr_size, sqr_num into function.  
    1.2 For extrinsic.run_calibration(), 
    2. If already have calibration info, or after first calibration, please run **load_param()** to get all intrinsic and extrinsic parameters.
    """
    def __init__(self, path_calib_info: str, width: int, height: int):
        """
        
        :param path_calib_info: path that contain intrinsic information.
        :param width: the image width.
        :param height: the image height.
        """
        self.intrinsic = Intrinsic(path_calib_info, width, height)
        self.extrinsic = Extrinsic(path_calib_info)
        self.width = width
        self.height = height

    def load_param(self, mode: str = "EyeToHand"):
        """
        Load both intrinsic and extrinsic camera calibration parameters.
        """
        self.intrinsic.load_param()
        self.extrinsic.load_param(mode)

    def compute_XYZ_eye_in_hand(self, arm_point: str, u: int, v: int) -> list:
        """
        Compute the world coordinates by the given image pixel coordinates
        image was capture with eye-in-hand camera

        :param arm_points: a string describing arm pose that capture this image: "X, Y, Z, Rx, Ry, Rz"
        :param u: the value along x-axis.
        :param v: the value along y-axis.
        :return: the real-world coordinate which represent as [X, Y, Z].
        """
        R_g2b, t_g2b = arm_point_to_rotm_tevc(arm_point)
        HTM_g2b = np.column_stack((R_g2b, t_g2b))
        HTM_g2b = np.row_stack((HTM_g2b, np.array([0, 0, 0, 1])))
        # print(f"HTM_g2b:\n{HTM_g2b}")

        HTM_c2b = self.extrinsic.HTM_c2g @ HTM_g2b
        R_c2b = HTM_c2b[:3, :3]
        t_c2b = HTM_c2b[:3, 3].reshape(3, 1)
        R_c2b_inv = np.linalg.inv(R_c2b)
        # print(f"R_c2b:\n{R_c2b}")
        # print(f"t_c2b:\n{t_c2b}")

        # solve: from image pixels, find world coordinate
        z_height = t_g2b[2][0]
        uv1 = np.array([[u, v, 1]], dtype=np.float32).T
        uv1 *= (z_height-60)
        # suv1 = self.extrinsic.scale * uv1
        # get camera coordinates
        xyz_c = self.intrinsic.mat_cam_new_inv.dot(uv1)
        xyz1 = np.row_stack((xyz_c, np.array([1])))
        # get real-world coordinate
        xyz = HTM_c2b.dot(xyz1)
        xyz = np.squeeze(xyz, axis=1).tolist()
        XYZ = [xyz[0], xyz[1], xyz[2]] # [X, Y, Z]
        return XYZ

    def compute_XYZ_eye_to_hand(self, u: int, v: int) -> list:
        # solve: from image pixels,find world coordinate
        uv1_ = np.array([[u, v, 1]], dtype=np.float32).T
        suv1_ = self.extrinsic.scale * uv1_
        # get camera coordinates
        xyz_c = self.intrinsic.mat_cam_new_inv.dot(suv1_) - self.extrinsic.tvec
        # get real-world coordinate
        XYZ_ = self.extrinsic.mat_R_inv.dot(xyz_c)
        return XYZ_

    def undistort_image(self, img: np.array):
        """
        Remove the distortion effect form image.

        :param img: the input image.
        :return: the undistorted image.
        """
        return cv2.undistort(img, self.intrinsic.mat_cam, self.intrinsic.dist_coeff, None, self.intrinsic.mat_cam_new)

    def get_centroid(self, img: np.array, timeout: int = 0, show_process = False, save_result = False) -> list:
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
        # blur_img = cv2.GaussianBlur(opening, (5, 5), 2)
        # if show_process:
        #     cv2.imshow("blur_img", blur_img)

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

                # Compute Principal Component Analysis (PCA)
                data_pts = np.array(cnt, dtype=np.float64).reshape(-1, 2)
                mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
                principal_axis = eigenvectors[0] # Extract the principal axis

                # 計算正向和反向延伸的點，確保線段延伸到圖像邊框外
                t = max(h, w)  # 大概的延伸係數，確保延伸到邊界
                p1 = (int(Cx + t * principal_axis[0]), int(Cy + t * principal_axis[1]))
                p2 = (int(Cx - t * principal_axis[0]), int(Cy - t * principal_axis[1]))

                cv2.line(img, p1, p2, (255, 0, 0), 2) # Draw the principal line in blue

                principal_angle = np.rad2deg(np.arctan2(principal_axis[1], principal_axis[0]))

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
    
    def get_centroid_and_corner(self, img: np.array, timeout: int = 0, show_process = False, save_result = False) -> list:
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
        if show_process:
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
                if show_process:
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
                temp_v = temp_v/np.sqrt(temp_v[0]**2+temp_v[1]**2)

                def rotate_2d(vector, angle):
                    rotation_matrix = np.array([
                        [np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]
                    ])
                    return np.dot(rotation_matrix, vector)
                
                temp_v = rotate_2d(temp_v, np.pi/4)
                principal_angle = np.rad2deg(np.arctan2(temp_v[0], temp_v[1]))

                # 計算正向和反向延伸的點，確保線段延伸到圖像邊框外
                t = max(h, w)  # 大概的延伸係數，確保延伸到邊界
                p1 = (int(Cx + t * temp_v[0]), int(Cy + t * temp_v[1]))
                p2 = (int(Cx - t * temp_v[0]), int(Cy - t * temp_v[1]))

                cv2.line(img, p1, p2, (255, 0, 0), 2) # Draw the principal line in blue

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
    
    def webcam_capture_images(self, save_path:str, file_name_prefix:str="cap", camera_id:int=0):
        """
        Use webcam to capture image.

        :param save_path: path of folder to save capture images.
        :param file_name_prefix: desire file name's prefix.
        :param camera_id: webcam camera id, usually is 0.
        """
        try:
            # create save directory if not exists
            os.makedirs(save_path, exist_ok=True)

            # define visualization window
            window_name = 'Webcam Capture'
            cv2.namedWindow(window_name, cv2.WINDOW_FULLSCREEN)

            # define video capture
            cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW) ### use this line if using Windows OS
            # cap = cv2.VideoCapture(camera_id) ### use this line if using Ubuntu OS
            i = 0
            while True:
                # capture video frame by frame
                ret, frame = cap.read()
                if ret:
                    frame_viz = frame.copy()
                    frame_viz = cv2.putText(frame_viz, "Press X to capture image", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 12, 255), 2)
                    frame_viz = cv2.putText(frame_viz, "Press Q to exit", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 12, 255), 2)
                    frame_viz = cv2.putText(frame_viz, f"Saved frames: {i}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 12, 255), 2)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('x'):
                        i += 1
                        save_img_path = os.path.join(save_path, f'{file_name_prefix}_{i}.jpg')
                        cv2.imwrite(save_img_path, frame)
                        print('>>>> Saved frame!')
                    elif key == ord('q'):
                        print('>>>> Exit')
                        break
                    cv2.imshow(window_name, frame_viz)
                else:
                    break

            cap.release()
            cv2.destroyAllWindows()

        except Exception as e:
            print(e)


    def preview_image(self, name: str, img: np.array, timeout: int = 0):
        """

        :param name: the window name.
        :param img: the input image in OpenCV format.
        :param timeout: the timeout to close the window.
                    If timeout=0, the window will close when pressing ESC. Default: 0.
        """

        cv2.namedWindow(name, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(name, img)
        cv2.waitKey(timeout)
        cv2.destroyAllWindows()

