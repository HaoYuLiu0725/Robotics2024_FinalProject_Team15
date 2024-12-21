import cv2
import numpy as np
import os
from final_project.camera import Camera

PATH_CALIB_INFO = "./calibration_info"
PATH_CAPTURE_IMG = "./captured_imgs"

# define the size in mm in chessboard
SQR_SIZE = 25 ### MANUAL EDIT ###

# define the number of chessboard corners
SQR_NUM = (8, 6) ### MANUAL EDIT ###

# define the size in meters in aruco marker
ARUCO_MARKER_SIZE = 0.019

# if we don't have calibration info, we do camera calibration using images
NEED_CAMERA_CALIBRATION_IMAGES = False

class CanvasLocalization():
    def __init__(self, aruco_marker_size):
        # the size of the detected markers
        self.aruco_marker_size = aruco_marker_size

    def get_canvas_location(self, img_path, camera_matrix, dist_coeffs):
        print('>>>> Reading aruco images...')
        img = cv2.imread(img_path)
        # Load ArUco dictionary and detection parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()

        # Detect markers
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img)
        
        markerIds = markerIds.flatten()

        # Zip together markerIds and markerCorners and sort by markerIds
        sorted_marker_data = sorted(zip(markerIds, markerCorners), key=lambda x: x[0])

        # Unzip the sorted list into separate lists
        sorted_markerIds, sorted_markerCorners = zip(*sorted_marker_data)

        # Convert back to numpy arrays (if needed)
        sorted_markerIds = np.array(sorted_markerIds)
        sorted_markerCorners = np.array(sorted_markerCorners)

        if sorted_markerIds is not None:
            all_rvecs = []
            all_tvecs = []
            all_pixels = []
            for i in range(len(sorted_markerIds)):
                rvecs, tvecs, pixel_positions = self.my_estimatePoseMarkers(sorted_markerCorners[i], camera_matrix, dist_coeffs)
                # Because the top-left corner of each aruco marker is placed nearest to the center of canvas, we can simply use the first returned rvec, tvec.
                all_rvecs.append(rvecs[0])
                all_tvecs.append(tvecs[0])
                all_pixels.append(pixel_positions[0])

            v1 = all_tvecs[0] - all_tvecs[1]
            v2 = all_tvecs[2] - all_tvecs[1]

            # Calculate the fourth corner position
            all_tvecs.append(all_tvecs[1] + v1 + v2)

            # Convert rvecs to rotation matrices
            R1, _ = cv2.Rodrigues(all_rvecs[0])
            R2, _ = cv2.Rodrigues(all_rvecs[1])
            R3, _ = cv2.Rodrigues(all_rvecs[2])

            # Compute the average rotation matrix
            R_avg = (R1 + R2 + R3) / 3.0

            # Convert average rotation matrix back to rotation vector
            tmp_rvec, _ = cv2.Rodrigues(R_avg)
            all_rvecs.append(tmp_rvec)

            pixel_position_4th, _ = cv2.projectPoints(
                np.array([all_tvecs[3]]),  # The 4th corner's position in the camera frame
                np.zeros((3, 1)),       # No additional rotation
                np.zeros((3, 1)),       # No additional translation
                camera_matrix, dist_coeffs
            )

            all_pixels.append(pixel_position_4th.flatten())

            # Output all corners' tvec, rvec
            # (id = 0) -> (id = 1) -> (id = 2) -> (blank)
            for i in range(len(all_tvecs)):
                print("Corner", i, "position:", all_tvecs[i])
                print("Corner", i, "orientation:", all_rvecs[i])
                print("Pixel", i, "position:", all_pixels[i])
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, all_rvecs[i], all_tvecs[i], ARUCO_MARKER_SIZE * 0.5)
                pixel = all_pixels[i]
                pixel = tuple(map(int, pixel))
                cv2.circle(img, pixel, radius=5, color=(255, 0, 255), thickness=-1)

            # Display the result
            show_img = cv2.resize(img, (1280, 960), interpolation=cv2.INTER_AREA)
            cv2.imshow("Detected Markers", show_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            pixel_np = np.array(all_pixels)
            return pixel_np
        else:
            print("No markers detected.")

    # reference:
    # https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    # Adjusted by ChatGPT so that it estimate each of the 4 corners of each marker instead of the center of each marker
    def my_estimatePoseMarkers(self, corners, mtx, distortion):
        """
        This will estimate the rvec and tvec for each of the 4 corners of each marker detected by:
          corners, ids, rejectedImgPoints = detector.detectMarkers(image)

        Parameters:
            corners: is an array of detected corners for each detected marker in the image
            mtx: is the camera matrix
            distortion: is the camera distortion matrix

        Returns:
            list of rvecs_corners, tvecs_corners and pixel positions
        """
        # Define the 3D positions of the corners in the marker's local coordinate system
        half_size = self.aruco_marker_size / 2
        marker_corners_3D = np.array([[-half_size, half_size, 0],
                                  [half_size, half_size, 0],
                                  [half_size, -half_size, 0],
                                  [-half_size, -half_size, 0]], dtype=np.float32)

        rvecs_corners = []
        tvecs_corners = []
        pixel_positions = []

        for c in corners:
            # SolvePnP for the entire marker first
            success, rvec_marker, tvec_marker = cv2.solvePnP(
                marker_corners_3D, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if not success:
                continue

            # Convert rvec to a rotation matrix
            R_marker, _ = cv2.Rodrigues(rvec_marker)

            # Calculate the pose of each corner
            for i, corner_3D in enumerate(marker_corners_3D):
                # Transform corner's 3D position from marker frame to camera frame
                corner_camera_frame = R_marker @ corner_3D + tvec_marker.flatten()
                
                # tvec for each corner is its position in the camera frame
                tvecs_corners.append(corner_camera_frame)

                # rvec for each corner is the same as the marker's rvec
                rvecs_corners.append(rvec_marker)

                # Project the 3D corner to the image plane
                corner_pixel, _ = cv2.projectPoints(
                    np.array([corner_camera_frame]),  # Single 3D point
                    np.zeros((3, 1)),                 # No additional rotation
                    np.zeros((3, 1)),                 # No additional translation
                    mtx, distortion
                )
                pixel_positions.append(corner_pixel.flatten())

        return rvecs_corners, tvecs_corners, pixel_positions

PATH_TEMP_FILE = "./temp_files"
if __name__ == '__main__':
    camera = Camera(PATH_CALIB_INFO, 1280, 960)
    canvas_localization = CanvasLocalization(ARUCO_MARKER_SIZE)

    if NEED_CAMERA_CALIBRATION_IMAGES:
        camera.intrinsic.run_calibration(PATH_CAPTURE_IMG, SQR_SIZE, SQR_NUM, True)

    camera.intrinsic.load_param()
    camera_matrix = camera.intrinsic.mat_cam
    dist_coeffs = camera.intrinsic.dist_coeff

    image_path = PATH_TEMP_FILE + "/canvas.jpg"
    canvas_localization.get_canvas_location(image_path, camera_matrix, dist_coeffs)

