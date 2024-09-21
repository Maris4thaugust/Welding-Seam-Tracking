import numpy as np
import cv2
import glob

# Dimensions of the chessboard pattern
row = 6
col = 9
square_size = 0.02

# Prepare object points
objp = np.zeros((row * col, 3), np.float32)
objp[:, :2] = np.mgrid[0:col, 0:row].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []
imgpoints = []
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Get a list of calibration image file names
image_files = glob.glob(r".\Calibration\Img_v2\*.jpg")

# Iterate through each calibration image
i = 0
for fname in image_files:
    # Load the image
    img = cv2.imread(fname)

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the corners of the chessboard pattern
    ret, corners = cv2.findChessboardCorners(gray, (col, row), None)

    # If corners are found, refine them and add them to the arrays
    if ret == True:
        objpoints.append(objp)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # for c in imgpoints[i]:
        #     pxy = c[0]
        #     cv2.circle(img, (int(pxy[0]), int(pxy[1])), 3, (0, 0, 255), -1)
        # cv2.imshow(f'Cal Img {i+1}', img)
        # i += 1

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)
print(dist)
# print(rvecs)
# print(tvecs)

R, _ = cv2.Rodrigues(rvecs[0])
# Tính toán ma trận ngoại của camera
extrinsic_matrix = np.concatenate((R, tvecs[0]), axis=1)
print(extrinsic_matrix)

# Test the calibration
img = cv2.imread(r"Chek.jpg")
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
# cv2.imwrite("./3_1.jpg", dst)

# Show the original and undistorted images side by side
cv2.imshow('Original', img)
cv2.imshow('Undistorted', dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
