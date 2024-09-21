import cv2
import numpy as np
import neccessary_method as nm
import joblib

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

regY = joblib.load('randomForest_Model_for_dataY.sav')
regZ = joblib.load('GradientBoostingRegressor_Model_for_dataZ.sav')

mtx = np.array([[698.98944598, 0, 350.98013604], 
                [0, 699.26868384, 232.9609348], 
                [0, 0, 1]])
dist = np.array([[-0.45737015, 0.44238336, 0.00212209, -0.00174516, -0.45640658]])

gc = [[]] # Global cornerss
test_time = 0
check_cam = 0
result = []
m = np.asarray([])

# i = 15
while (cap.isOpened()):
    ret, frame = cap.read()
    try:
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        frame_ud = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        frame_ud = frame_ud[40:345, 45:620]
        frame_sav = frame_ud.copy()

        # Vị trí cố định camera!
        cv2.line(frame_ud, (128, 8), (460, 8), [0, 0, 255], 2)
        cv2.line(frame_ud, (130, 8), (31, 290), [0, 0, 255], 2)
        cv2.line(frame_ud, (460, 8), (540, 290), [0, 0, 255], 2)
        cv2.line(frame_ud, (31, 290), (540, 290), [0, 0, 255], 2) # 235
        cv2.circle(frame_ud, (129, 8), 5, [0, 255, 0], -1) # Origin
        cv2.circle(frame_ud, (460, 8), 5, [0, 255, 0], -1)
        cv2.circle(frame_ud, (460, 8), 5, [0, 255, 0], -1)
        cv2.circle(frame_ud, (32, 290), 5, [0, 255, 0], -1)
        cv2.circle(frame_ud, (540, 290), 5, [0, 255, 0], -1)        
        
        frame_crop = frame_ud[50:240, 260:310].copy()
        frame_lb = frame_ud[120:275, 330:400].copy()
    except:
        check_cam += 1
        if check_cam > 25:
            break
        continue

    try:    
        m, c = nm.detectOrigin_ver2(frame_crop)
        if len(c) > 0:
            mean_point = np.mean(c, axis=0, dtype=int)
            result.append(mean_point)
            test_time += 1
            if len(result) == 5:
                print("Detected the Origin!")
                mean_point = np.mean(np.array(result), axis=0, dtype=int)                
                gc[0] = list(np.array(mean_point) + [260, 50])
     
                print("Needed Point: ", gc[0])
                X, Y, Z = nm.getXYZmm(gc[0][0], gc[0][1], (nm.detectLightback(frame_lb) + 120), regY, regZ)
                print(f"X = {round(X, 2)} mm, Y = {round((Y*10), 2)} mm, Z = {round(Z, 2)} mm")
                result = []
                test_time = 0
        else:
            if test_time > 10:
                print("\nFailed to detect origin!\n")
                test_time = 0
                result = []

        cv2.circle(frame_ud, (gc[0]), 3, [0, 255, 0], -1)
        cv2.circle(frame_crop, (mean_point[0], mean_point[1]), 1, (0, 255, 0), -1)
        cv2.putText(frame_ud, f'({round(X, 2)}, {round((Y*10), 2)}, {round(Z, 2)})', (gc[0][0]-5, gc[0][1]-5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0, 255, 0], 1, cv2.LINE_AA)
    except:
        pass

    cv2.imshow("Result Detect Origin", frame_crop)
    cv2.imshow("Result Detect Origin Binary", m)
    cv2.imshow("Y Detection", frame_lb)
    cv2.imshow("CHeck", frame_ud)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        cv2.imwrite(f"{43}_1.jpg", frame_sav[10:240, 275:300])
        cv2.imwrite(f"{43}_2.jpg", frame_sav[10:250, 330:400])
        cv2.imwrite("Chek.jpg", frame_sav)
        # cv2.imwrite(f"./Calibration/Img_v2/{i}.jpg", frame)
        break


