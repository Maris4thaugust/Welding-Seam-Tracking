import numpy as np
import cv2
import time

def empty(a):
    pass

def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def detectOrigin(img):
    h_min = 150
    h_max = 179
    s_min = 50
    s_max = 255
    v_min = 55
    v_max = 255

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    while 1:
        co = []
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHSV, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)

        for y in range(mask.shape[0]):
            for x in range(mask.shape[1]):
                pixel_value = mask[y, x]
                if pixel_value > 0:
                    co.append([x, y])
                if len(co) > 3:
                    break
            if len(co) > 3:
                break

        if 0 <= len(co) < 3:
            break
        else:
            s_min += 1
            if s_min > 200:
                break
    return mask, co

def detectTopCorner(img):
    h_min = 125
    h_max = 179
    s_min = 55
    s_max = 255
    v_min = 0
    v_max = 255

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    while 1:
        co = []
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(imgHSV, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)

        for y in range(mask.shape[0]):
            for x in range(mask.shape[1]):
                pixel_value = mask[y, x]
                if pixel_value > 0:
                    co.append([x, y])
                if len(co) > 5:
                    break
            if len(co) > 5:
                break

        if 0 < len(co) < 5:
            break
        else:
            v_min += 1
            if v_min == 200:
                break
    return mask, co


def detectBottomCorner(img):
    b, g, r = cv2.split(img)
    x_max = np.argmax(r, axis=1)
    re = np.zeros((r.shape))
    for y in range(r.shape[0]):
        re[y, x_max[y]] = 255
        if (y > 0) and (x_max[y] - x_max[y-1] <0):
            return [x_max[y], y]
        elif (y>5) and (x_max[y] == x_max[y-1] == x_max[y-2] == x_max[y-3] == x_max[y-4] == x_max[y-5] == x_max[y-6]):
            return [x_max[y], y]
        
def createPointCloud(gc):
    points = []

    x_coords = np.array([-1, 0, 3])
    y_coords = np.array([2, 0, 1])
    z_coords = np.array([0, 0, 4])

    points = np.array([x_coords, y_coords, z_coords]).T

def detectOrigin_ver2(img):
    m = 0
    M = 150
    
    # Tìm ngưỡng cho từng kênh màu riêng biệt
    lower_white = np.array([m, m, m], dtype=np.uint8)
    upper_white = np.array([M, M, M], dtype=np.uint8)
    mask = cv2.inRange(img, lower_white, upper_white)

    # Loại bỏ các pixel thừa bằng phép toán morphology
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Áp dụng phép toán bitwise để tạo ra ảnh đã loại bỏ máu trắng
    img = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(opening))

    b, g, r = cv2.split(img)
    r = cv2.medianBlur(r, 3)
    # kernel = np.ones((2, 2), np.uint8)
    # r = cv2.dilate(r, kernel, iterations=1)

    y_max = np.argmax(r, axis=0)
    idx = []
    for i in range(1, len(y_max)):
        if (abs(y_max[i] - y_max[i-1]) > 5):
            idx.append(i)
    y_ndp = np.delete(y_max, idx)     

    re = np.zeros((r.shape), dtype=np.uint8)
    for x in range(r.shape[1]):
        if x == 0:
            re[y_max[x], x] = 255
        if (x > 0) and (abs(y_max[x] - y_max[x-1]) < 5):
            re[y_max[x], x] = 255
        else:
            continue

    my_list = list(y_max)
    indexes = []
    for i in range(len(my_list)):
        if my_list[i] == max(y_ndp):
            indexes.append(i)
    # time.sleep(0.25)
    ndp = []
    if len(indexes) > 2:
        for i in range(len(indexes)):
            ndp.append([indexes[i], max(y_ndp)])

    # plt.imshow(re, cmap="gray")
    # plt.show()
    return re, ndp

def get_OXY_function(x1, y1, x2, y2):
    """
    Tính và trả về hàm x = ay + b, với hai điểm đã cho.
    :param x1: tọa độ x của điểm đầu tiên
    :param y1: tọa độ y của điểm đầu tiên
    :param x2: tọa độ x của điểm thứ hai
    :param y2: tọa độ y của điểm thứ hai
    :return: hàm x = ay + b dưới dạng tuple (a, b)
    """
    # Tính hệ số góc m
    m = (x2 - x1) / (y2 - y1)

    # Tính hệ số bằng cách sử dụng điểm (x1, y1) và m
    b = x1 - m * y1

    # Trả về hàm dưới dạng tuple (a, b)
    return (m, b)

def getXYZmm(x0, y0, y0_delta, regY, regZ):
    coff1 = get_OXY_function(130, 8, 31, 290)
    # print(f"X = {coff1[0]}*Y + {coff1[1]}") 
    # print(f"In Y = 182 => X = {round(coff1[0]*y0 + coff1[1])}")
    X1 = round(coff1[0]*y0 + coff1[1])

    coff2 = get_OXY_function(460, 8, 540, 290)
    # print(f"X = {coff2[0]}*Y + {coff2[1]}") 
    # print(f"In Y = 182 => X = {round(coff2[0]*y0 + coff2[1])}")
    X2 = round(coff2[0]*y0 + coff2[1])

    # print(f"X in mm: {240*(x0 - X1)/(X2 - X1)}")
    X = round(240*(x0 - X1)/(X2 - X1), 5)
    Y = round(regY.predict([[y0_delta]])[0], 5)
    Z = round(regZ.predict([[y0, y0_delta - y0]])[0], 5)
    return X, Y, Z

def detectLightback(img):
    b, g, r = cv2.split(img)
    r = cv2.medianBlur(r, 3)

    y_max = np.argmax(r, axis=0)
    my_list = list(y_max)
    counts = np.bincount(my_list)
    max_index = np.argmax(counts)

    return max_index