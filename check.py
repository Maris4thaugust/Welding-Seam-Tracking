import cv2
import numpy as np
import matplotlib.pyplot as plt

path = '43_2.jpg'
img_o = cv2.imread(path)
img = cv2.imread(path)

b, g, r = cv2.split(img)
r = cv2.medianBlur(r, 3)
kernel = np.ones((2, 2), np.uint8)
# r = cv2.dilate(r, kernel, iterations=1)

y_max = np.argmax(r, axis=0)

re = np.zeros((r.shape), dtype=np.uint8)
for x in range(r.shape[1]):
    if x == 0:
        re[y_max[x], x] = 255
    if (x > 0) and (abs(y_max[x] - y_max[x-1]) < 5):
        re[y_max[x], x] = 255
    else:
        continue

my_list = list(y_max)
counts = np.bincount(my_list)
max_index = np.argmax(counts)
# print(my_list[max_index])
print(max_index)

fig = plt.figure(figsize=(10, 7))
# setting values to rows and column variables
rows = 1
columns = 2

fig.add_subplot(rows, columns, 1)
plt.imshow(cv2.cvtColor(img_o, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.title("ROI")

fig.add_subplot(rows, columns, 2)
plt.imshow(re, cmap='gray')
plt.axis('off')
plt.title("DETECT")

plt.show()
