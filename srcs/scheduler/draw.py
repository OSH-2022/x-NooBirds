from cmath import pi
import math
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt

r1 = 2000
r2 = 2000
r3 = 2000

s1 = math.pi*r1/100.0*r1/100.0
s2 = math.pi*r2/100.0*r2/100.0
s3 = math.pi*r3/100.0*r3/100.0


def check_charset(file_path):
    import chardet
    with open(file_path, "rb") as f:
        data = f.read(4)
        charset = chardet.detect(data)['encoding']
    return charset

def tfloat(string):
    if string[0] == '-':
        string = string[1:]
        string.lstrip()
        return -float(string)
    else:
        return float(string)

plt.figure(1)
ax = plt.subplot(111, aspect = 'equal')
x = [1.0, 2.0, 3.0]
y = [1.0, 2.0, 3.0]

line, = ax.plot(x, y, 'ro')

plt.xlim(0, 50)
plt.ylim(0, 50)

my_x_ticks = np.arange(0, 50, 5)

my_y_ticks = np.arange(0, 50, 5)
plt.xticks(my_x_ticks)
plt.yticks(my_y_ticks)

plt.ion()
plt.show()

qwq = ""

cnt = 0

with open('.\\out.txt', encoding=check_charset('.\\out.txt')) as f:
    for lines in f.readlines():
        try:
            qwq = lines[:-2].split(' ')
        except Exception:
            pass
        # if cnt == 120:
        #     time.sleep(3)
        #     cnt = 0
        # else:
        #     cnt = cnt + 1
        # print(cnt)
        ax.clear()
        ax = plt.subplot(111, aspect = 'equal')
        plt.xlim(-20, 50)
        plt.ylim(-20, 50)
        my_x_ticks = np.arange(-20, 50, 5)
        my_y_ticks = np.arange(-20, 50, 5)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        x1 = (tfloat(qwq[0])+1200.0)/100.0
        x2 = (tfloat(qwq[2])+1200.0)/100.0
        x3 = (tfloat(qwq[4])+1200.0)/100.0
        y1 = (tfloat(qwq[1])+1200.0)/100.0
        y2 = (tfloat(qwq[3])+1200.0)/100.0
        y3 = (tfloat(qwq[5])+1200.0)/100.0

        ax.scatter(x1, y1, s=s1, c='wheat', marker='o')
        ax.scatter(x2, y2, s=s2, c='wheat', marker='o')
        ax.scatter(x3, y3, s=s3, c='wheat', marker='o')
        ax.scatter(x1, y1, c='red', marker='*')
        ax.scatter(x2, y2, c='green', marker='s')
        ax.scatter(x3, y3, c='blue', marker='o')
        # x = [(tfloat(qwq[0])+1200.0)/100.0, (tfloat(qwq[2])+1200.0)/100.0, (tfloat(qwq[4])+1200.0)/100.0]
        # y = [(tfloat(qwq[1])+1200.0)/100.0, (tfloat(qwq[3])+1200.0)/100.0, (tfloat(qwq[5])+1200.0)/100.0]
        # line, = ax.plot(x, y, 'ro')
        ax.text(x1, y1, 0)
        ax.text(x2, y2, 1)
        ax.text(x3, y3, 2)
        plt.show()
        plt.pause(float(1/360))
        if cnt == 0:
            plt.pause(2)
            cnt = cnt + 1
        x.clear()
        y.clear()
