import time
import math
import numpy as np
from rknn1 import *
import cv2
from Uart_GXS2025LFT import *
import threading

Red = 0
Green = 1
Blue = 2


# rknn_wk = rknn.rknn(r"/home/radxa/Desktop/demo_python/wk.rknn", 0.5, 0.5)
# rknn_sz = rknn.rknn(r"/home/radxa/Desktop/demo_python/sz.rknn", 0.5, 0.5)

def Uart_Threading(uart):
    global global_task, global_color
    while True:
        local_task, local_color = uart.wait_for_data_packet()
        with lock:
            global_task = local_task
            global_color = local_color
        cv2.waitKey(100)

class fundation:

    def qr_scan(self, frame):
        """
        :param frame: 传入图像
        :return: 二维码信息
        """
        qrcode = cv2.QRCodeDetector()  # 载入文件库
        QR, points, code = qrcode.detectAndDecode(frame)  # 对二维码进行解码，返回二维码的信息
        if code is None:  # 如果code返回值是none说明没有识别到二维码
            return None, None
        if code is not None:  # 如果code有返回值说明识别到二维码
            return [int(QR[0]), int(QR[1]), int(QR[2])], [int(QR[4]), int(QR[5]), int(QR[6])]

    def parallel(self, frame_parallel):
        img1 = cv2.cvtColor(frame_parallel, cv2.COLOR_RGB2GRAY)  # 进rgb图
        frame_parallel_candy = cv2.Canny(img1, 100, 200)
        lines = cv2.HoughLinesP(frame_parallel_candy, 1, np.pi / 180, 140, minLineLength=150, maxLineGap=800)
        if lines is not None:
            x1, y1, x2, y2 = lines[0][0]
            # 画线自己看
            # cv2.line(frame_parallel, (x1, y1), (x2, y2), (255, 0, 255), 2)
            # cv2.imshow('frame_parallel_line', frame_parallel)
            dx = x1 - x2
            dy = y1 - y2
            # 计算arctan值
            arctan = math.atan(dy / dx) * 180 / math.pi
            # x是整数部分， y是小数部分
            x = int(arctan)
            y = int(arctan * 100) - int(arctan) * 100
            return x, y
        else:
            return -128, -128

    def cam_check(self, n, cam_index, set):
        """
        开启并检查摄像头
        :param n: 摄像头序号
        :param cam_index: 硬件中摄像头的序号
        :param set: 是否改格式
        :return: 摄像头
        """
        global flagcam2, flagcam1
        cam = cv2.VideoCapture(cam_index)
        if set:
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if n == 2:
            cam.set(3, width)
            cam.set(4, height)
        if cam.isOpened():
            print(f"cam{n} ready2")
            if n == 1:
                flagcam1 = True
            elif n == 2:
                flagcam2 = True

        return cam

    def img_read(self, cam):
        global ret, img
        while True:
            local_ret, local_img = cam.read()
            with lock:
                ret = local_ret
                img = local_img

    def find_center(self, boxs):
        left, top, right, bottom = boxs
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom)
        centerx = (float)(left + right) / 2
        centery = (float)(top + bottom) / 2
        dx = centerx - 320
        dy = centery - 320
        return dx, dy, centerx, centery

    def qr_obtain(self):
        # 相机1初始化
        cam1 = self.cam_check(1, 2, False)
        while True:
            ret, img = cam1.read() #read会返回是否读取到ret以及图像数列img
            if ret is False or img is None: #if not 用于判断其是否不成立（是否为False）
                continue
            image = cv2.resize(img, (width, height), interpolation=cv2.INTER_LINEAR)
            order1, order2 = self.qr_scan(image)
            if order1 is not None:
                print(order1)
                print(order2)
                #展示数字
                #cv2.putText(imgnumber, "".join(map(str, order1)) + "+" + "".join(map(str, order2)),
                #            (50, 400), 3, 11, (255, 255, 255), 5)
                #cv2.imshow("imgnumber", imgnumber)
                #发送任务数字四次
                uart.uart_send_order(order1[0], order1[1], order1[2], order2[0], order2[1], order2[2])
                uart.uart_send_order(order1[0], order1[1], order1[2], order2[0], order2[1], order2[2])
                uart.uart_send_order(order1[0], order1[1], order1[2], order2[0], order2[1], order2[2])
                uart.uart_send_order(order1[0], order1[1], order1[2], order2[0], order2[1], order2[2])
                cv2.waitKey(3)
                break
            #cv2.imshow("img", img)
            # cv2.waitKey(1)
        cam1.release()

class Eyes_task(fundation):

    def task1(self):
        """原料区夹取"""
        # 变量创建
        global img, ret, cam_process,global_task
        preX, preY = 0, 0
        Movement_threshold = 5 #移动的阈值
        dx_threshold = 50
        dy_threshold = 300
        # 若任务为1，即未发送停止指令
        while global_task == Raw_Material_Scanning:
            boxs, scores, classes = rknn1.rknn_detect(img)
            if not ret or scores is None:
                continue
            if scores > 0.80:
                dx, dy, centerx, centery = self.find_center(boxs)
                centerx = int(centerx)
                centery = int(centery)
                if (abs(centerx - preX) + abs(centery - preY)) < Movement_threshold and abs(dx) <dx_threshold and abs(dy)<dy_threshold:
                    uart.uart_send_yes(color=classes) # 若满足直接发送每次的颜色与允许夹取
                cv2.waitKey(100) # 每0.1s 检测一次，即两次之间的距离
                preX = centerx
                preY = centery

    def task2(self):
        """粗调细调"""
        global img, ret, cam_process, global_task , global_color
        if global_color == Green:
            global_color = Blue
        elif global_color == Blue:
            global_color = Green
        # 开辟变量
        flagcount = 0
        dx1_threshold = 10
        dy1_threshold = 10
        dx2_threshold = 2
        dy2_threshold = 2
        # 第一次纠正
        while True:
            boxs, scores, classes = rknn2.rknn_detect(img, global_color)
            if ret is False or img is None or scores is None:
                continue
            if scores > 0.50 and classes == global_color:
                dx, dy, centerx, centery = self.find_center(boxs)
                flagxy = uart.car_move_xy_cm(dx, dy, dx1_threshold, dy1_threshold)
                if flagxy: # 如果达到临界值,发送五次最后参数？
                    flagcount += 1
                    cv2.waitKey(200)
                else:
                    flagcount=0
                if flagcount >= 3:
                    uart.uart_send_yes()
                    uart.uart_send_yes()
                    uart.uart_send_yes()
                    flagcount = 0
                    break
        #等待机械臂移动
        cv2.waitKey(300)
        # 第二次细纠
        while True:
            boxs, scores, classes = rknn2.rknn_detect(img, global_color)
            if ret is False or img is None or scores is None:
                continue
            if scores > 0.50 and classes == global_color:
                dx, dy, centerx, centery = self.find_center(boxs)
                flagxy = uart.car_move_xy_cm(dx, dy, dx2_threshold, dy2_threshold)
                if flagxy:  # 如果达到临界值,发送五次最后参数？
                    flagcount += 1
                    cv2.waitKey(200)
                else:
                    flagcount = 0
                if flagcount >= 3:
                    uart.uart_send_yes()
                    uart.uart_send_yes()
                    uart.uart_send_yes()
                    break

    def task3(self,):
        """暂存区堆叠用 """
        global img, ret, cam_process, global_task,global_color
        # 变量
        flagcount = 0
        dx1_threshold = 10
        dy1_threshold = 10
        # 第一次纠正
        while True:
            boxs, scores, classes = rknn2.rknn_detect(img, global_color)
            if ret is False or img is None or scores is None:
                continue
            if scores > 0.50 and classes == global_color:
                dx, dy, centerx, centery = self.find_center(boxs)
                flagxy = uart.car_move_xy_cm(dx, dy, dx1_threshold, dy1_threshold)
                if flagxy:  # 如果达到临界值,发送五次最后参数？
                    flagcount += 1
                    cv2.waitKey(200)
                else:
                    flagcount = 0
                if flagcount >= 3:
                    uart.uart_send_yes()
                    uart.uart_send_yes()
                    uart.uart_send_yes()
                    break

    def task4(self):
        """原料区误差纠正"""
        global img, ret, cam_process, global_task,global_color
        # 变量
        Distance_threshold = 30
        dx1_threshold = 10
        dy1_threshold = 10
        preX = 0
        preY = 0
        centerx = 0
        centery = 0
        flagcount = 0
        # 原料区纠正
        while True:
            boxs, scores, classes = rknn1.rknn_detect(img)
            if not ret or scores is None:
                continue
            if scores > 0.78: # 若识别到一个颜色
                dx, dy, centerx, centery = self.find_center(boxs)
                if math.sqrt(math.pow(preX - centerx, 2) + math.pow(preY - centery, 2)) > Distance_threshold: # 直线距离大于一定值，即移动了
                    flagcount = 0
                    continue
                flagxy = uart.car_move_xy_cm(dx, dy, dx1_threshold, dy1_threshold, color=classes)
                if flagxy:
                    flagcount += 1
                else:
                    flagcount = 0
                if flagcount >= 3:
                    break
            cv2.waitKey(100)  # 间隔时间判断
            preX = centerx
            preY = centery

    def task5(self):
        """角度修正"""
        global img, ret, cam_process, global_task,global_color
        flagcount = 0
        while True:
            if not ret or img is None or global_color==0: # 0 代表32忙中，还没完成移动
                continue
            X, Y = self.parallel(img)
            if X != -128 and Y != -128: # 康的代码中只是将数字扩大100放入，理论上正负相同 且actan在-90到90之间，没有溢出问题
                if X == 0 and Y == 0:
                    flagcount += 1
                else:
                    uart.uart_send_angle(abs(X), abs(Y), 0 if Y > 0 else 1, 0x01)
                    flagcount = 0
                if flagcount >= 3:
                    uart.uart_send_angle(0x00, 0x00, 0x00, 0x03)
                    break
        global_task = Raw_Material_Scanning

if __name__ == "__main__":


    # 初始化rknn
    rknn1 = rknn("wk.rknn", 0.6, 0.6)
    rknn2 = rknn("rknn_sz1.rknn", 0.6, 0.6)

    width, height = 640, 480
    imgnumber = np.zeros(
        (600, 1700), dtype=np.uint8)

    # 全局变量开辟
    ret = False
    img = None
    tasking = False
    cam_process = False
    
    # 解析引用实例（类似于开个对象）
    Cam2_Do = Eyes_task()
    uart = Uart()

    # 串口接收线程
    global_task = 0
    global_color = 0
    lock = threading.Lock()
    uart_receive_threading = threading.Thread(target=Uart_Threading, args=(uart,), name="uart_receive_threading")
    uart_receive_threading.daemon = True
    uart_receive_threading.start()

    # 摄像头线程
    cam2 = Cam2_Do.cam_check(2, 0, True)
    cam_threading = threading.Thread(target=Cam2_Do.img_read, args=(cam2,), name="cam_threading")
    cam_threading.start()

    # 二维码扫描
    Cam2_Do.qr_obtain()

    while True:
        if global_task == Raw_Material_Scanning:
            Cam2_Do.task1()
        elif global_task == UnStacking_Correction:
            Cam2_Do.task2()
        elif global_task == Stacking_Correction:
            Cam2_Do.task3()
        elif global_task == Angle_Correction:
            Cam2_Do.task4()
        elif global_task == Raw_Material_Correction:
            Cam2_Do.task5()
        else:
            time.sleep(0.3)
