import time
import serial
# 任务码
QR_code = -1
Raw_Material_Scanning = 1
Raw_Material_Correction = 5
UnStacking_Correction = 2
Stacking_Correction = 3
Angle_Correction = 4

class Uart:
    def __init__(self, com="/dev/ttyUSB0"):
        # 实例化串口对象
        self.uart = serial.Serial(com,115200 , timeout=10)
        if self.uart.isOpen():
            print("uart is ready")

    def close(self):
        self.uart.close()

    def wait_for_data_packet(self, timeout=10):
        time_begin = time.time()
        time_end = time_begin + timeout
        task = None
        color = None
        buffer = bytearray()  
        while time.time() < time_end:  
          if self.uart.inWaiting() > 0:  
              buffer.extend(self.uart.read(self.uart.inWaiting()))  # 读取所有可用数据  
              while len(buffer) >= 4:  # 至少要有一个开头字节和三个数据字节  
                  if buffer[0] == 0xBB and buffer[3] == 0xCC:  
                      task = buffer[1]  # 提取task数据  
                      color = buffer[2]  # 提取color数据  
                      print(f"task data: {task}")  
                      print(f"color data: {color}")  
                      buffer = buffer[4:]  # 移除已处理的数据  
                      break  
                  else:  
                      buffer.pop(0)  # 移除第一个字节，继续检查
        if task is None and color is None:
            print("Timeout waiting for data packet.")
        return task, color  # 超时返回None, None

    def uart_head(self,param_task,param1,param2,param3,param4,param5,param6):
        myinput=bytes([0xFF,param_task,param1,param2,param3,param4,param5,param6,0xAA])
        self.uart.write(myinput)

    def uart_send_order(self, order1, order2, order3, order4, order5, order6):
        self.uart_head(QR_code,order1, order2, order3, order4, order5, order6)

    def uart_send_yes(self,fn=0x03,color=0):
        self.uart_head(1,0x03,0x03,0x03,0x03,fn,color)

    def uart_send_command(self, param1, param2, param3, param4, fn=0x01,color=0):
        self.uart_head(1,param1, param2, param3, param4, fn,color)

    def uart_send_angle(self, BigAngle, SmallAngle,Direction ,fn=0x01):
        self.uart_head(Angle_Correction,BigAngle,SmallAngle, Direction, fn,0,0)

    def car_move_xy_cm(self, dx, dy, dx_threshold=8, dy_threshold=8, color=0):
        dx = int(0.5 * dx)
        dy = int(0.5 * dy)
        if abs(dy) > 255:
            dy = 255 * 1 if dy >= 0 else -1
        if abs(dx) > 255:
            dx = 255 * 1 if dx >= 0 else -1
        if abs(dx) < dx and abs(dy) < dy:
            self.uart_send_command(param1=dx, param2=dy, param3 = 0 if dx >= 0 else 1, param4=0 if dy >= 0 else 1, fn=0x03,color=color)
            return True
        else:
            self.uart_send_command(param1=dx, param2=dy, param3 = 0 if dx >= 0 else 1, param4=0 if dy >= 0 else 1,fn=0x01,color=color)
            return False

if __name__ == "__main__":
    uart = Uart()
    


