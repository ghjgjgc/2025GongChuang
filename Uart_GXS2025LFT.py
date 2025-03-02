import time
import serial
# 任务码
QR_code = 255
Raw_Material_Scanning = 1
Raw_Material_Correction = 5
UnStacking_Correction = 2
Stacking_Correction = 3
Angle_Correction = 4

Red = 0
Green = 1
Blue = 2

global_task = QR_code
global_color = Red


class Uart:
    def __init__(self, com="/dev/ttyUSB0"):
        # 实例化串口对象
        self.uart = serial.Serial(com,115200 , timeout=10)
        if self.uart.isOpen():
            print("uart is ready")

    def close(self):
        self.uart.close()

    def wait_for_data_packet(self, timeout=10):
        global global_task,global_color
        local_task = None
        local_color = None
        time_begin = time.time()
        time_end = time_begin + timeout
        buffer = bytearray()  
        while time.time() < time_end:  
          if self.uart.inWaiting() > 0:  
              buffer.extend(self.uart.read(self.uart.inWaiting()))  # 读取所有可用数据  
              while len(buffer) >= 4:  # 至少要有一个开头字节和三个数据字节  
                  if buffer[0] == 0xBB and buffer[3] == 0xCC:  
                      local_task = buffer[1]  # 提取task数据  
                      local_color = buffer[2]  # 提取color数据 
                      print(f"task data: {local_task}")  
                      print(f"color data: {local_color}")  
                      buffer = buffer[4:]  # 移除已处理的数据  
                      return local_task,local_color
                  else:  
                      buffer.pop(0)  # 移除第一个字节，继续检查
        if local_task is None or local_color is None:
            print("Timeout waiting for data packet.")
        return None,None    
        

    def uart_head(self,param_task,param1,param2,param3,param4,param5,param6):
        print(param_task,param1,param2,param3,param4,param5,param6)
        myinput=bytes([0xFF,param_task,param1,param2,param3,param4,param5,param6,0xAA])
        self.uart.write(myinput)

    def uart_send_order(self, order1, order2, order3, order4, order5, order6):
        self.uart_head(QR_code,order1, order2, order3, order4, order5, order6)

    def uart_send_yes(self,fn=0x03,color=0x03):
        global global_task
        self.uart_head(global_task,0,0,0,0,fn,color)

    def uart_send_command(self, param1, param2, param3, param4, fn=0x01,color=0):
        self.uart_head(1,param1, param2, param3, param4, fn,color)

    def uart_send_angle(self, BigAngle, SmallAngle,Direction ,fn=0x01):
        self.uart_head(Angle_Correction,BigAngle,SmallAngle, Direction, fn,0,0)

    def car_move_xy_cm(self, dx, dy, dx_threshold=8, dy_threshold=8, color=0, Magnification = 0.3):
        dx = int(dx)
        dy = int(dy)
        if abs(dy) > 255:
            dy = 255 * 1 if dy > 0 else -1
        if abs(dx) > 255:
            dx = 255 * 1 if dx > 0 else -1
        if abs(dx) < dx_threshold and abs(dy) < dy_threshold:
            dy=int(dy * Magnification)
            dx=int(dx * Magnification)
            self.uart_send_command(param1=abs(dy),param2=abs(dx), param3 = 0 if dy >= 0 else 1, param4=0 if dx >= 0 else 1, fn=0x01,color=color)
            return True
        else:
            dy=int(dy * Magnification)
            dx=int(dx * Magnification)
            self.uart_send_command(param1=abs(dy), param2=abs(dx), param3 = 0 if dy >= 0 else 1, param4=0 if dx >= 0 else 1,fn=0x01,color=color)
            return False

if __name__ == "__main__":
    uart = Uart()
    


