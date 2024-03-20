# coding:UTF-8
# Version: V1.5.1
import serial

buf_length = 11

RxBuff = [0]*buf_length

ACCData = [0.0]*8
GYROData = [0.0]*8
AngleData = [0.0]*8
FrameState = 0  # What is the state of the judgment
CheckSum = 0  # Sum check bit

start = 0 #帧头开始的标志
data_length = 0 #根据协议的文档长度为11 eg:55 51 31 FF 53 02 CD 07 12 0A 1B

acc = [0.0]*3
gyro = [0.0]*3
Angle = [0.0]*3

def GetDataDeal(list_buf):
    global acc,gyro,Angle

    if(list_buf[buf_length - 1] != CheckSum): #校验码不正确
        return
        
    if(list_buf[1] == 0x51): #加速度输出
        for i in range(6): 
            ACCData[i] = list_buf[2+i] #有效数据赋值
        acc = get_acc(ACCData)

    elif(list_buf[1] == 0x52): #角速度输出
        for i in range(6): 
            GYROData[i] = list_buf[2+i] #有效数据赋值
        gyro = get_gyro(GYROData)

    elif(list_buf[1] == 0x53): #姿态角度输出
        for i in range(6): 
            AngleData[i] = list_buf[2+i] #有效数据赋值
        Angle = get_angle(AngleData)

    print("acc:%10.3f %10.3f %10.3f \n" % (acc[0],acc[1],acc[2]))
    print("gyro:%10.3f %10.3f %10.3f \n" % (gyro[0],gyro[1],gyro[2]))
    print("angle:%10.3f %10.3f %10.3f \n" % (Angle[0],Angle[1],Angle[2]))

    
    

def DueData(inputdata):  # New core procedures, read the data partition, each read to the corresponding array 
    global start
    global CheckSum
    global data_length
    # print(type(inputdata))
    if inputdata == 0x55 and start == 0:
        start = 1
        data_length = 11
        CheckSum = 0
        #清0
        for i in range(11):
            RxBuff[i] = 0

    if start == 1:
        CheckSum += inputdata #校验码计算 会把校验位加上
        RxBuff[buf_length-data_length] = inputdata #保存数据
        data_length = data_length - 1 #长度减一
        if data_length == 0: #接收到完整的数据
            CheckSum = (CheckSum-inputdata) & 0xff 
            start = 0 #清0
            GetDataDeal(RxBuff)  #处理数据
        

def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]
    k_acc = 16.0
    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc
    return acc_x, acc_y, acc_z


def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return gyro_x, gyro_y, gyro_z


def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle
    return angle_x, angle_y, angle_z

if __name__ == '__main__':
    port = '/dev/ttyUSB0' # USB serial port linux
    #port = 'COM12' # USB serial port  windowns
    baud = 9600   # Same baud rate as the INERTIAL navigation module
    ser = serial.Serial(port, baud, timeout=0.5)
    print("Serial is Opened:", ser.is_open)
    while(1):
        RXdata = ser.read(1)#一个一个读
        RXdata = int(RXdata.hex(),16) #转成16进制显示
        DueData(RXdata)
        

