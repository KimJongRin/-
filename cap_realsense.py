import sys
import time
import datetime
import serial
import struct
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.register_read_message import ReadInputRegistersResponse
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from itertools import count
import pyrealsense2 as rs
import numpy as np
import cv2
import logging
import pymysql


sensor_stat=0
time1=0
time2=0

client=ModbusClient('rtu',port='/dev/ttyUSB0',stopbits=1,bytesize=8,parity='N',baudrate=9600,timeout=5)

conn = pymysql.connect(
    host='localhost',
    user='pi',
    password='irs5540',
    db='korea',
    charset='utf8'
)




dev_list=[1,25,26,27,28,29,30,31,32,33,34]


#동작시간 요청 및 시간정보 전송
recieve_info=[0x2b,0x07,0x51,0x08,0x07,0x46,0x3a]
send_info=[0x2b,0x07,0x52,0x01,0x0a,0x0f,0x45]# 기본 시간80min


#상단리미트 상단 리미트 스위치 도달. 사진 촬영 시작 정보
recieve_capture=[0x2b,0x04,0x53,0x7c]
send_capture=[0x2b,0x04,0x54,0x7b]


#상단리미트 상단 리미트 스위치 접지후 리프트를 내리는 시점에 전송(사진 촬영 불가)
recieve_stop=[0x2b,0x04,0x57,0x78]
send_stop=[0x2b,0x04,0x58,0x77]


#수질센서값 취득 명령. 하단 리미트 스위치 도달. 
recieve_sensor_start=[0x2b,0x04,0x5a,0x75]
send_sensor_start=[0x2b,0x04,0x5b,0x74]

# 수질센서값 취득 불가 명령. 하단리미트 접지상태에서 리프트가 상단으로 움직이는 시점
recieve_sensor_stop=[0x2b,0x04,0x5d,0x72]
send_sensor_stop=[0x2b,0x04,0x5e,0x71]

serial_port = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

serial_port.close()
serial_port.open()

def stop_capture() :
    print("capture stop....")
    serial_port.write(bytes(bytearray(send_stop)))

def start_sensor() :
    print("sensing starting....")
    serial_port.write(bytes(bytearray(send_sensor_start)))

def stop_sensor() :
    print("stop sensing.....")
    serial_port.write(bytes(bytearray(send_sensor_stop)))



def call_data() : 

    _count = 203
    i=0
    while True :
        cur = conn.cursor()

        if _count > 233 :
            break
        client=ModbusClient('rtu',port='/dev/ttyUSB0',stopbits=1,bytesize=8,parity='N',baudrate=9600,timeout=5)
        value =client.read_holding_registers(_count,2,unit=0x01)
        val = struct.unpack('f', struct.pack('HH', value.registers[0], value.registers[1]))[0]
        
        sql = "INSERT INTO shrimp_water_condition (observation_time,dev_code,value) VALUES (now(),%s, %s)"
        temp_val= round(val,3)

        cur.execute(sql, (dev_list[i] ,temp_val ))
        conn.commit()
        print("장비코드 : "+str(dev_list[i])+"센서값 : "+str(temp_val))
        i=i+1

        _count = _count+3

while True :
        
        
        if serial_port.readable():
            serial_port.timeout = None
            bytesToRead = serial_port.inWaiting()
            data=serial_port.read(bytesToRead)
            l_data=list(data)
            #data =serial_port.readline()
            
            #data =serial_port.read_until('',None)
            #정보요청
            if len(l_data) ==7 :
                print("Header : " , str.join("", ("0x%02X " % i for i in l_data)))
                sum = l_data[0]^l_data[1]^l_data[2]^l_data[3]^l_data[4]^l_data[5]
                CS = l_data[6]
                CS_SUM= sum^CS

                if CS_SUM == 0 :

                    now = datetime.datetime.now()    
                    now1 = now.strftime('%Y-%m-%d %H:%M:%S')
                                     
                    print("동작정보요청   "+now1)
                    send_info[6] = send_info[0]^send_info[1]^send_info[2]^send_info[3]^send_info[4]^send_info[5]
                    serial_port.write(bytes(bytearray(send_info)))

                else : 
                    pass

            #동작명령            
            if len(l_data) == 0x04 :
                print("Header : " , str.join("", ("0x%02X " % i for i in l_data)))
                
                if l_data[2] == 0x5a :
                    sensor_stat=0

                    if sensor_stat == 0 :
                        print("센싱....")
                        start_sensor()
                           
                        sum_time = time2 - time1
                            
                            #    if time1 == 0:
                            #         #global idx=0
                            #        time1 = time.time()
                        
                            #        for i in water_sensor : 
                            #             call_data(i)

                                

                            #    elif sum_time < 10:
                            #         print("저장안함")
                            #         time2 = time.time()
                            #    elif sum_time > 10:
                            #         print("저장한뒤 마지막시간 초기화")
                            #         time1 = time2
                            #         for i in water_sensor : 
                            #             call_data(i) 
                        print(sensor_stat)
                    else :
                        print("노센싱....")
                        print(sensor_stat)
                    
                #촬영시작
                if l_data[2] == 0x53:

                    now = datetime.datetime.now()    
                    now2 = now.strftime('%Y-%m-%d %H:%M:%S')
                                     
                    print("start_cap...."+now2)


                    serial_port.write(bytes(bytearray(send_capture)))
                    counter=13
                    pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_device('115222070761')
                    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
                    config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
                    
                    profile = pipeline.start(config)
                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                    ir_depth_frame = frames.get_depth_frame()
                    ir1_frame = frames.get_infrared_frame(1)
                    ir2_frame = frames.get_infrared_frame(2)
                    #time.sleep(1)
                    try:
                        while True:
                            timer = time.strftime('%Y%m%d_%H%M%S')

                            # Camera 2
                            # Wait for a coherent pair of frames: depth and color
                            frames = pipeline.wait_for_frames()
                            depth_frame = frames.get_depth_frame()
                            color_frame = frames.get_color_frame()
                            ir_depth_frame = frames.get_depth_frame()
                            ir1_frame = frames.get_infrared_frame(1)
                            ir2_frame = frames.get_infrared_frame(2)
                            #time.sleep(1)

                            if not depth_frame or not color_frame:
                                continue
                            # Convert images to numpy arrays

                        
                            depth_image = np.asanyarray(depth_frame.get_data())
                            color_image = np.asanyarray(color_frame.get_data())
                            ir_depth_image = np.asanyarray(ir_depth_frame.get_data())
                            ir1_image = np.asanyarray(ir1_frame.get_data())
                            ir2_image = np.asanyarray(ir2_frame.get_data())
                            

                            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                            depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_JET)
                            ir_depth_map = cv2.applyColorMap(cv2.convertScaleAbs(ir_depth_image, alpha=0.2), cv2.COLORMAP_JET)
                            # Stack all images horizontally
                            images1 = np.hstack((color_image,depth_colormap_2))
                            images2 = np.hstack((ir1_image,ir2_image))
                            # Stack all images horizontally
                            
                            path='/data/shrimp/realsense/'
                            cv2.imwrite(path+"IR_DEPTH/my_ir_depth"+timer+"_"+str(counter)+".png",ir_depth_map)
                            cv2.imwrite(path+"DEPTH/my_depth"+timer+"_"+str(counter)+".png",depth_colormap_2)
                            cv2.imwrite(path+"RGB/my_color"+timer+"_"+str(counter)+".png",color_image)
                            cv2.imwrite(path+"IR/my_ir"+timer+"_"+str(counter)+".png",ir2_image)
                            np.save(path+'IR_NPY/my_ir_npy'+timer+"_"+str(counter),ir_depth_image)
                            np.save(path+'DEPTH_NPY/my_depth_npy'+timer+"_"+str(counter), depth_image)
                        
                            print ("Save "+str(counter))
                            if counter == 60 :
                                counter=13
                                break
                            else :
                                counter=counter+1
                           # Show images from both cameras
                           # cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
                            #cv2.imshow('RealSense', images1)
                            #cv2.waitKey(1)

                    finally:

                        # Stop streaming
                        pipeline.stop()
                    sensor_stat=1
                #촬영정지
                if l_data[2] == 0x57:
                    stop_capture()
                    sensor_stat=1
                if l_data[2] == 0x5d:
                    stop_sensor()
                    sensor_stat=1
