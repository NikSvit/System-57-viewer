#!/usr/bin/env python3

import sys
import time


# Import mavutil
from pymavlink import mavutil
from tkinter import *

# The tkinter window object
global window

# Create the connection
# drone = mavutil.mavlink_connection('/dev/ttyUSB0')
# drone = mavutil.mavlink_connection('udpin:192.168.0.51:14445')
drone = mavutil.mavlink_connection('udpin:127.0.0.1:14445')
# # drone = mavutil.mavlink_connection('udpin:127.0.0.1:14445',source_system=100,
# #                                    source_component=191)
drone.wait_heartbeat()
time.sleep(2)
# # Wait a heartbeat before sending commands
# drone.wait_heartbeat()
# mavlink_message = drone.recv_match(type='HEARTBEAT', blocking=True)
# print("Heartbeat ", 'System ', '%3d' % (mavlink_message.get_srcSystem()), '%3d' % (mavlink_message.get_srcComponent()))
# # target_system = drone.target_system
# # target_component = drone.target_component

# target_system = 57
# target_component = 191

i = 0
HeartbeatPX4Timer = time.time()
HeartbeatCompTimer = time.time()


def changeState():
    # обработка сигналов MAVLink
    global HeartbeatPX4Timer 
    global HeartbeatCompTimer 

    switchGPS = {
            0: "No GPS connected",
            1: "No position information, GPS is connected",
            2: "2D position",
            3: "3D position",
            4: "DGPS/SBAS aided 3D position",
            5: "RTK float, 3D position",
            6: "RTK Fixed, 3D position",
            7: "Static fixed, typically used for base stations",
            8: "PPP, 3D position"
        }

    # try:
    #     mass = drone.recv_match().to_dict()
    #     if mass is not None:
    #         print (mass)
    # finally:
    mavlink_message = drone.recv_match(type = 'HEARTBEAT', blocking = True )

    
    
    
    if mavlink_message is not None:
        # поиск сигнала автопилота
        print (mavlink_message.get_srcComponent())
        if mavlink_message.get_srcComponent() == 1:
            HeartbeatPX4Label.configure(text = 'Heartbeat PX4 - Ok', background = "#d0fecc")
            HeartbeatPX4Timer = time.time()
        else: 
            if HeartbeatPX4Timer+5 < time.time():
                HeartbeatPX4Label.configure(text = 'Heartbeat PX4 - Lost', background = "#fedad9")
                    
        # поиск сигнала компаньона
        if mavlink_message.get_srcComponent() == 191:
            HeartbeatCompLabel.configure(text = 'Heartbeat companion - Ok',background="#d0fecc")
            HeartbeatCompTimer =time.time()
            if (mavlink_message.custom_mode % 10000)//1000 == 1:
                FotoLabel.configure(text = 'Camera activation - Ok',background="#d0fecc")
            else:
                FotoLabel.configure(text = 'Camera activation - Error',background="#fedad9")
            if (mavlink_message.custom_mode % 100000)//10000 == 1:
                TimeSynLabel.configure(text = 'Time synchronization - Ok',background="#d0fecc")
            else:
                TimeSynLabel.configure(text = 'Time synchronization - Error',background="#fedad9")
        else: 
            if HeartbeatCompTimer+3 < time.time():
                HeartbeatCompLabel.configure(text = 'Heartbeat companion - Lost', background="#fedad9")
                
    # состояние СРНС приемника
    mavlink_message = drone.recv_match(type='GPS_RAW_INT', blocking=True, timeout = 0.1)
    if mavlink_message is not None:
        GPSFixTypeLabel.configure(text = 'GPS status - '+switchGPS[mavlink_message.fix_type])
        print(mavlink_message)
        if mavlink_message.fix_type > 1:
            GPSFixTypeLabel.configure(background="#d0fecc")
            mavlink_message = drone.recv_match(type='UTM_GLOBAL_POSITION', blocking=True,  timeout = 0.1)
            if mavlink_message is not None:
                GPSTimeLabel.configure(text = 'GPS time - '+time.strftime("%m/%d/%Y, %H:%M:%S",time.gmtime(mavlink_message.time/1000000)))
        else:
            GPSFixTypeLabel.configure(background="#fedad9")

            #  время СРНС приемника

        
        # print (time.gmtime(mavlink_message.time/1000000))

    frame.after(1, changeState)


window = Tk()
window.wm_title(" Prefly control ")
window.geometry("300x200+200+300")
frame = Frame(window) 

HeartbeatPX4Label = Label(frame, width=100) 
HeartbeatPX4Label.pack(anchor=W)
# HeartbeatPX4Label.after(10, changeStatePX4)

HeartbeatCompLabel = Label(frame, width=100) 
HeartbeatCompLabel.pack(anchor=W)

GPSFixTypeLabel = Label(frame, width=100) 
GPSFixTypeLabel.pack(anchor=W)

GPSTimeLabel = Label(frame, width=100) 
GPSTimeLabel.pack(anchor=W)

FotoLabel = Label(frame, width=100) 
FotoLabel.pack(anchor=W)

TimeSynLabel = Label(frame, width=100) 
TimeSynLabel.pack(anchor=W)

# HeartbeatCompLabel.after(10, changeState)
frame.after(1,changeState)

# 
# print("Heartbeat ", 'System ', '%3d' % (mavlink_message.get_srcSystem()), '%3d' % (mavlink_message.get_srcComponent()))
# if mavlink_message.get_srcComponent() == 1:
#     HeartbeatLabel.configure(text = 'Heartbeat PX4 - Ok')
# else: 
#     HeartbeatLabel.configure(text = 'Heartbeat PX4 - Lost')
# HeartbeatLabel.grid(column=0, row=0)

# locationLabel = Label(frame, width=60)
# locationLabel.pack()
# i = i+1
# print (i)
# locationLabel.after(1000, changeState)
frame.pack()
# attitudeLabel = Label(frame, text = "No Att", width=60)
# attitudeLabel.pack()
# modeLabel = Label(frame, text = "mode")
# modeLabel.pack()
# Button(frame, text = "Auto" ).pack()

window.mainloop()

# i=0
# while True:
    
#     i=i+1
#     m = drone.recv_match(type='HEARTBEAT', blocking=True)
#     locationLabel = Label(frame, text = "HEARTBEAT", width=60)
#     if m.get_srcComponent() == target_component:
#         print("Heartbeat ", 'System ', '%3d' % (m.get_srcSystem()),' component ','%3d' % (m.get_srcComponent()),
#         ' mode ',m.custom_mode)
#     m = drone.recv_match(type='GPS_RAW_INT', blocking=True)
#     print("GPS type ",'%3d' % (m.fix_type),
#      ' GPS time ', time.gmtime(m.time_usec))
#     time.sleep(3)
#     # drone.mav.command_long_send(
#     # target_system,
#     # target_component,
#     # mavutil.mavlink.MAV_CMD_USER_1,#Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values
#     # 0,  # Confirmation
#     # 1, 
#     # 2,  
#     # 3, 
#     # 4, 
#     # 5, 
#     # 6,
#     # 7)
# window.mainloop()

# drone.mav.command_long_send(
# drone.target_system,
# drone.target_component,
# mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
# 0,
# 0, 1, 0, 0, 0, 0, 0, 0)

# drone.mav.command_long_send(
#     drone.target_system,
#     drone.target_component,
#     mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,#Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values
#     0, # Confirmation
#     0,  # Reserved (Set to 0)
#     0,  # Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).
#     1,  # Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.
#     0,  # Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.
#     0, 0, 0, 0)


    # drone.mav.command_long_send(
    #     target_system,
    #     target_component,
    #     mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,#SControl digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files 
    #     0,  # Confirmation
    #     0,  # Session control e.g. show/hide lens
    #     0,  # Zoom's absolute position
    #     0,  # Zooming step value to offset zoom from the current position
    #     0,  # Focus Locking, Unlocking or Re-locking
    #     1,  # Shooting Command
    #     0,  # Command Identity
    #     0  # Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.
    #     )
    # print(i, "  Send MAV_CMD_DO_DIGICAM_CONTROL to system",'%3d' % (target_system)," conponent ", '%3d' % (target_component))
print("END")
