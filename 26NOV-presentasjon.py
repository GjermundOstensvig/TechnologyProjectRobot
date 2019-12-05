from picamera.array import PiRGBArray
import cv2
import picamera
import numpy as np
import time
import easygopigo3 as easy
import os

# import distanceSensor as disSen

my_gopigo = easy.EasyGoPiGo3()
h = 192
w = 112
height = h
width = w
h_senter = int(h / 2)
w_senter = int(w / 2)

# CAMERA INIT
camera = picamera.PiCamera()
camera.resolution = (h, w)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(h, w))

# PID
integralArea = 0.0
previousError = 0.0
setPoint = 0.5
motorSpeed = 200
leftMotorSpeed = 0
rightMotorSpeed = 0
Kp = 160
Ki = 0
Kd = 80

'''Kp = 390
Ki = 0.0
Kd = 20'''
current = 0.5

time.sleep(0.1)

def punkt_generator_orig(punkter, verdier, omraade, retning, fra, til, intervall, startpunkt):
    punkter.append([])
    for i in range(fra, til + 1):
        if retning == 0:
            punkter[0].append((startpunkt - (i * intervall)))
        else:
            punkter[0].append(i * intervall)
    for j in range(fra, til + 1):
        if retning == 0:
            verdier.append(omraade[1, punkter[0][j - 1]])  # FRA ROW 1, COLUMN punkter[0][j - 1]
        else:
            verdier.append(omraade[punkter[0][j - 1], 1])
    return verdier

def punktGenerator(punkter, verdier, omraade, retning, fra, til, intervall, ytterkant):
	punkter.append([])
	for i in range(fra, til+1):
		if retning == 0:
			punkter[0].append((ytterkant - (i * intervall)))
		else:
			punkter[0].append(i * intervall)
	for j in range(fra, til+1):
		if retning == 0:
			verdier.append(omraade[1, punkter[0][j - 1]]) #FRA ROW 1, COLUMN punkter[0][j - 1]
		else:
			verdier.append(omraade[punkter[0][j - 1], 1])
	return verdier

# Measure temp
def temperature():
    temp = os.popen("vcgencmd measure_temp").readline()
    return (temp.replace("temp=", ""))

#camera.start_recording("h" + str(h) + "b" + str(w) + ".h264")
camera.start_recording("finalCode.h264")

# Loop over all frames captured by camera indefinitely
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # Display camera input
    image = frame.array
#    image_ = image_org[h-60:h-40, 1:w]

    # convert to grayscale, gaussian blur, and threshold
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    rev, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # thresh1 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)

    # Erode to eliminate noise, Dilate to restore eroded parts of image
    mask = cv2.erode(thresh, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    main_tracker = thresh[h_senter:h_senter+20, 1:width]
    _, contours_main_tracker, hierarchy = cv2.findContours(main_tracker.copy(), 1, cv2.CHAIN_APPROX_NONE)
    
    kryss = thresh[40:60, 1:w]  # lager et utsnitt
    sensorKryssPunkter, sensorKryssVerdier = [], []
    #punkter, verdier, område, horisontal/vertikal, fra, til, intervall
    punkt_generator_orig(sensorKryssPunkter, sensorKryssVerdier, kryss, 0, 1, 20, 4, width-1)  # 0 for horisontal

    # HOVEDTRACKER
    if len(contours_main_tracker) > 0:

        cxLive = 92
        center_of_contours = []
        for c in contours_main_tracker:
            if cv2.contourArea(c) < 200:
                contours_main_tracker.remove(c)
            else:
                moment = cv2.moments(c)
                if moment["m00"] != 0:
                    ccx = int(moment["m10"] / moment["m00"])
                    center_of_contours.append(ccx)
        if not center_of_contours:
            cx = cxLive
        else:
            cx = max(center_of_contours)
            cxLive = cx

        # Map input
        print(cx)
        current = cx / w

        #        print(str(current) + " - Temp: " + temperature())

        # calculate correction
        error = current - setPoint
        if Ki < 0.0001 and Ki > -0.0001:
            integralArea = 0.0
        else:
            integralArea += error

        correction = Kp * error + Ki * integralArea + Kd * (error - previousError)
        previousError = error

        # calculate motor speedss
        leftMotorSpeed = int(motorSpeed + correction)
        rightMotorSpeed = int(motorSpeed - correction)
        if leftMotorSpeed == 0: leftMotorSpeed = 1
        if rightMotorSpeed == 0: rightMotorSpeed = 1

        # update motor speeds
        my_gopigo.set_motor_dps(my_gopigo.MOTOR_LEFT, dps=leftMotorSpeed)
        my_gopigo.set_motor_dps(my_gopigo.MOTOR_RIGHT, dps=rightMotorSpeed)
        
        if sum(sensorKryssVerdier) == len(sensorKryssVerdier) * 255:
            time.sleep(2.2)
            my_gopigo.turn_degrees(90)
    rawCapture.truncate(0)
camera.stop_recording()
