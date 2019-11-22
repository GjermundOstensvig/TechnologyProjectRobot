from picamera.array import PiRGBArray
import cv2
import picamera
import time
import easygopigo3

gopigo = easygopigo3.EasyGoPiGo3()

# CAMERA SETUP
w = 192
h = 112

cam = picamera.PiCamera()
cam.resolution = (w, h)
cam.framerate = 20
rawCapture = PiRGBArray(cam, size=(w, h))

w_center = int(w / 2)
h_center = int(h / 2)

# PID CONTROLLER SETUP
integralArea = 0.0
previousError = 0.0
setPoint = 0.5
motorSpeed = 200
leftMotorSpeed = 0
rightMotorSpeed = 0
Kp = 160
Ki = 0
Kd = 80
current = 0.5

time.sleep(0.1)

def valuePointGenerator(points, values, area, rangeFrom, rangeTo, interval, startPoint):
    points.append([])
    rangeTo += 1

    for i in range(rangeFrom, rangeTo):
        points[0].append((startPoint - (i * interval)))

    for j in range(rangeFrom, rangeTo):
        values.append(area[1, points[0][j -1]])

    return values

for frame in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    # GET FRAME FROM CAMERA INPUT
    image = frame.array

    # CONVERT TO GRAYSCALE, APPLY GAUSSIAN BLUR AND THRESHOLD TO CREATE A BINARY IMAGE
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # ELIMINATE NOISE FROM CAMERA INPUT
    mask = cv2.erode(thresh, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # CROP OUT A SECTION OF THE FRAME TO BE USED BY THE LINE TRACKER, imageSource[fromRow:toRow, fromColumn:toColumn]
    tracker = thresh[h_center:h_center+20, 1:w]

    # FIND CONTOURS OF TRACK
    _, contours_tracker, hierarchy = cv2.findContours(tracker.copy(), 1, cv2.CHAIN_APPROX_NONE)

    # CROP OUT A SECTION OF THE FRAME USED TO DETECT AN INTERSECTION
    intersection = thresh[40:43, 1:w]

    # GENERATE AN ARRAY FOR CONTINUOUS MEASURING OF SAID POINT VALUES
    intersectionPoints, intersectionValues = [], []
    valuePointGenerator(intersectionPoints, intersectionValues, intersection, 1, 20, 4, (w - 1))

    # LINE TRACKER
    if len(contours_tracker) > 0:

        centerOfContours = []
        cxLive = w_center

        for c in contours_tracker:

            if cv2.contourArea(c) < 200:
                contours_tracker.remove(c)

            else:
                moment = cv2.moments(c)

                if moment["m00"] != 0:
                    ccx = int(moment["m10"] / moment["m00"])
                    centerOfContours.append(ccx)

        if not centerOfContours:
            cx = cxLive

        else:
            cx = max(centerOfContours)
            cxLive = cx

        # MAP INPUT
        current = cx / w
        error = current - setPoint

        if Ki < 0.0001 and Ki > -0.0001:
            integralArea = 0.0

        else:
            integralArea += error

        correction = Kp * error + Ki * integralArea + Kd * (error - previousError)
        previousError = error

        # CALCULATE MOTOR SPEEDS
        leftMotorSpeed = int(motorSpeed + correction)
        rightMotorSpeed = int(motorSpeed - correction)
        if leftMotorSpeed == 0: leftMotorSpeed = 1
        if rightMotorSpeed == 0: rightMotorSpeed = 1

        # UPDATE MOTOR SPEEDS
        gopigo.set_motor_dps(gopigo.MOTOR_LEFT, dps=leftMotorSpeed)
        gopigo.set_motor_dps(gopigo.MOTOR_RIGHT, dps=rightMotorSpeed)

        # IF RIGHT TURN INTERSECTION IS DETECTED, TURN 90 DEGREES RIGHT
        if sum(intersectionValues) == len(intersectionValues) * 255:
            time.sleep(2.2)
            gopigo.turn_degrees(90)

    rawCapture.truncate(0)
