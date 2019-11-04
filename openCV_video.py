import cv2

fil = "vid.mp4"
video = cv2.VideoCapture(fil)

if video.isOpened():
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(width, height)
w_senter = int(width / 2)
h_senter = int(height / 2)


def punkt_generator(punkter, verdier, omraade, retning, fra, til, intervall):
    punkter.append([])
    for i in range(fra, til + 1):
        if retning == 0:
            punkter[0].append((width - (i * intervall)))
        else:
            punkter[0].append(i * intervall)
    for j in range(fra, til + 1):
        if retning == 0:
            verdier.append(omraade[1, punkter[0][j - 1]])  # FRA ROW 1, COLUMN punkter[0][j - 1]
        else:
            verdier.append(omraade[punkter[0][j - 1], 1])
    return verdier


def punkt_generator(punkter, verdier, omraade, retning, antPunkter):
    punkter.append([])
    avstandMPunkter = int((width - antPunkter) / antPunkter)
    for i in range(1, antPunkter + 1):
        if retning == 0:
            punkter[0].append((width - avstandMPunkter))
        else:
            punkter[0].append((i + avstandMPunkter) * i)
    for j in range(1, antPunkter + 1):
        if retning == 0:
            verdier.append(omraade[1, punkter[0][j - 1]])  # FRA ROW 1, COLUMN punkter[0][j - 1]
        else:
            verdier.append(omraade[punkter[0][j - 1], 1])
    return verdier


def pixelate(bilde, px):
    inn = cv2.resize(bilde, (px, px), interpolation=cv2.INTER_LINEAR)
    out = cv2.resize(inn, (width, height), interpolation=cv2.INTER_NEAREST)
    return out


while True:

    ret, frame = video.read()
    if not ret:
        video = cv2.VideoCapture(fil)
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 1), 0)
    ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,21,11)

    mask = cv2.erode(thresh, None, iterations=5)
    mask = cv2.dilate(mask, None, iterations=5)

    main_tracker = thresh[h_senter:h_senter + 20, 1:width]

    contours_main_tracker, hierarchy = cv2.findContours(main_tracker.copy(), 1, cv2.CHAIN_APPROX_NONE)

    kryss = thresh[h_senter - 2:h_senter, 1:width]
    sensor_kryss_punkter, sensor_kryss_verdier = [], []
    # punkt_generator(sensor_kryss_punkter, sensor_kryss_verdier, kryss, 0, 1, 10, 5)  # 0 for horisontal
    punkt_generator(sensor_kryss_punkter, sensor_kryss_verdier, kryss, 0, 10)  # 0 for horisontal

    if len(contours_main_tracker) > 0:

        c = max(contours_main_tracker, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cxLive = cx
        #	cy = int(M["m10"]/M["m00"])

        # SETTER CX = 180 NÅR VIEWPORT GÅR UTENFOR REKKEVIDDE TIL HØYRE
        elif cxLive > 150 and M["m00"] == 0:
            cx = 180
        else:
            cx = 1

        cv2.circle(main_tracker, (cx, 10), 2, (0, 255, 0), 2)
        print(cx)

    cv2.imshow("Frame", frame)
    cv2.imshow("Tracker", main_tracker)

    print(sensor_kryss_verdier)

    if sum(sensor_kryss_verdier) == len(sensor_kryss_verdier) * 255:
        print("KRYSS!")

    key = cv2.waitKey(25)
    if key == 113:
        break

video.release()
cv2.destroyAllWindows()
