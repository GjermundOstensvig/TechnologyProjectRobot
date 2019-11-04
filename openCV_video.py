import cv2

fil = "vid.mp4"
video = cv2.VideoCapture(fil)
# video = cv2.VideoCapture(0)
# video.set(3, 640)
# video.set(4, 480)

if video.isOpened():
	w = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
	h = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(w, h)
w_senter = int(w / 2)
h_senter = int(h / 2)

def punktGenerator(punkter, verdier, omraade, retning, fra, til, intervall):
	punkter.append([])
	for i in range(fra, til+1):
		if retning == 0:
			punkter[0].append((w - (i * intervall)))
		else:
			punkter[0].append(i * intervall)
	for j in range(fra, til+1):
		if retning == 0:
			verdier.append(omraade[1, punkter[0][j - 1]]) #FRA ROW 1, COLUMN punkter[0][j - 1]
		else:
			verdier.append(omraade[punkter[0][j - 1], 1])
	return verdier

def punktGenerator(punkter, verdier, omraade, retning, antPunkter):
	punkter.append([])
	avstandMPunkter = int((w-antPunkter)/antPunkter)
	for i in range(1, antPunkter + 1):
		if retning == 0:
			punkter[0].append((w - avstandMPunkter))
		else:
			punkter[0].append((i+avstandMPunkter)*i)
	for j in range(1, antPunkter + 1):
		if retning == 0:
			verdier.append(omraade[1, punkter[0][j - 1]]) #FRA ROW 1, COLUMN punkter[0][j - 1]
		else:
			verdier.append(omraade[punkter[0][j - 1], 1])
	return verdier

def pixelate(bilde, px):
    inn = cv2.resize(bilde, (px, px), interpolation=cv2.INTER_LINEAR)
    out = cv2.resize(inn, (w, h), interpolation=cv2.INTER_NEAREST)
    return out

while True:

	ret, frame = video.read()
	if not ret:
		video = cv2.VideoCapture(fil)
		continue

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray, (3, 1), 0)
	ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
#	thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,21,11)

	mask = cv2.erode(thresh, None, iterations=5)
	mask = cv2.dilate(mask, None, iterations=5)

	main_tracker = thresh[h_senter:h_senter + 20, 1:w]

	contours_main_tracker, hierarchy = cv2.findContours(main_tracker.copy(), 1, cv2.CHAIN_APPROX_NONE)

	kryss = thresh[h_senter - 2:h_senter, 1:w]
	sensorKryssPunkter, sensorKryssVerdier = [], []
	#punktGenerator(sensorKryssPunkter, sensorKryssVerdier, kryss, 0, 1, 10, 5)  # 0 for horisontal
	punktGenerator(sensorKryssPunkter, sensorKryssVerdier, kryss, 0, 10)  # 0 for horisontal

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

	print(sensorKryssVerdier)

	if sum(sensorKryssVerdier) == len(sensorKryssVerdier) * 255:
		print("KRYSS!")

	key = cv2.waitKey(25)
	if key == 113:
		break

video.release()
cv2.destroyAllWindows()
