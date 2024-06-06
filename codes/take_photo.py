import cv2
import time
import getch

cam = cv2.VideoCapture(0)

img_counter = 0

while True:
	ret,frame = cam.read()

	if not ret:
		print("error al tomar foto")
		break

	#cv2.imshow("test",frame)

	k = getch.getch()

	if k == '\x1b':
		print("Esc presionado, cerrando programa")
		break

	elif k == ' ':
		timestamp = time.strftime("%Y%m%d_%H%M%S") 
		img_name = "foto_{}_{}.png".format(img_counter, timestamp)
		cv2.imwrite(img_name, frame)
		print("Foto tomada")
		img_counter += 1
		# Espera un momento para evitar múltiples capturas con una sola pulsación de espacio
		time.sleep(0.5)

cam.release()

cam.destroyAllWindows()
