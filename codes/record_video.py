import cv2
import time

# Inicializar la cámara
cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Error al abrir la cámara")
    exit()

# Obtener el ancho y alto de los fotogramas de la cámara
frame_width = int(cam.get(3))
frame_height = int(cam.get(4))
print(frame_width,frame_height)
# Definir el codec y crear el objeto VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')
timestamp = time.strftime("%Y%m%d_%H%M%S")
out = cv2.VideoWriter(f'video_{timestamp}.avi', fourcc, 20.0, (frame_width, frame_height))

print("Presiona 'a' para terminar la grabación.")

img_counter = 0
 
try:
    while True:
        ret, frame = cam.read()

        if not ret:
            print("Error al capturar el video")
            break

        # Escribir el frame en el archivo de video
        out.write(frame)
        
        # The original input frame is shown in the window  
        #cv2.imshow('Original', frame) 
        
        # Wait for 'a' key to stop the program  
        if cv2.waitKey(1) & 0xFF == ord('a'): 
           break
    
finally:
    # Liberar la cámara y el escritor de video
    cam.release()
    out.release()
    #cv2.destroyAllWindows()
    print("Grabación terminada y recursos liberados correctamente.")
