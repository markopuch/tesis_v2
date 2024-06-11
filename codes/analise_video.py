import cv2
import numpy as np
import time

def crop_mask(file):
    height = 880
    top=400
    polygons = np.array([[(250, height), (1500, height), (1200,top),(400, top)]])
    roi = np.zeros_like(file)
    cv2.fillPoly(roi, polygons, 255)
    mask_roi=cv2.bitwise_and(file, roi)
    return  mask_roi

def process(img,kernel=35,th=140,kernel2=7,iter=5):
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gauss_filter = cv2.GaussianBlur(gray,(kernel,kernel),5)
    crop=crop_mask(gauss_filter)
    _,bw = cv2.threshold(crop, th, 255, cv2.THRESH_BINARY)
    k=np.ones((kernel2, kernel2), np.uint8)
    erode = cv2.erode(bw, k, iterations=iter) 
    return erode

# Nombre del archivo de video
video_name = 'video_20240603_124132.avi'  # Cambia este nombre al nombre del archivo de tu video

# Abrir el archivo de video
cap = cv2.VideoCapture(video_name)

if not cap.isOpened():
    print("Error al abrir el archivo de video")
    exit()

# Obtener el frame rate del video
fps = cap.get(cv2.CAP_PROP_FPS)
wait_time = int(1000 / fps)

# Inicializar el contador de unos y ceros
count_ones = 0
count_zeros = 0

# Inicializar la variable de tiempo de inicio
start_time = None

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Fin del video")
        break

    # Mostrar el frame en una ventana
    cv2.imshow('Video', frame)
    
    # Procesar el frame
    processed_frame = process(frame)
    
    # Comprobar si el frame procesado es todo negro o tiene pixeles en blanco
    if cv2.countNonZero(processed_frame) > 0:
        count_ones += 1
        count_zeros = 0
        msg=True
        start_time = None
    else:
        if count_ones != 0:  # ajusta estos números según tus necesidades
            count_zeros += 1
            msg=True
            if start_time is None:
                start_time = time.time()   
            elif time.time() - start_time > 3:
                msg=False
                start_time = None
                count_ones = 0
                count_zeros = 0
        else:
            start_time = None
            msg=False
            count_zeros = 0
        
    
    # Mostrar el frame procesado en otra ventana
    cv2.imshow('Procesado', processed_frame)
    
    # Esperar según el frame rate del video
    if cv2.waitKey(wait_time) & 0xFF == ord('q'):
        break

# Liberar el objeto de captura y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()

