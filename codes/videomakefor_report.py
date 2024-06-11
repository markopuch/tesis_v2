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

def rectang(img):
    height = 880
    top=400
    polygons = np.array([[(250, height), (1500, height), (1200,top),(400, top)]])
    n=img.copy()
    square=cv2.polylines(n, [polygons], isClosed=True, color=(255, 0, 0), thickness=5)
    return square

def process(img,kernel=35,th=140,kernel2=7,iter=5):
    
    square=rectang(img)
    
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gauss_filter = cv2.GaussianBlur(gray,(kernel,kernel),6)
    
    crop=crop_mask(gauss_filter)
    _,bw = cv2.threshold(crop, th, 255, cv2.THRESH_BINARY)
    
    k=np.ones((kernel2, kernel2), np.uint8)
    erode = cv2.erode(bw, k, iterations=iter)
    
    return img,square,crop,erode

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

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(frame_width,frame_height)

# Crear un objeto VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, fps, (frame_width, frame_height))

# Calcular las dimensiones de los frames redimensionados
frame_width = frame_width // 2
frame_height = frame_height // 2

# Inicializar el contador de unos y ceros
count_ones = 0
count_zeros = 0

# Inicializar la variable de tiempo de inicio
start_time = None

#mensaje
msg="motor off"
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
color = (255, 255, 255)
thickness = 2

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Fin del video")
        break

    # Mostrar el frame en una ventana
    #cv2.imshow('Video', frame)
    
    # Procesar el frame
    img,square,crop,processed_frame = process(frame)
    
    # Asegurarse de que todas las imágenes tengan el mismo número de columnas
    img = cv2.resize(img, (frame_width, frame_height))
    square = cv2.resize(square, (frame_width, frame_height))
    crop = cv2.resize(crop, (frame_width, frame_height))
    processed_frame = cv2.resize(processed_frame, (frame_width, frame_height))
    
    # Concatenar los frames horizontalmente para formar las dos filas
    top_row = cv2.hconcat([img, square])
    bottom_row = cv2.hconcat([crop, processed_frame])
    bottom_row = cv2.cvtColor(bottom_row, cv2.COLOR_GRAY2BGR)
   
    # Concatenar las filas verticalmente
    combined_frame = cv2.vconcat([top_row, bottom_row])
    
    # Comprobar si el frame procesado es todo negro o tiene pixeles en blanco
    if cv2.countNonZero(processed_frame) > 0:
        #print(1)
        count_ones += 1
        count_zeros = 0
        #print("motor on1",count_ones,count_zeros)
        msg="motor on"
        start_time = None
    else:
        #print(0)
        if count_ones != 0:  # ajusta estos números según tus necesidades
            
            count_zeros += 1
            #print("motor on2",count_ones,count_zeros)
            msg="motor on"
            
            if start_time is None:
                start_time = time.time()
                
            elif time.time() - start_time > 3:
                msg="motor off"
                start_time = None
                #print("motor off1",count_ones,count_zeros)
                count_ones = 0
                count_zeros = 0
            
        else:
            start_time = None
            msg="motor off"
            count_zeros = 0
            #print("motor off2",count_ones,count_zeros)

    
    # Mostrar el frame combinado
    text_size = cv2.getTextSize(msg, font, font_scale, thickness)[0]
    text_x = combined_frame.shape[1] - text_size[0] - 10  # 10 píxeles de margen desde la derecha
    text_y = text_size[1] + 10  # 10 píxeles de margen desde la parte superior
    combined_frame=cv2.putText(combined_frame, msg, (text_x, text_y), font, font_scale, color, thickness)

    cv2.imshow("Combined2 Video", combined_frame)
    
    # Escribir el frame procesado en el archivo de salida
    out.write(combined_frame)
    
    # Esperar según el frame rate del video
    if cv2.waitKey(wait_time) & 0xFF == ord('q'):
        break

# Liberar el objeto de captura y cerrar las ventanas
cap.release()
out.release()
cv2.destroyAllWindows()

