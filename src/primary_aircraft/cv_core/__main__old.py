import numpy as np
from tqdm import tqdm

from target_tracking_old import *

# variables initialization 
frame_rate = 15 # the frame rate
prev = 0 # frame rate variable
K_it = 0 # actualization of the number of detection (loading bar)
nb_detect = 30 # total number of detection wanted
vect_GPS_target = np.zeros([nb_detect, 2]) # vector to stock GPS target coordinates
color = "blue" # color = input("Couleur de la cible") # target's color

compass_theta = 330
ratio = 30/144 # chaque pixel vaut 50/120 ~ 0.41666... [m]
PA_coordinates = np.array([45.61968, -73.53193]) # cible 45.62065, -73.53361
lat_PA = PA_coordinates[0]
lon_PA = PA_coordinates[1]

pbar = tqdm(total = nb_detect)

cap = cv2.VideoCapture(0) # activation of the video capture

while(cap.isOpened()):

    res, frame = cap.read()

    frame = cv2.flip(frame, 1)

    # tracking
    GPS_lat_target, GPS_lon_target, vect_GPS_target, _, _ = target_tracking(frame, color, lat_PA, lon_PA, compass_theta, vect_GPS_target)#, prev, frame_rate)

    cv2.imshow(" ", frame)

    if ~np.isnan(GPS_lat_target):
        K_it += 1
        pbar.update(1)

    if cv2.waitKey(25) & 0xFF == ord('q') or K_it == nb_detect:
        break

pbar.close() 

cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)

print(vect_GPS_target)
print("All detections done")