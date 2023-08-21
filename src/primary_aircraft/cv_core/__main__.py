<<<<<<< Updated upstream
=======
import numpy as np

from target_tracking import *
>>>>>>> Stashed changes
from tqdm import tqdm

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

if (cap.isOpened() == False): 
  print("Error opening video stream or file")

while(cap.isOpened()):

    res, frame = cap.read()

#     frame = cv2.flip(frame, 1)

    # tracking
    vect_GPS_target, frame, _, _ = target_tracking(frame, color, lat_PA, lon_PA, compass_theta)

    #cv2.imshow(" ", frame)

    if not np.isnan(vect_GPS_target).any():
        K_it += 1
        pbar.update(1)

    if cv2.waitKey(25) & 0xFF == ord('q') or K_it == nb_detect:
        break

pbar.close()

print("\n All detections done \n")

#cap.release()
#cv2.destroyAllWindows()
#cv2.waitKey(1)

# if you want to display all target positions
# print(vect_GPS_target)

# mean value of the coordinates
mean_GPS_target = np.nanmean(vect_GPS_target, 0)
print(mean_GPS_target)
# standard deviation ===> std_GPS = stat.stdev(vect_GPS)
std_GPS_target = np.nanstd(vect_GPS_target, 0)
# erreur type sur la moyenne ===> err_GPS = std_GPS / np.sqrt(N)
# err_GPS = std_GPS_target / np.sqrt(len(vect_GPS_target)) 
err_GPS = std_GPS_target / np.sqrt((np.count_nonzero(~np.isnan(vect_GPS_target)) / 2)) 

print('mean target coordinates : {a} \u00B1 {b}, {c} \u00B1 {d} \n '.format(a = mean_GPS_target[0], b = err_GPS[0], c = mean_GPS_t>


