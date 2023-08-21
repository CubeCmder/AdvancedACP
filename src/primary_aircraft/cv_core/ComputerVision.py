import time

import cv2
import numpy as np


def color_detection(color, hsv, img):
    # traitement des couleurs sur l'image (color_detection(color, hsv))
    # filtrage (upper and lower color)
    color = color.lower()
    if color == "rouge" or color == "red":

        # lower boundary RED color range values; Hue (0 - 10)
        lower_red1 = np.array([0, 100, 25])  # S was 80
        upper_red1 = np.array([10, 255, 255])

        # upper boundary RED color range values; Hue (160 - 180)
        lower_red2 = np.array([170, 100, 25])  # H = 170 ou autre chose...
        upper_red2 = np.array([180, 255, 255])

        lower_mask = cv2.inRange(hsv, lower_red1, upper_red1)
        upper_mask = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = lower_mask + upper_mask

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)


    elif color == "jaune" or color == "yellow":

        # lower and upper boundaries YELLOW color range values; Hue (40 - 75)
        lower_jaune = np.array([20, 80, 50])
        upper_jaune = np.array([39, 255, 255])

        mask = cv2.inRange(hsv, lower_jaune,
                           upper_jaune)  # select the green pixel in the range of the lower and upper green limit

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)

    elif color == "vert" or color == "verte" or color == "green":
        # lower and upper boundaries GREEN color range values; Hue (40 - 75)
        lower_green = np.array([40, 80, 25])  # lower_green = np.array([40, 80, 25])
        upper_green = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower_green,
                           upper_green)  # select the green pixel in the range of the lower and upper green limit

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)

    elif color == "turquoise":
        # lower and upper boundaries TURQUOISE color range values; Hue (40 - 75)
        lower_turquoise = np.array([85, 80, 25])
        upper_turquoise = np.array([95, 255, 255])

        mask = cv2.inRange(hsv, lower_turquoise,
                           upper_turquoise)  # select the turquoise pixel in the range of the lower and upper turquoise limit

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)

    elif color == "bleu" or color == "bleue" or color == "blue":
        lower_blue = np.array([100, 80, 25])
        upper_blue = np.array([135, 255, 255])

        mask = cv2.inRange(hsv, lower_blue,
                           upper_blue)  # select the blue pixel in the range of the lower and upper blue limit

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)

    elif color == "rose" or color == "pink":
        # lower and upper boundaries PINK color range value
        lower_pink = np.array([145, 80, 25])
        upper_pink = np.array([169, 255, 255])

        mask = cv2.inRange(hsv, lower_pink,
                           upper_pink)  # select the pink pixel in the range of the lower and upper pink limit

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)

    # Not working well...
    elif color == "mauve" or color == "purple":
        # lower and upper boundaries PINK color range value
        lower_purple = np.array([135, 50, 25])
        upper_purple = np.array([145, 255, 255])

        mask = cv2.inRange(hsv, lower_purple,
                           upper_purple)  # select the pink pixel in the range of the lower and upper pink limit

        result = cv2.bitwise_and(img, img, mask=mask)

        return (result, mask)


def radius(lat):
    # lat is the PA's latitude
    '''The geocentric radius is the distance from the Earth's
       center to a point on the spheroid surface at geodetic latitude φ
    '''
    lat = np.radians(lat)  # converting into radians
    a = 6378.137  # Radius at sea level at equator
    b = 6356.7523  # Radius at poles
    c = (a ** 2 * np.cos(lat)) ** 2
    d = (b ** 2 * np.sin(lat)) ** 2
    e = (a * np.cos(lat)) ** 2
    f = (b * np.sin(lat)) ** 2
    R = np.sqrt((c + d) / (e + f))
    # print('rayon terrestre : {} [m] '.format(R*1000))
    return R * 1000  # geocentric radius [m]


# we have to face North to get latitude and longitude
def rotation_ref(x, y, theta):
    # x and y are the target's coordinates in pixel
    # theta is the fligth direction of the PA
    '''The compass always gives you a positive number: North = 0°, West = 270° and so on.
       We consider the PA's coordinate system as the Local system and the earth's as our
       Global system coordinate, where theta is always the angle between the North and the
       PA's fligth direction so we have to change it
    '''
    # theta = np.radians(random.randint(0, 360)) # gets a random value between 0 and 360 and convert it to radians (compass output)
    theta = np.radians(-theta)
    c, s = np.cos(theta), np.sin(theta)
    #R_transpose = np.array([[c, s], [-s, c]])  # considering the earth as our "Global coordinate system"
    R = np.array([[c, -s], [s, c]])  # considering the PA as our "Local coordinate system"
    p = np.array([[x], [y]])
    output = R.dot(p)
    north_x, north_y = output[0][0], output[1][0]

    return (north_x, north_y)


# heres the big chunk
def coord_new_reference_point(frame, cX, cY, ratio, lat, lon, theta):
    # The frame is the picture where the target has been spotted by the VC. Its just a big matrix
    # cX and cY are the pixel coordinates of the center of the target in the frame
    # ratio is in [m/pixel] and its use to go from pixel to meters
    # lat and lon are respectively the latitude and longitude of the PA which is the center of our frame
    # theta is the fligth direction of the PA (compass)
    '''We find the middle of the frame [pixel] and it will become our origin (0,0)
       technically the true origin is the top left corner. From this new origin we
       find the cartesian coordinates of the target. Then, we change the Euclidean
       space using the rotation matrix to be aligned with the geographic North.
       Finally, we calculate the latitude and the longitude of the target.
    '''
    R = radius(lat)

    # get the frame dimension
    dimensions = frame.shape
    # get the center of the frame
    center_img_y = np.int0(dimensions[0] / 2)  # line of the matrix, pixel in y-axis
    center_img_x = np.int0(dimensions[1] / 2)  # column of the matrix, pixel in x-axis
    # get the distance between the middle and the target (x and y axis in absolute value)
    dist_target_x = np.abs(cX - center_img_x) * ratio
    dist_target_y = np.abs(cY - center_img_y) * ratio

    # if the target is in the first quadrant of the new cartesian system (where the middle point of the frame is the origin)
    if cX >= center_img_x and cY <= center_img_y:
        new_x = dist_target_x
        new_y = dist_target_y

        north_x, north_y = rotation_ref(new_x, new_y, theta)

        lon_target = 2 * np.arcsin(np.abs(north_x) / (2 * R))
        lat_target = 2 * np.arcsin(np.abs(north_y) / (2 * R))

    # if the target is in the second quadrant
    elif cX <= center_img_x and cY <= center_img_y:
        new_x = -dist_target_x
        new_y = dist_target_y

        north_x, north_y = rotation_ref(new_x, new_y, theta)

        lon_target = -2 * np.arcsin(np.abs(north_x) / (2 * R))
        lat_target = 2 * np.arcsin(np.abs(north_y) / (2 * R))

    # if the target is in the third quadrant
    elif cX <= center_img_x and cY >= center_img_y:
        new_x = -dist_target_x
        new_y = -dist_target_y

        north_x, north_y = rotation_ref(new_x, new_y, theta)

        lon_target = -2 * np.arcsin(np.abs(north_x) / (2 * R))
        lat_target = -2 * np.arcsin(np.abs(north_y) / (2 * R))

    # if the target is in the fourth quadrant
    elif cX <= center_img_x and cY <= center_img_y:
        new_x = dist_target_x
        new_y = -dist_target_y

        north_x, north_y = rotation_ref(new_x, new_y, theta)

        lon_target = 2 * np.arcsin(np.abs(north_x) / (2 * R))
        lat_target = -2 * np.arcsin(np.abs(north_y) / (2 * R))

    # if you are really reading this... Hi :)
    # final target coordinates in lat. / lon.
    GPS_lat_target = np.degrees(lat_target) + lat
    GPS_lon_target = np.degrees(lon_target) + lon

    return (north_x, north_y, GPS_lat_target, GPS_lon_target)

#################################################################################################################
#################################################################################################################

start = time.time()

frame_rate = 15
prev = 0
# compte le nombre de fois qu'on trouve un bon contour
K_it = 0
nb_detect = 3
vect_GPS_target = np.zeros([nb_detect, 2])

# on entre la couleur de la cible qu'on cherche
color = input("Couleur de la cible")

# opening the webcam
cap = cv2.VideoCapture(0)

while (cap.isOpened()):

    time_elapsed = time.time() - prev
    res, frame = cap.read()

    if time_elapsed > 1. / frame_rate:
        prev = time.time()

        #################################### 1. ####################################

        # on applique certain filtre pour diminuer le bruit dans l'image

        # filtre gaussien
        gaussianblur_img = cv2.GaussianBlur(frame, (5, 5), 0)
        # filtre median
        medianblur = cv2.medianBlur(gaussianblur_img, 5)

        # on transforme l'image en couleur HSV (BGR to HSV)
        hsv = cv2.cvtColor(medianblur, cv2.COLOR_BGR2HSV)

        # on introdruit la fonction pour trouver la couleur
        result, mask = color_detection(color, hsv, frame)

        #################################### 2. ####################################

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        # closing small holes inside the foreground objects, or small black points on the object
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # useful in removing noise, opening is just another name of erosion followed by dilation
        mask_opened = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

        # On convertit l'image du masque en grayscale pour faire l'érosion
        gray_filtered = cv2.bilateralFilter(mask_opened, 7, 50, 50)

        # structuring element used for erosion
        kernel = np.ones((5, 5), np.uint8)
        # ****À REVOIR****

        erode_gray = cv2.erode(gray_filtered, kernel,
                               iterations=1)  # erode_gray = cv2.erode(gray, kernel, iterations = 1)

        contours, _ = cv2.findContours(erode_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #################################### 3. ####################################

        # selon l'altitude du P.A. et la grosseur de la cible
        contour_area = []
        contours_circles = []
        cir_min = 0.85
        cir_max = 1
        area_min = 500  # à revoir !
        area_max = 100000  # à revoir !
        font = cv2.FONT_HERSHEY_COMPLEX

        # on définit le périmètre de la cible sachant sa vraie grosseur
        r = 1.3  # cm
        real_perimeter_target = 2 * np.pi * r
        # print("Le périmètre théorique de la cible est de {a} cm".format(a = real_perimeter_target))

        for contour in contours:
            area = cv2.contourArea(contour)
            # print(area)
            dimensions = frame.shape
            # Les distances transformées en "integer" [pixel]
            cv2.drawContours(frame, contour, -1, (255, 0, 255), 2)
            # nombre de ligne de la matrice donc le nombre de pixel en y
            centre_img_y = np.int0(dimensions[0] / 2)
            # nombre de colonne de la matrice donc le nombre de pixel en x
            centre_img_x = np.int0(dimensions[1] / 2)
            # on dessine une ligne entre le centre de l'image et le centre de la cible
            cv2.line(frame, (centre_img_x, centre_img_y), (cX, cY), (255, 0, 0), 2)
            # on dessine un point au centre de l'image
            cv2.circle(frame, (centre_img_x, centre_img_y), 5, (0, 0, 255), -1)

            if area_min < area < area_max:  # on trouvera environ à 50 pieds d'altitude c'est quoi la limite d'air possible pour une cible selon notre caméra

                perimeter = cv2.arcLength(contour, True)  # in pixel
                # print(perimeter)
                # contour_area = cv2.contourArea(contour)

                # if perimeter == 0:
                #     ''' ici on devra dire au code que s'il ne trouve aucun contour (donc aucune cible)
                #     il doit attendre pour la prochaine image, mais pas nécessaire pour le moment'''
                #     Warning('Aucun contour trouvé')
                #     break;

                circularity = 4 * np.pi * (area / (perimeter ** 2))

                if cir_min < circularity < cir_max:
                    pix_cm_ratio = real_perimeter_target / perimeter  # [cm/pixel]
                    gg_contour = contour

                    print(circularity)

                    K_it = K_it + 1
                    print(K_it)
                    # contours_circles.append(contour)

                    # # on print les paramètres importants en cm
                    # print("La circularité de la cible est de {a}".format(a = circularity))
                    # print("L'aire de la cible est de {a} cm^2".format(a = area  / pix_cm_ratio**2))
                    # print("Le périmètre de la cible est de {a} cm".format(a = perimeter / pix_cm_ratio))

                    cv2.drawContours(frame, gg_contour, -1, (255, 0, 255), 2)

                    #################################### 4. ####################################

                    M = cv2.moments(gg_contour)

                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        # centre = cv2.circle(copie_frame, (cX, cY), 6, (255, 0, 255), -1)
                    else:
                        cX, cY = 0, 0
                        Warning("Aucune cible n'a été trouvée")
                        pass

                    # "m01" in M:

                    #################################### 5. ####################################

                    # On définit les dimensions de l'image ou du frame (même affaire) on le fait au début,
                    # mais on le refait ici pour que ça soit clair
                    dimensions = frame.shape

                    # Les distances transformées en "integer" [pixel]
                    # nombre de ligne de la matrice donc le nombre de pixel en y
                    centre_img_y = np.int0(dimensions[0] / 2)
                    # nombre de colonne de la matrice donc le nombre de pixel en x
                    centre_img_x = np.int0(dimensions[1] / 2)

                    # on dessine une ligne entre le centre de l'image et le centre de la cible
                    cv2.line(frame, (centre_img_x, centre_img_y), (cX, cY), (255, 0, 0), 2)
                    # on dessine un point au centre de l'image
                    cv2.circle(frame, (centre_img_x, centre_img_y), 5, (0, 0, 255), -1)

                    #################################### 6. ####################################

                    # On trouve la distance entre le centre de l'image et le centre du cercle, en x et en y
                    dist_cible_x = np.abs(cX - centre_img_x)  # [pixel]
                    dist_cible_y = np.abs(cY - centre_img_y)  # [pixel]

                    print("La distance en x entre la cible et le centre de l'image est de {a} cm".format(
                        a=dist_cible_x * pix_cm_ratio))
                    print("La distance en y entre la cible et le centre de l'image est de {a} cm".format(
                        a=dist_cible_y * pix_cm_ratio))

                    dist_norme = np.sqrt(dist_cible_x ** 2 + dist_cible_y ** 2) * pix_cm_ratio
                    print("La distance entre la cible et le centre de l'image est de {a} cm".format(a=dist_norme))

                    #################################### 7. ####################################
                    # on transforme les distances en des points GPS

                    cd1 = np.array([45.52284, -73.61116])  # latitude, longitude
                    lat = cd1[0]
                    lon = cd1[1]
                    compass_theta = 0

                    north_x_target, north_y_target, GPS_lat_target, GPS_lon_target = coord_new_reference_point(frame,
                                                                                                               cX, cY,
                                                                                                               pix_cm_ratio,
                                                                                                               lat, lon,
                                                                                                               compass_theta)

                    # print('latitude du PA : {} [°] '.format(lat))
                    # print('longitude du PA : {} [°] '.format(lon))

                    # print('latitude de la cible : {} [°] '.format(GPS_lat_target))
                    # print('longitude de la cible : {} [°] '.format(GPS_lon_target))

                    #################################### 8. ####################################
                    # On stock les différents points GPS et on fait une moyenne sur l'ensemble des points retenus
                    '''On l'initialise au début avec un certain nombre d'images souhaités,
                       exemple: vect_distance = np.zeros(20) contiendra 20 emplacements pour garder les distances entre
                       le centre de la cible et le PA. On doit aussi dire au code de s'arrêter après avoir trouvé 20 
                       cibles, alors on le dit dans le "break" à la fin: K_it == 20. '''

                    # vect_GPS_target[K_it - 1] = np.array([GPS_lat_target, GPS_lon_target])


                else:
                    pass

        # Do something with your image here
        cv2.imshow('Frame', frame)
        # Press q on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q') or K_it == nb_detect:
            break

#################################### 9. ####################################
# # on calcule la moyenne des points GPS à partir de "vect_GPS_target" et on calcule l'erreur
# import statistics as stat
# # mean value of the coordinates
# mean_GPS_target = np.mean(vect_GPS_target, 0)
# # standard deviation ===> std_GPS = stat.stdev(vect_GPS)
# std_GPS_target = np.std(vect_GPS_target, 0)
# # erreur type sur la moyenne ===> err_GPS = std_GPS / np.sqrt(N)
# err_GPS = std_GPS_target / np.sqrt(len(vect_GPS_target))

# print('la coordonnée moyenne est : {a} \u00B1 {b}, {c} \u00B1 {d}'.format(a = mean_GPS_target[0], b = err_GPS[0], c = mean_GPS_target[1], d = err_GPS[1]))

# Closes all the frames
cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)