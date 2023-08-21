from .color_detection import *
from .coord_new_reference_point import *

def target_tracking(frame, color, lat_PA, lon_PA, compass_theta):
    #time_elapsed = time.time() - prev

    #if time_elapsed > 1. / frame_rate:
    #prev = time.time()

    #################################### 1. ####################################
    gaussianblur_img = cv2.GaussianBlur(frame, (5, 5), 0)  # gaussian filter
    medianblur = cv2.medianBlur(gaussianblur_img, 5)  # median filter
    hsv = cv2.cvtColor(medianblur, cv2.COLOR_BGR2HSV)  # color transformation (BGR to HSV)
    result, mask = color_detection(color, hsv, frame)  # color detection in the frame

    #################################### 2. ####################################

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (15, 15))  # big kernel for the erosion (smoothing the noise)
    mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # erosion followed by dilation
    mask_opened = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)  # erosion followed by dilation
    gray_filtered = cv2.bilateralFilter(mask_opened, 7, 50, 50)  # color to grayscale for the erosion
    kernel = np.ones((5, 5), np.uint8)  # another kernel used for the erosion
    erode_gray = cv2.erode(gray_filtered, kernel, iterations=1)  # erosion using the smaller kernel 

    contours, _ = cv2.findContours(erode_gray, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)  # find contours of the color we chose

    #################################### 3. ####################################

    # To be set before the flight 
    area_min = 1000  # to be change
    area_max = 100000  # to be change

    cir_min = 0.85
    cir_max = 1


    if not contours:
        ''' if no contours are find we skip to a new frame'''
        cX, cY = np.nan, np.nan
        north_x_target = np.nan
        north_y_target = np.nan
        vect_GPS_target = np.array([np.nan, np.nan])
        # print("frame", K_it)
        # print("no contour")
        pass

    for contour in contours:

        area = cv2.contourArea(contour)

        if area_min < area < area_max:

            perimeter = cv2.arcLength(contour, True)  # in pixel

            circularity = 4 * np.pi * (area / (perimeter ** 2))

            if cir_min < circularity < cir_max:
                # pix_cm_ratio = real_perimeter_target / perimeter #[cm/pixel]
                ratio = 30 / 144  # [m/px]
                gg_contour = contour

                # print("-----------------------------------------------------")
                # print("frame number ", K_it)

                # print("Circularity of {} ".format(circularity))

                # # on print les paramètres importants en cm
                # print("La circularité de la cible est de {a}".format(a = circularity))
                # print("L'aire de la cible est de {a} cm^2".format(a = area  / pix_cm_ratio**2))
                # print("Le périmètre de la cible est de {a} cm".format(a = perimeter / pix_cm_ratio))

                # print contours
                cv2.drawContours(frame, gg_contour, -1, (0, 0, 255), 2)

                #################################### 4. ####################################

                M = cv2.moments(gg_contour)

                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # centre = cv2.circle(copie_frame, (cX, cY), 6, (255, 0, 255), -1) 
                elif not M:
                    cX, cY = np.nan, np.nan
                    vect_GPS_target = np.array([np.nan, np.nan])
                    print("No coordinates")
                    Warning("No target found. cv2.moments empty")

                #################################### 5. ####################################

                dimensions = frame.shape  # Get frame dimension

                # Les distances transformées en "integer" [pixel] 
                centre_img_y = np.int0(dimensions[0] / 2)  # number of line in the matrix, pixel in the y direction
                centre_img_x = np.int0(
                    dimensions[1] / 2)  # number of column in the matrix, pixel in the x direction
                cv2.line(frame, (centre_img_x, centre_img_y), (cX, cY), (255, 0, 0),
                         2)  # line between the center of the target and the center of the frame
                cv2.circle(frame, (centre_img_x, centre_img_y), 5, (0, 0, 255),
                           -1)  # small dot in center of the frame

                #################################### 6. ####################################

                # # On trouve la distance entre le centre de l'image et le centre du cercle, en x et en y
                # dist_cible_x = np.abs(cX - centre_img_x) # [pixel]
                # dist_cible_y = np.abs(cY - centre_img_y) # [pixel]

                # print("La distance en x entre la cible et le centre de l'image est de {a} m".format(a = dist_cible_x * ratio))
                # print("La distance en y entre la cible et le centre de l'image est de {a} m".format(a = dist_cible_y * ratio))

                # dist_norme = np.sqrt(dist_cible_x**2 + dist_cible_y**2) * ratio
                # print("La distance entre la cible et le centre de l'image est de {a} m".format(a = dist_norme))

                #################################### 7. ####################################
                # on transforme les distances en des points GPS  

                # cd1 = np.array([45.52284, -73.61116]) # latitude, longitude
                # lat = cd1[0]
                # lon = cd1[1]
                # compass_theta = 

                north_x_target, north_y_target, GPS_lat_target, GPS_lon_target = coord_new_reference_point(frame,
                                                                                                           cX, cY,
                                                                                                           ratio,
                                                                                                           lat_PA,
                                                                                                           lon_PA,
                                                                                                           compass_theta)

                #################################### 8. ####################################
                # On stock les différents points GPS
                '''On l'initialise au début avec un certain nombre d'images souhaités,
                exemple: vect_distance = np.zeros(20) contiendra 20 emplacements pour garder les distances entre
                le centre de la cible et le PA. On doit aussi dire au code de s'arrêter après avoir trouvé 20 
                cibles, alors on le dit dans le "break" à la fin: K_it == 20. '''

                vect_GPS_target = np.array(
                    [GPS_lat_target, GPS_lon_target])  # stocking all the target's GPS coordinates
                # print('frame numéro : {} '.format(np.count_nonzero(~np.isnan(vect_GPS_target)))) # doit compter juste les lignes mais live on compte tous les éléments
                # print('vecteur avec les coordonnées GPS de la cible: \n {} '.format(vect_GPS_target)) 
                ############################################################################

                # print('latitude du PA : {} [°] '.format(lat_PA))
                # print('longitude du PA : {} [°] '.format(lon_PA))

                # print('latitude de la cible : {} [°] '.format(GPS_lat_target))
                # print('longitude de la cible : {} [°] '.format(GPS_lon_target))



            else:

                north_x_target = np.nan
                north_y_target = np.nan
                vect_GPS_target = np.array([np.nan, np.nan])

        else:

            north_x_target = np.nan
            north_y_target = np.nan
            vect_GPS_target = np.array([np.nan, np.nan])

    return (vect_GPS_target, frame, north_x_target, north_y_target)
