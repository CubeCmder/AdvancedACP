# Nous tentons de calculer la position d'une cible CIRCULAIRE d'une couleur connue à partir d'un avion en vol. Nous connaissons sa position GPS et son altitude, ainsi que son orientation.
# Nous utiliseron python et open cv
# Inputs:
#   - Résolution [m/px]
#   - Position centre de l'image [GPS]
#   - Couleur recherchée
#   - Image
#   - Bearing / Orientation

# 1) Détecter la cible sur l'image
#
# 2) Évaluer la position de la cible [en pixels] par rapport au centre de l'image
#
# 3) Utiliser l'altitude [en metres], la direction [degrès par rapport au nord] et la position du véhicule [coordonnées GPS] pour calculer la position de la cible

import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from utils.nav_math import get_lat_lon_from_dy_dx
from modules.cameratransform import SpatialOrientation, camera, projection

def calculate_target_coordinates_vertical(target_loc, altitude, img_size, fov):
    """

    Args:
        altitude:
        img_size:
        fov:

    Returns:

    """
    # Image center coordinates
    center_x_image = img_size[0] // 2
    center_y_image = img_size[1] // 2

    # Target coordinates on the image in [px]
    target_x = target_loc[0]
    target_y = target_loc[1]

    # Calculate ground distance, in [m]
    ground_width = 2 * altitude * math.tan(math.radians(fov / 2))

    # Calculate the position of the target relative to the image center (which itself represents the aircraft)
    rel_target_x = target_x - center_x_image
    rel_target_y = target_y - center_y_image

    # Calculate the real world offsets in meters, ground_width / img_size[0] is the image resolution in [m/px]
    offset_x = rel_target_x * ground_width / img_size[0]
    offset_y = rel_target_y * ground_width / img_size[0]

    # maybe offset y needs to be *-1
    return offset_x, offset_y

def get_camera_angles_from_aircraft(pitch, roll):
    rot_mat = Rotation.from_euler('xy', [roll, pitch], degrees=True).as_matrix()

    z_prime = np.linalg.linalg.matmul(rot_mat, [0.0, 0.0, -1.0])
    math.degrees(np.pi/2 - np.arccos(np.abs(np.linalg.linalg.dot(z_prime, [0.0, 0.0, -1.0]))/(np.linalg.norm(z_prime))))
def get_img_offset_meters(target_loc, altitude, pitch, roll, img_size, hfov):
    """
    Calculate the real world offsets in meters for the target position relative to the aircraft. This function considers
    aircraft pitch and roll.

    Args:
        target_loc: Target position on image; array (1x2) [pixels]
        altitude: Aircraft altitude, or camera height above the ground plane; float [meters]
        pitch: Camera pitch (0 is looking straight in front) [degrees]
        roll: Camera roll angle [degrees]
        img_size: Image size in pixels; array (1x2) [pixels]
        hfov: Horizontal FOV angle of camera [degrees]

    Returns: X and Y offsets in meters relative to the aircraft.

    """
    camera_pitch, camera_roll, camera_heading = get_camera_angles_from_aircraft()
    orientation = SpatialOrientation(elevation_m=altitude, tilt_deg=90 - pitch, roll_deg=roll)

    proj = projection.RectilinearProjection(image_width_px=img_size[0], image_height_px=img_size[1], view_x_deg=hfov,
                                            view_y_deg=hfov * img_size[1] / img_size[0])

    cam = camera(proj, orientation)

    offset_x, offset_y, _ = cam.spaceFromImage(target_loc)

    # maybe offset y needs to be *-1
    return offset_x, offset_y


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


def detect_target(frame, color):
    """
    Detects the target on the captured frame.

    Args:
        frame: The image frame as taken from the camera
        color: A string describing the target's color

    Returns:
        frame: The image frame as taken from the camera, with the detected contour.
        target_contour: The contour of the target if detected, otherwise None
    """

    # Convertir l'image en espace de couleur HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Filtrer la couleur rouge dans l'image
    result, mask = color_detection(color, hsv_frame, frame)

    # Appliquer un flou pour réduire le bruit
    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Trouver les contours de la cible
    contours, _ = cv2.findContours(blurred_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Identifier le contour circulaire le plus grand (supposé être la cible)
    if len(contours) == 0:

        return frame, None

    else:
        target_contour = max(contours, key=cv2.contourArea)

        # Dessiner le contour de la cible sur l'image d'origine
        cv2.drawContours(frame, [target_contour], -1, (0, 255, 0), 2)

        return frame, target_contour


def pos_target(frame, target_contour, altitude, hfov, pitch=0, roll=0):
    '''
    Find the position of the center of the target relative to the image center.

    Args:
        frame: The image
        target_contour: The contour of the detected target

    Returns:
        pos: The [X, Y] coordinates (in pixels) of the target's center relative to the image center.
             A positive X indicates the target is on the right side of the image and
             a positive Y indicates that the target is on the upper side of the image.

    '''

    # Calculer les coordonnées du centre de l'image
    height, width = frame.shape[:2]
    center_x_image = width // 2
    center_y_image = height // 2

    # Trouver le centre du contour de la cible
    M = cv2.moments(target_contour)
    center_x_target = int(M["m10"] / M["m00"])
    center_y_target = int(M["m01"] / M["m00"])
    # on dessine un point au centre de l'image
    cv2.circle(frame, (center_x_target, center_y_target), 3, (0, 255, 0), -1)
    cv2.line(frame, (center_x_target, center_y_target), (center_x_image, center_y_image), (0, 255, 0), 2)

    offx, offy = get_img_offset_meters([center_x_target, center_y_target],
                                       altitude,
                                       pitch,
                                       roll,
                                       [width, height],
                                       hfov)
    return offx, offy


def locate_target(bearing, center_gps, x_y_target ):
    '''
    Locate the target based on the GPS coordinate system.

    Args:
        bearing: the compass bearing [degrees from North]
        center_gps: the gps coordinates of the image center,
                    assumed to be the coordinates of the plane [latitude, longitude]
        x_y_target: the [X, Y] coordinates of the target about the image center [meters]

    Returns:
        target_gps: The target's GPS coordinates.
    '''

    # Convertir la direction de l'avion de degrés en radians
    bearing_rad = math.radians(bearing)

    # Convertir les coordonnées GPS du véhicule en coordonnées cartésiennes (X, Y) par rapport à l'origine (centre de l'image)
    # Vous pouvez utiliser des formules de conversion spécifiques en fonction de votre système de coordonnées (par exemple, WGS84)
    latitude_origin = center_gps[0]
    longitude_origin = center_gps[1]

    x, y = x_y_target

    # Effectuer la rotation en fonction de la direction de l'avion [mètres vers l'Est et vers le Nord]
    rotated_x = x * math.cos(bearing_rad) - y * math.sin(bearing_rad)
    rotated_y = x * math.sin(bearing_rad) + y * math.cos(bearing_rad)

    # À partir des coordonnées du véhicule et le offset en mètres vers le Nord et le Sud, trouver les coordonnées de la Cible
    dlat, dlong = get_lat_lon_from_dy_dx(latitude_origin, longitude_origin, rotated_x, rotated_y)
    # dlat2, dlong2 = get_point_at_distance(latitude_origin, longitude_origin, rotated_x, rotated_y)

    target_gps = [dlat, dlong]
    target_pos_rel = [x, y]

    return target_gps, target_pos_rel


if __name__ == '__main__':
    resolution = 945.4581754044675 / 803
    resolution = 463.6084526469746 / 1109
    latitude_vehicule, longitude_vehicule = 45.51778389240875, -73.78416575263206
    bearing = 0

    # Charger l'image
    image_path = r'C:\Users\Admin\Desktop\All Files\Projects\2.AvionCargo\AdvancedSystems\PA\map2.png'
    image = cv2.imread(image_path)

    res, target_contour = detect_target(image, 'red')
    if target_contour is None:
        print('\nNo target detected.')
    else:
        target_pos_rel, target_pos = pos_target(image, target_contour,altitude, hfov, pitch=0, roll=0)
        target_gps, target_pos_rel = locate_target(bearing, [latitude_vehicule, longitude_vehicule], target_pos_rel,
                                                   resolution)

        cv2.imshow('Image avec la cible détectée', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
