import cv2
import numpy as np


def color_detection(color, hsv, img):

    # traitement des couleurs sur l'image (color_detection(color, hsv))
    # filtrage (upper and lower color)

    if color == "rouge" or color == "Rouge" or color == "ROUGE" or color == "red" or color == "Red" or color == "RED":

        # lower boundary RED color range values; Hue (0 - 10)
        lower_red1 = np.array([0, 100, 25]) # S was 80
        upper_red1 = np.array([10, 255, 255])
    
        # upper boundary RED color range values; Hue (160 - 180)
        lower_red2 = np.array([170, 100, 25]) # H = 170 ou autre chose...
        upper_red2 = np.array([180, 255, 255])
        
        lower_mask = cv2.inRange(hsv, lower_red1, upper_red1)
        upper_mask = cv2.inRange(hsv, lower_red2, upper_red2)
        
        mask = lower_mask + upper_mask
        
        result = cv2.bitwise_and(img, img, mask=mask)

    elif color == "jaune" or color == "Jaune" or color == "JAUNE" or color == "yellow" or color == "Yellow" or\
        color == "YELLOW":

        # lower and upper boundaries YELLOW color range values; Hue (40 - 75)
        lower_jaune = np.array([20, 80, 50])
        upper_jaune = np.array([39, 255, 255])

        mask = cv2.inRange(hsv, lower_jaune, upper_jaune) # select the yellow pixel in the range of the lower and upper yellow limit

        result = cv2.bitwise_and(img, img, mask=mask)

    elif color == "vert" or color == "Vert" or color == "VERT" or color == "verte" or \
        color == "Verte" or color == "VERTE" or color == "green" or color == "Green" or color == "GREEN":

        # lower and upper boundaries GREEN color range values; Hue (40 - 75)
        lower_green = np.array([40, 80, 25]) # lower_green = np.array([40, 80, 25])
        upper_green = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green) # select the green pixel in the range of the lower and upper green limit

        result = cv2.bitwise_and(img, img, mask=mask)

    elif color == "turquoise" or color == "Turquoise" or color == "TURQUOISE":

        # lower and upper boundaries TURQUOISE color range values; Hue (40 - 75)
        lower_turquoise = np.array([85, 80, 25])
        upper_turquoise = np.array([95, 255, 255])

        mask = cv2.inRange(hsv, lower_turquoise, upper_turquoise) # select the turquoise pixel in the range of the lower and upper turquoise limit

        result = cv2.bitwise_and(img, img, mask=mask)

    elif color == "bleu" or color == "Bleu" or color == "BLEU" or color == "bleue" or\
        color == "Bleue" or color == "BLEUE" or color == "blue" or color == "Blue" or color == "BLUE":

        lower_blue = np.array([100, 80, 25])
        upper_blue = np.array([135, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue) # select the blue pixel in the range of the lower and upper blue limit

        result = cv2.bitwise_and(img, img, mask=mask)
    
    elif color == "rose" or color == "Rose" or color == "ROSE" or color == "pink" or color == "Pink" or\
        color == "PINK":

        # lower and upper boundaries PINK color range value
        lower_pink = np.array([145, 80, 25])
        upper_pink = np.array([169, 255, 255])

        mask = cv2.inRange(hsv, lower_pink, upper_pink) # select the pink pixel in the range of the lower and upper pink limit

        result = cv2.bitwise_and(img, img, mask=mask)

    # Not working well...
    elif color == "mauve" or color == "Mauve" or color == "MAUVE" or color == "purple" or color == "Purple" or\
        color == "PURPLE":

        # lower and upper boundaries PINK color range value
        lower_purple = np.array([135, 50, 25])
        upper_purple = np.array([145, 255, 255])

        mask = cv2.inRange(hsv, lower_purple, upper_purple) # select the purple pixel in the range of the lower and upper purple limit

        result = cv2.bitwise_and(img, img, mask=mask)

    return(result, mask)