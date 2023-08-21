from .radius import *
from .rotation_ref import *

def coord_new_reference_point(frame, cX, cY, ratio, lat, lon, theta):
    # The frame is the picture where the target has been spotted by the VC. Its just a big matrix
    # cX and cY are the pixel coordinates of the center of the target in the frame
    # ratio is in [m/pixel] and its use to go from pixel to meters
    # lat and lon are respectively the latitude and longitude of the PA which is the center of our frame
    # theta is the fligth direction of the PA (compass)
    '''We find the middle of the frame [pixel] and it will become our origin (0,0)
       technically the true origin is the top left corner. From this new origin we 
       find the cartesian coordinates of the target. Then, we change the Euclidean 
       space using the rotation matrix to be align with the geographic North. 
       Finally we calculate the latitude and the longitude of the target.
    '''
    R = radius(lat)

    # get the frame dimension
    dimensions = frame.shape
    # get the center of the frame
    center_img_y = np.int0(dimensions[0] / 2) # line of the matrix, pixel in y-axis 
    center_img_x = np.int0(dimensions[1] / 2) # column of the matrix, pixel in x-axis 
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
    elif cX >= center_img_x and cY >= center_img_y:
        new_x = dist_target_x
        new_y = -dist_target_y

        north_x, north_y = rotation_ref(new_x, new_y, theta)
    
        lon_target = 2 * np.arcsin(np.abs(north_x) / (2 * R))
        lat_target = -2 * np.arcsin(np.abs(north_y) / (2 * R))
    
    else:
        lat_target = np.nan
        lon_target = np.nan
        north_x = np.nan
        north_y = np.nan
        # print('ok')

    # final target coordinates in lat. / lon.
    GPS_lat_target = np.degrees(lat_target) + lat
    GPS_lon_target = np.degrees(lon_target) + lon

    return(north_x, north_y, GPS_lat_target, GPS_lon_target)