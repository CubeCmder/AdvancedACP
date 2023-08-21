import numpy as np

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
    c, s = np.cos(theta), np.sin(theta)
    R_transpose = np.array([[c, s], [-s, c]]) # considering the earth as our "Global coordinate system"
    R = np.array([[c, -s], [s, c]]) # considering the PA as our "Local coordinate system"
    p = np.array([[x], [y]])
    output = R.dot(p)
    north_x, north_y = output[0][0], output[1][0]

    return(north_x, north_y)