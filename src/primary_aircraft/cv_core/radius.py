import numpy as np

def radius(lat):
    # lat is the PA's latitude 
    '''The geocentric radius is the distance from the Earth's 
       center to a point on the spheroid surface at geodetic latitude φ
    '''
    lat = np.radians(lat) #converting into radians
    a = 6378.137  #Radius at sea level at equator
    b = 6356.7523  #Radius at poles
    c = (a**2*np.cos(lat))**2
    d = (b**2*np.sin(lat))**2
    e = (a*np.cos(lat))**2
    f = (b*np.sin(lat))**2
    R = np.sqrt((c + d)/(e + f))
    # print('rayon terrestre : {} [m] '.format(R*1000))
    return R*1000 # geocentric radius [m]