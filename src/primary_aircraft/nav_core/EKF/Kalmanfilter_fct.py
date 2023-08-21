# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 10:53:30 2023

@author: Catherine
"""
import math

import numpy as np
from numpy import pi, sin, cos, arccos, arcsin, sqrt


def radius (lat):
    lat=math.radians(lat) #converting into radians
    a = 6378.137  #Radius at sea level at equator
    b = 6356.752  #Radius at poles
    c = (a**2*math.cos(lat))**2
    d = (b**2*math.sin(lat))**2
    e = (a*math.cos(lat))**2
    f = (b*math.sin(lat))**2
    R = math.sqrt((c+d)/(e+f))
    return R # @sea level

def distCoords(cd1,cd2,altitude):
    '''
    :param cd1: coords1, degrees [lat,long]
    :param cd2:
    :return:
    '''
    
    earthRadii = radius((cd1[0]+cd2[0])/2) #Km #EARTH RADII VARIES BASED ON LATITUDE
    #altitude = 0.233
    
    
    lat1 = cd1[0] / 180 * pi
    long1 = cd1[1] / 180 * pi

    lat2 = cd2[0] / 180 * pi
    long2 = cd2[1] / 180 * pi

    d1 = earthRadii * arccos((sin(lat1) * sin(lat2)) + cos(lat1) * cos(lat2) * cos(long2-long1))
    #print(d1*1000)
    d2 = (earthRadii+altitude) * arccos((sin(lat1) * sin(lat2)) + cos(lat1) * cos(lat2) * cos(long2 - long1))
    #print(d2*1000)

    # Haversine formula
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

    c = 2 * arcsin(sqrt(a))


    # calculate the result
    return c * earthRadii*1000


def orientation(cd1, cd2 ):
    '''
    :param cd1:
    :param cd2:
    :param altitude:
    :return:
    '''

    lat1 = cd1[0]
    long1 = cd1[1]
    lat2 = cd2[0]
    long2 = cd2[1]

    dlon = long2 - long1
    dlat = lat2 - lat1

    # North is +, East is +
    # degrees_temp = math.atan2(dlon, dlat) / math.pi * 180 # Degrees measured from equator (East) to North
    if dlat >= 0:
        message = '[ N '+str(abs(dlat))+' ; '
    elif dlat < 0:
        message = '[ S '+str(abs(dlat))+' ; '

    if dlon >= 0:
        message += 'E '+str(abs(dlon))+' ] '
    elif dlon < 0:
        message += 'W '+str(abs(dlon))+' ] '

    #print(message)
    # N dlat ; E dlon -> ex: (N 10deg ; E 20deg)
    
    
    
    
    


def get_bearing(lat1, long1, lat2, long2):
    #fonction qui retourne le bearing entre 2 ping gps
    dLon = (long2 - long1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    brng = np.arctan2(x,y)
    brng = np.degrees(brng)

    return brng



def get_xy(x,alt):
    #Fonction qui prend en entrée un np.array contenant des données gps tel que montré ci-dessous
    # x=np.array([[lat1,lon1],[lat2,lon2]....,[latn,lonn]])
    #La fonction retourne la distance entre les 2 derniers ping GPS
    coords=x
 
    #coords=np.array([[0,0], [1.9740288191360256e-05,0.0001322460891825204]])
    dists=np.empty(0)
    ori=np.empty(0)
    if len(coords)<2:
        x=0
        y=0
        return x,y

        
        
    for i in range(0,len(coords)-1):
       cord1=coords[i]
       #print(cord1)
       cord2=coords[i+1]
       #print(cord2)
       dist=distCoords(cord1,cord2,alt)
       dists=np.append(dists,dist)
       brng=get_bearing(cord1[0],cord1[1],cord2[0],cord2[1])
       ori=np.append(ori,brng)

    cmb=len(ori)
    angle=np.full(cmb,90)
    angle=np.subtract(angle,ori)
    cumulx=np.array([0])
    cumuly=np.array([0])
    varx=0
    vary=0
    for i in range(len(dists)):
        varx+=dists[i]*math.cos(math.radians(angle[i]))
        vary+=dists[i]*math.sin(math.radians(angle[i]))
        cumulx=np.append(cumulx,varx)
        cumuly=np.append(cumuly,vary)
        distx=varx+cumulx[-1]
        disty=vary+cumuly[-1]
        x=np.array([distx])
        y=np.array([disty])
    return x,y
#array pour plot 
xt = [] #position x corrected
yt = [] #position y corrected
dxt= [] #vitesse x corrected
dyt= [] #vitesse y corrected
ddxt=[] #acceleration x corrected
ddyt=[] #acceleration y corrected
Zx = [] #position x lue
Zy = [] #position y lue
Px = [] #uncertainty position x
Py = [] #uncertainty position y
Pdx= [] #uncertainty vitesse x
Pdy= [] #uncertainty vitesse y
Pddx=[] #uncertainty acc x
Pddy=[] #uncertainty acc y
Kx = [] #kalman gain position x
Ky = [] #kalman gain position y
Kdx= [] #kalman gain vitesse x
Kdy= [] #kalman gain vitesse y
Kddx=[] #kalman gain acc x
Kddy=[] #kalman gain acc y
    
def savestates(x, Z, P, K):
    xt.append(float(x[0]))
    yt.append(float(x[1]))
    dxt.append(float(x[2]))
    dyt.append(float(x[3]))
    ddxt.append(float(x[4]))
    ddyt.append(float(x[5]))
    Zx.append(float(Z[0]))
    Zy.append(float(Z[1]))
    Px.append(float(P[0,0]))
    Py.append(float(P[1,1]))
    Pdx.append(float(P[2,2]))
    Pdy.append(float(P[3,3]))
    Pddx.append(float(P[4,4]))
    Pddy.append(float(P[5,5]))
    Kx.append(float(K[0,0]))
    Ky.append(float(K[1,0]))
    Kdx.append(float(K[2,0]))
    Kdy.append(float(K[3,0]))
    Kddx.append(float(K[4,0]))
    Kddy.append(float(K[5,0]))    


#d#istance=distCoords(cd1,cd2)
#orient=orientation(cd1,cd2)


#coords=np.array([[],[],[]])
#x,y=get_xy(coords,0.3)
#print(x,y)

#plt.plot(x,y)

#plt.ticklabel_format(style='plain')

#brng=get_bearing(cd1[0],cd1[1],cd2[0],cd2[1])
#angle=90-brng
#angler=math.radians(angle)
#x=distance*np.cos(angler)
#y=distance*np.sin(angler)
#plt.ticklabel_format(style='plain')
#plt.plot(x,y,marker='o')



