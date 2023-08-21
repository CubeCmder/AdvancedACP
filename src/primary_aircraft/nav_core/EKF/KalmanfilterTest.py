# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 10:26:33 2023

@author: Catherine
"""
from GPSPoller import GpsPoller
from LSM6DSL import *
from PA.EKF.Kalmanfilter_fct import *

#initial state
x = np.matrix([[0.0],[0.0],[0.0],[0.0],[0.0], [0.0]])
#print(x, x.shape)
n=x.size # States

#initial uncertainty 
P = np.diag([100.0, 100.0, 10.0, 10.0, 1.0, 1.0])


#print(P, P.shape)

#dynamic matrix

dt = 0.1 # Time Step between Filter Steps

A = np.matrix([[1.0, 0.0, dt, 0.0, 1/2.0*dt**2, 0.0],
              [0.0, 1.0, 0.0, dt, 0.0, 1/2.0*dt**2],
              [0.0, 0.0, 1.0, 0.0, dt, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, dt],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
#print(A, A.shape)

#Measurement matrix

H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
#print(H, H.shape)

#Measurement noise covariance 

ra = 10.0**2   # Noise of Acceleration Measurement
rp = 100.0**2  # Noise of Position Measurement
R = np.matrix([[rp, 0.0, 0.0, 0.0],
               [0.0, rp, 0.0, 0.0],
               [0.0, 0.0, ra, 0.0],
               [0.0, 0.0, 0.0, ra]])
#print(R, R.shape)

# processs noise covariance matrix

sj=0.1

Q = np.matrix([[(dt**6)/36, 0, (dt**5)/12, 0, (dt**4)/6, 0],
               [0, (dt**6)/36, 0, (dt**5)/12, 0, (dt**4)/6],
               [(dt**5)/12, 0, (dt**4)/4, 0, (dt**3)/2, 0],
               [0, (dt**5)/12, 0, (dt**4)/4, 0, (dt**3)/2],
               [(dt**4)/6, 0, (dt**3)/2, 0, (dt**2),0],
               [0, (dt**4)/6, 0, (dt**3)/2, 0, (dt**2)]]) *sj**2

#print(Q, Q.shape)

#I matrix

I = np.eye(n)
#print(I, I.shape)

#tests
cd1 = [45.5147478972148, -73.7745783865117]
cd2 = [75.48154597970112, -154.3539584108125]

# GPS readings (position)
m=100
GPS=np.ndarray(m,dtype=bool)
GPS[0]=True
mpx=np.array([0])
mpy=np.array([0])
coords=np.empty(0)
dists=np.empty(0)
ori=np.empty(0)
varx=0
vary=0
mx=np.array([0])
my=np.array([0])

#initie les sensors
#initie le IMU
LSM6DSL = LSM6DSL(smbus.SMBus(i2c_bus))
#initie le GPS
gpsp=GpsPoller()
gpsp.start()
#lat=gpsp.gpsd.fix.latitude
#lon=gpsp.gpsd.fix.longitude
#atl=gpsp.gpsd.fix.altitude
#coords=np.array([[lat,lon]])
#print(cord)

#initie le barometer


#prend les mesures à 0.1s d'interval
for i in range(1,m):
    time.sleep(0.1)
    #prise de mesure
    ax=LSM6DSL.readACCx()
    ay=LSM6DSL.readACCy()
    mx=np.append(mx,ax)
    my=np.append(my,ay)
    
    if i%10==0:
        GPS[i]=True
        lat=gpsp.gpsd.fix.latitude
        lon=gpsp.gpsd.fix.longitude
        atl=gpsp.gpsd.fix.altitude
        cord=np.array([[lat,lon]])
        print(cord)
        coords=np.append(coords,cord, axis=0)
        print(coords)
        X,Y=get_xy(coords,atl)
        print(X,Y)
        mpx=np.append(mpx,X)
        mpy=np.append(mpy,Y)
    else:
        mpx=np.append(mpx,np.array([mpx[-1]]),axis=0)
        mpy=np.append(mpy,np.array([mpy[-1]]),axis=0)
        GPS[i]=False



measurements=np.vstack((mpx,mpy,mx,my))
#print(measurements)
#print(measurements.shape)

#filtering  
#arrays for plotting 

for filterstep in range(m):
    
    # Time Update (Prediction)
    # ========================
    # Project the state ahead

    x = A*x
    
    
    # Project the error covariance ahead
    P = A*P*(A.T) + Q    
    
    
    # Measurement Update (Correction)
    # ===============================
    # if there is a GPS Measurement
    if GPS[filterstep]:
        # Compute the Kalman Gain
        S = H*P*H.T + R
        K = (P*H.T) * np.linalg.pinv(S)
    
        
        # Update the estimate via z
        Z = measurements[:,filterstep].reshape(H.shape[0],1)
        y = Z - (H*x)                            # Innovation or Residual
        x = x + (K*y)
        
        # Update the error covariance
        P = (I - (K*H))*P

   
    
    # Save states for Plotting
    savestates(x, Z, P, K)

#print(xt,yt) #positions corrigées
#print(Zx,Zy) #positions lues
diffx=np.subtract(Zx,xt)
#print(diffx)
print('ok done')
