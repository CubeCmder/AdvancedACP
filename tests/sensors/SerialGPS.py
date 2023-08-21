
import os,sys
print(sys.stdin.isatty()) ### True means shell, false means gui
from gps import *
from time import *
import time
import threading
import plotext as plt;
from numpy import std, sqrt, pi, linspace, cos, sin, append;
from coordDist import distCoords;
import csv


os.system('clear') #clear the terminal (optional)

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    while gpsp.running:
      self.gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread

  N = 0
  avg_lat = 0
  avg_long = 0
  done = False
  x = []
  y = []


  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
      if N == 5 and not done:
        N = 0
        avg_lat = 0
        avg_long = 0
        done = True
        x = []
        y = []

      os.system('clear')
      lat = gpsp.gpsd.fix.latitude
      long = gpsp.gpsd.fix.longitude
      print()
      print (' GPS reading')
      print ('----------------------------------------')
      print ('latitude        ' , lat)
      print ('longitude       ' , long)
      avg_lat = (avg_lat * N + lat)/(N+1)
      avg_long = (avg_long * N + long)/(N+1)
      print('average coords   ' , avg_lat,',',avg_long)
      print ('time utc        ' , gpsp.gpsd.utc,' + ', gpsp.gpsd.fix.time)
      print ('altitude (m)    ' , gpsp.gpsd.fix.altitude)
      print ('eps             ' , gpsp.gpsd.fix.eps)
      print ('epx             ' , gpsp.gpsd.fix.epx)
      print ('epv             ' , gpsp.gpsd.fix.epv)
      print ('ept             ' , gpsp.gpsd.fix.ept)
      print ('speed (m/s)     ' , gpsp.gpsd.fix.speed)
      print ('climb           ' , gpsp.gpsd.fix.climb)
      print ('track           ' , gpsp.gpsd.fix.track)
      print ('mode            ' , gpsp.gpsd.fix.mode)
      print()
      print('sats             ', gpsp.gpsd.satellites)
      y.append(lat)
      x.append(long)

      time.sleep(1)  # set to whatever
      N+=1

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("\nKilling Thread...")
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing

  # Study accuracy
  std_lat = std(y)
  std_long = std(x)
  # 50% confidence radius
  CEP = 0.59*(std_lat + std_long)
  # 95% confidence radius
  DRMS_95 = 2*sqrt(std_lat**2+std_long**2)

  # Generate radius
  angle = linspace(0, 2 * pi, 150)
  xCEP = CEP * cos(angle)+avg_long
  yCEP = CEP * sin(angle)+avg_lat

  l, p = avg_long, avg_lat;
  plt.scatter(x,y, marker = 'x');
  plt.scatter([avg_long], [avg_lat], marker='o', color='red+');
  plt.plot(xCEP, yCEP, color='red+');
  #plt.plotsize(100, 30);
  plt.title('Some Smart Title');
  plt.xlabel('Longitude');
  plt.ylabel('Latitude');
  plt.ticks_color('red');
  plt.ticks_style('bold');
  #plt.xlim(-l // 10, l + l // 10);
  #plt.ylim(-1.5, 1.5);
  #xticks = [l * i / (2 * p) for i in range(2 * p + 1)];
  #xlabels = [str(i) + 'π' for i in range(2 * p + 1)];
  #plt.xticks(xticks, xlabels);
  plt.show()

  header = ['Longitude', 'Latitude']
  data = append(y,x,1)

  # Save the data
  with open('coords.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)

    # write the header
    writer.writerow(header)

    # write multiple rows
    writer.writerows(data)

  print('\n\n\n')
  print('Average Coords:   ', avg_lat, ',', avg_long)
  print('CEP:   ', distCoords([0,0],[0.59*(std_long),0.59*(std_lat)]), '\n\n')
  print("Done.\nExiting.")