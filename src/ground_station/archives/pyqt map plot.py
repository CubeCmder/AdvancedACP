# in UI __init__
#self.setNewFigure('plotA', self.gridLayout_plotA, True)
#self.setNewFigure('GPS_Acc', self.gridLayout_plotB, True)
#self.setNewFigure('Altitude', self.gridLayout_plotC, True)
#fig = self.PLOT_FIGURES['Altitude']['fig']
#ax = fig.gca()
#ax.plot([], [], '-o')
#ax.grid()
#self.PLOT_FIGURES['Altitude']['canvas'].draw()

#lon1 = -73.78810
#lon2 = -73.77935
#lat1 = 45.51600
#lat2 = 45.52080
#fig = self.PLOT_FIGURES['plotA']['fig']
#ax = fig.gca()
#ax.set_xlim(round(lon1,5), round(lon2,5))
#ax.set_ylim(round(lat1,5), round(lat2,5))
#ax.xaxis.set_major_locator(LinearLocator(numticks = 7))
#ax.xaxis.set_major_formatter(FormatStrFormatter('% 1.5f'))
#ax.yaxis.set_major_locator(LinearLocator(numticks = 7))
#ax.yaxis.set_major_formatter(FormatStrFormatter('% 1.5f'))
#ax.set_xticks(np.arange(lon1, lon2, (lon2-lon1)/4))
#ax.set_yticks(np.arange(lat1, lat2, (lat2-lat1)/4))
#ax.plot([],[],'o')
#ax.imshow(plt.imread('map.png'), extent=(lon1, lon2, lat1, lat2), aspect=(1756/2252)**-1)  # Load the image to matplotlib plot.
#ax.grid()
#self.PLOT_FIGURES['plotA']['canvas'].draw()

#fig = self.PLOT_FIGURES['GPS_Acc']['fig']
#ax = fig.gca()
#ax.plot([], [], 'ro')
#ax.grid()
#self.PLOT_FIGURES['GPS_Acc']['canvas'].draw()