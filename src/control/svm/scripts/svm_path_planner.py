import numpy as np
import matplotlib.pyplot as plt
from sklearn.svm import SVC
# from svm_plot import plot_contours
from sklearn.metrics import accuracy_score
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class SvmPathPlanner():

	def __init__(self):
		self.N = 1
		self.clf = SVC(C = 100, gamma = 0.01, kernel = 'rbf', max_iter = -1)
		
	def generateCenterLine(self, cones):
		yellow_cones = []
		blue_cones = []
		cone_x = []
		cone_y = []

		#print("Numero de cones :", len(cones.cone_detections))
		for cone in cones.cone_detections:
			if cone.color == 1:#yellow
				yellow_cones.append([cone.position.x, cone.position.y])
			if cone.color == 0:#blue
				blue_cones.append([cone.position.x, cone.position.y])
			cone_x.append(cone.position.x)
			cone_y.append(cone.position.y)
		
		xmin = np.floor(np.amin(cone_x, axis=0))
		xmax = np.ceil(np.amax(cone_x, axis=0))
		ymin = np.floor(np.amin(cone_y, axis=0))
		ymax = np.ceil(np.amax(cone_y, axis=0))

		# Add cone to car side
		# yellow_cones.append([-1,-2])
		# blue_cones.append([-1,2])

		yellow_cones = np.array(yellow_cones)
		blue_cones = np.array(blue_cones)
		# print("Nº azuis: ",len(blue_cones))
		# print("Nº amarelos: ",len(yellow_cones))
		class_yellow = np.full(len(yellow_cones), 1)
		class_blue = np.full(len(blue_cones), -1)

		color_yellow = np.full(len(yellow_cones), "#FFF300")
		color_blue = np.full(len(blue_cones), "#3399FF")

		all_cones = np.concatenate((yellow_cones, blue_cones))
		all_class = np.concatenate((class_yellow, class_blue))
		all_colors = np.concatenate((color_yellow,color_blue))
		# Sensors limit
		xlim = [xmin, xmax]
		ylim = [ymin, ymax]

		step = 5 # distance between points is 1/step	

		grid_size_x = int((abs(xlim[0]) + abs(xlim[1]))*step)
		grid_size_y = int((abs(ylim[0]) + abs(ylim[1]))*step)

		#create grid to evaluate model
		xx = np.linspace(xlim[0], xlim[1], grid_size_x)
		yy = np.linspace(ylim[0], ylim[1], grid_size_y)
		self.YY, self.XX = np.meshgrid(yy, xx)
		self.xy = np.vstack([self.XX.ravel(), self.YY.ravel()]).T

		self.clf.fit(all_cones,all_class)

		# print("PEDRO DIZ: MAMAS")
		
		X = all_cones
		y = all_class
		
		plt.cla() # to stabilize runtime
		# plot point in the window
		plt.scatter(X[:, 0], X[:, 1], c=all_colors, s=5*4, cmap=plt.cm.Paired)
				
		# plot the decision function
		ax = plt.gca()
		ax.set_aspect('equal', adjustable='box')
				
		Z = self.clf.decision_function(self.xy).reshape(self.XX.shape)

		# plot decision boundary and margins
		contour = ax.contour(self.XX, self.YY, Z, colors='k', levels=[-1, 0, 1], alpha=0.5,
				linestyles=['--', '-', '--'])
		
		# plt.show()
		path_left = Path()
		path_centerline = Path()
		path_right = Path()
		paths = [path_left, path_centerline, path_right]
		for i in range(0, len(paths)):			
			p = contour.collections[i].get_paths()[0]
			v = p.vertices
			x = v[:,0]
			y = v[:,1]

			line = np.vstack([x, y]).T
			path = Path()
			path.header = cones.header
			path.header.frame_id = 'map'
			counter = 0;
			for point in line:
				counter += 1
				if counter == 10:
					pose = PoseStamped()
					pose.header = cones.header
					pose.pose.position.x = point[0]
					pose.pose.position.y = point[1]
					path.poses.append(pose)
					counter = 0
			path.poses.append(path.poses[0])
			paths[i]=path

		return paths	