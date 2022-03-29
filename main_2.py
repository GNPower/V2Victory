#===IMPORT===#
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

#===IMPORT===#

class Plotter():
	def __init__(self):	# Constructor
		print("Plotter constructed")
		self.step = 0
		self.steps = 0
		self.create_track()
		self.get_vehicle_data()

	def get_vehicle_data(self):
		f = open("log.txt","r")
		parse = f.read()
		f.close()
		# print(parse)
		self.parsed = parse.split("\n")
		self.steps = len(self.parsed)
		# print(self.parsed)

	def create_track(self):
		print("Creating track")
 
		# Blank black image
		self.img = np.zeros((1024,1024,3), np.uint8)
		
		# Horizontal lanes
		cv.line(self.img,(0,512),(1023,512),(255,255,255),5)
		cv.line(self.img,(0,526),(1023,526),(0,255,0),20)
		cv.line(self.img,(0,498),(1023,498),(0,255,0),20)

		# Vertical lanes
		cv.line(self.img,(512,0),(512,1023),(255,255,255),5)
		cv.line(self.img,(526,0),(526,1023),(255,0,0),20)
		cv.line(self.img,(498,0),(498,1023),(255,0,0),20)

		# Intersection
		self.pts_inter = np.array([[462,462],[562,462],[562,562],[462,562]],np.int32)
		self.pts_inter = self.pts_inter.reshape((-1,1,2))
		cv.fillPoly(self.img,[self.pts_inter],(255,255,0))

		# Text
		font = cv.FONT_HERSHEY_SIMPLEX
		cv.putText(self.img,'V2Victory',(1,1000), font, 1, (255,255,255), 2, cv.LINE_AA)

		self.default_img = self.img

	def draw_vehicle(self,pstn,heading):
		# Draw vehicle portion of code
		# West: y = 498
		# East: y = 526
		# North: x = 498
		# South: x = 526
		# Vehicle size: 15x15
		# Given vehicle position and heading, plot oblong rectangle centred around given point
		# Draw arrow according to heading
		width = 14
		# pstn = (498,700)
		# heading = 270*math.pi/180
		arrow_length = width*3
		arrow_width = round(width/3)
		arrow_tip_length = width
		theta = 15*math.pi/180

		pstn_corners = []
		pstn_corners.append([int(round(pstn[0]-width/2)),int(round(pstn[1]-width/2))])
		pstn_corners.append([int(round(pstn[0]+width/2)),int(round(pstn[1]-width/2))])
		pstn_corners.append([int(round(pstn[0]+width/2)),int(round(pstn[1]+width/2))])
		pstn_corners.append([int(round(pstn[0]-width/2)),int(round(pstn[1]+width/2))])
		# print(pstn_corners)
		pstn_corners_np = np.array(pstn_corners,np.int32)
		pstn_corners_np = pstn_corners_np.reshape((-1,1,2))
		cv.fillPoly(self.img,[pstn_corners_np],(0,255,255))

		arrow_start = pstn
		arrow_end = (int(round(pstn[0]+arrow_length*math.cos(heading))),int(round(pstn[1]+arrow_length*math.sin(heading))))
		# print("arrow")
		# print(arrow_start)
		# print(arrow_end)
		cv.line(self.img,arrow_start,arrow_end,(128,255,255),2)
		left_arrow_start = arrow_end
		right_arrow_start = arrow_end
		left_arrow_end = (int(round(arrow_start[0]+(arrow_length-arrow_tip_length)*math.cos(heading+theta))), int(round(arrow_start[1]+(arrow_length-arrow_tip_length)*math.sin(heading+theta))))
		right_arrow_end = (int(round(arrow_start[0]+(arrow_length-arrow_tip_length)*math.cos(heading-theta))), int(round(arrow_start[1]+(arrow_length-arrow_tip_length)*math.sin(heading-theta))))
		# print("left arrow")
		# print(left_arrow_start)
		# print(left_arrow_end)
		# print("right arrow")
		# print(right_arrow_start)
		# print(right_arrow_end)
		cv.line(self.img,left_arrow_start,left_arrow_end,(128,255,255),2)
		cv.line(self.img,right_arrow_start,right_arrow_end,(128,255,255),2)

	def update_vehicles(self):
		print("updated ya")
		# Message format: (time;(CarID, x, y, heading); (CarID, x, y, heading); (CarID, x, y, heading))
		# self.img = self.default_img
		self.create_track()
		print(self.step)
		print(self.steps)
		if(self.step < self.steps):
			# print(self.parsed[self.step])
			thing = self.parsed[self.step].split(";")
			print(thing)

			num_vehicles = len(thing)-1
			# print(num_vehicles)

			for a in range(1,num_vehicles+1):
				# print(thing[a])
				# print(type(thing[a]))
				thingy = thing[a].strip().split(",")
				# print(thingy)
				print(int(thingy[1].strip("()")))
				print(int(thingy[2].strip("()")))
				print(int(thingy[3].strip("()")))
				self.draw_vehicle((int(thingy[1].strip("()")),int(thingy[2].strip("()"))),int(thingy[3].strip("()"))*math.pi/-180)

			self.step = self.step + 1
		else:
			print("Output complete")

	def plot(self):
		while(1):
			cv.imshow('img',self.img)
			k = cv.waitKey(300)
			# print(k)
			if(k == 113):
				break
			self.update_vehicles()

		print("Exited")

def main():
	plot = Plotter()
	plot.plot()

main()

# plot.draw_vehicle((526,700),270*math.pi/180)
# plot.draw_vehicle((498,200),90*math.pi/180)
# plot.draw_vehicle((200,498),180*math.pi/180)
# plot.draw_vehicle((700,526),0*math.pi/180)
# plot.draw_vehicle((250,250),135*math.pi/180)
# plot.draw_vehicle((750,750),340*math.pi/180)