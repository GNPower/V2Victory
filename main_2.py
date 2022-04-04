#===IMPORT===#
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

#===IMPORT===#

class Plotter():
	def __init__(self,file,delay_ms,translate,angle_rotation):	# Constructor
		print("Plotter constructed")
		# Colours
		# (BGR)
		self.white = (255,255,255)
		self.red = (0,0,255)
		self.orange = (0,255,255)
		self.yellow = (0,128,255)
		self.blue = (255,0,0)
		self.green = (0,255,0)
		self.violet = (255,0,255)
		self.black = (0,0,0)
		self.grass_green = (0,179,0)
		self.curb_grey = (128,128,128)
		self.asphalt_grey = (43,49,51)

		self.step = 0
		self.steps = 0
		self.delay_ms = delay_ms
		self.translate = translate
		self.angle_rotation = angle_rotation
		self.create_track()
		self.get_vehicle_data(file)

	def get_vehicle_data(self,file):
		f = open(file,"r")
		parse = f.read()
		f.close()
		# print(parse)
		self.parsed = parse.split("\n")
		self.steps = len(self.parsed)-1
		# print(self.parsed)

	def create_track(self):
		# print("Creating track")

		# Blank black image
		self.img = np.zeros((1024,1024,3), np.uint8)

		# Grass
		whole_img = np.array([[0,0],[1023,0],[1023,1023],[0,1023],],np.int32)
		whole_img = whole_img.reshape(-1,1,2)
		print(self.step)
		print(self.white)
		print(self.grass_green)
		cv.fillPoly(self.img,[whole_img],self.grass_green)
		
		# Horizontal lanes
		cv.line(self.img,(0,512),(1023,512),self.white,2)
		cv.line(self.img,(0,526),(1023,526),self.asphalt_grey,25)
		cv.line(self.img,(0,498),(1023,498),self.asphalt_grey,25)
		cv.line(self.img,(0,485),(1023,485),self.curb_grey,5)
		cv.line(self.img,(0,539),(1023,539),self.curb_grey,5)

		# Vertical lanes
		cv.line(self.img,(512,0),(512,1023),self.white,2)
		cv.line(self.img,(526,0),(526,1023),self.asphalt_grey,25)
		cv.line(self.img,(498,0),(498,1023),self.asphalt_grey,25)
		cv.line(self.img,(485,0),(485,1023),self.curb_grey,5)
		cv.line(self.img,(539,0),(539,1023),self.curb_grey,5)

		# Intersection
		self.pts_inter = np.array([[462,462],[562,462],[562,562],[462,562]],np.int32)
		self.pts_inter = self.pts_inter.reshape((-1,1,2))
		cv.fillPoly(self.img,[self.pts_inter],self.asphalt_grey)
		cv.rectangle(self.img,(463,462),(562,562),self.white,1)

		# Text
		font = cv.FONT_HERSHEY_SIMPLEX
		cv.putText(self.img,'V2Victory',(1,1000), font, 1, (255,255,255), 2, cv.LINE_AA)

		# N/S Traffic Light
		ns_traffic_light = np.array([[542,212],[592,212],[592,362],[542,362]],np.int32)
		ns_traffic_light = ns_traffic_light.reshape(-1,1,2)
		cv.fillPoly(self.img,[ns_traffic_light],self.asphalt_grey)
		cv.rectangle(self.img,(542,212),(592,362),self.white,5)
		cv.circle(self.img,(567,237),18,self.white,5)
		cv.circle(self.img,(567,287),18,self.white,5)
		cv.circle(self.img,(567,337),18,self.white,5)

		# W/E Traffic Light
		we_traffic_light = np.array([[800,540],[950,540],[950,590],[800,590]],np.int32)
		we_traffic_light = we_traffic_light.reshape(-1,1,2)
		cv.fillPoly(self.img,[we_traffic_light],self.asphalt_grey)
		cv.rectangle(self.img,(800,540),(950,590),self.white,5)
		cv.circle(self.img,(825,565),18,self.white,5)
		cv.circle(self.img,(875,565),18,self.white,5)
		cv.circle(self.img,(925,565),18,self.white,5)

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
		print("pstn_corners")
		print(pstn_corners)
		pstn_corners_np = np.array(pstn_corners,np.int32)
		pstn_corners_np = pstn_corners_np.reshape((-1,1,2))
		# cv.fillPoly(self.img,[pstn_corners_np],self.white)
		cv.rectangle(self.img,(pstn_corners[0][0],pstn_corners[0][1]),(pstn_corners[2][0],pstn_corners[2][1]),self.red,3)

		arrow_start = pstn
		arrow_end = (int(round(pstn[0]+arrow_length*math.cos(heading))),int(round(pstn[1]+arrow_length*math.sin(heading))))
		# print("arrow")
		# print(arrow_start)
		# print(arrow_end)
		cv.line(self.img,arrow_start,arrow_end,self.red,2)
		cv.fillPoly(self.img,[pstn_corners_np],self.white)	# Moved here so the line doesn't fill the box
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
		cv.line(self.img,left_arrow_start,left_arrow_end,self.red,2)
		cv.line(self.img,right_arrow_start,right_arrow_end,self.red,2)

	def update_vehicles(self):
		# print("updated ya")
		# Message format: (time;intersection light;(CarID, x, y, heading); (CarID, x, y, heading); (CarID, x, y, heading))
		# self.img = self.default_img
		self.create_track()
		# print(self.step)
		# print(self.steps)
		if(self.step < self.steps):
			# print(self.parsed[self.step])
			thing = self.parsed[self.step].split(";")
			print(thing)

			num_vehicles = len(thing)-2
			# print(num_vehicles)

			# Update traffic light
			thang = thing[1].strip("()").split(",")
			print(thang)
			# N/S traffic light
			if(thang[0] == "R"):
				cv.circle(self.img,(567,237),16,(0,0,255),-1)
			elif(thang[0] == "Y"):
				cv.circle(self.img,(567,287),16,(0,255,255),-1)
			else:
				cv.circle(self.img,(567,337),16,(0,255,0),-1)

			# W/E traffic light
			if(thang[1] == "R"):
				cv.circle(self.img,(825,565),16,(0,0,255),-1)
			elif(thang[1] == "Y"):
				cv.circle(self.img,(875,565),16,(0,255,255),-1)
			else:
				cv.circle(self.img,(925,565),16,(0,255,0),-1)

			for a in range(2,num_vehicles+2):
				# print(thing[a])
				# print(type(thing[a]))
				thingy = thing[a].strip().split(",")
				print("debug here")
				print(thingy)
				print(int(float(thingy[1].strip("()"))))
				print(int(float(thingy[2].strip("()"))))
				print(int(float(thingy[3].strip("()"))))
				x_coord = 10*int(float(thingy[2].strip("()"))) + self.translate[1]
				y_coord = 10*int(float(thingy[1].strip("()"))) + self.translate[0]
				heading = int(float(thingy[3].strip("()")))
				pstn = (x_coord, y_coord)

				self.draw_vehicle(pstn,(heading+self.angle_rotation)*math.pi/-180)

			self.step = self.step + 1
		else:
			print("Starting over")
			self.step = 0

	def plot(self):
		while(1):
			cv.imshow('img',self.img)
			k = cv.waitKey(self.delay_ms)
			# print(k)
			if(k == 113):
				break
			self.update_vehicles()

		print("Exited")

def main():
	x_translate = 524
	y_translate = 512
	translate = (x_translate,y_translate)
	angle_rotation = 270
	delay_ms = 500
	plot = Plotter("vis.log",delay_ms,translate,angle_rotation)
	plot.plot()

main()

# plot.draw_vehicle((526,700),270*math.pi/180)
# plot.draw_vehicle((498,200),90*math.pi/180)
# plot.draw_vehicle((200,498),180*math.pi/180)
# plot.draw_vehicle((700,526),0*math.pi/180)
# plot.draw_vehicle((250,250),135*math.pi/180)
# plot.draw_vehicle((750,750),340*math.pi/180)