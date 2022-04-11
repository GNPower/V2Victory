#===IMPORT===#
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import sys

#===IMPORT===#

class Plotter():
	def __init__(self, file, delay_ms, scale):	# Constructor
		# Colours (BGR)
		self.white = (255, 255, 255)
		self.red = (0, 0, 255)
		self.orange = (0, 255, 255)
		self.yellow = (0, 128, 255)
		self.blue = (255, 0, 0)
		self.green = (0, 255, 0)
		self.violet = (255, 0, 255)
		self.black = (0, 0, 0)
		self.grass_green = (0, 179, 0)
		self.curb_grey = (128, 128, 128)
		self.asphalt_grey = (43, 49, 51)

		self.step = 0
		self.steps = 0
		self.delay_ms = delay_ms
		self.scale = scale
		
		self.parse_file(file)
		self.create_track()

	def parse_file(self, file):
		with open(file) as f:
			lines = f.readlines()
		int_params = lines[0].split()
		#get params
		self.int_length = int(int_params[0])
		if int(int_params[1]) == 0:
			self.is_signal = True
		else:
			self.is_signal = False

		self.parsed = lines[1:]
		self.steps = len(self.parsed) - 1

	def create_track(self):
		track_x = 1024; #vertical direction
		track_y = 1024; #horizontal direction
		self.center_x = int(track_x/2)
		self.center_y = int(track_y/2)
		self.end_x = track_x - 1
		self.end_y = track_y - 1

		#Blank Black Image
		self.img = np.zeros((track_y, track_x, 3), np.uint8)

		#Grass
		whole_img = np.array([[[0, 0], [self.end_y, 0], [self.end_y, self.end_x], [0, self.end_x]]], np.int32)
		cv.fillPoly(self.img, whole_img, self.grass_green)

		center_width = 2
		curb_width = int(self.scale*self.int_length/4)
		road_width = int(self.scale*self.int_length/2)

		#Curbs
		curb_offset = int((center_width + curb_width)/2 + road_width)
		cv.line(self.img, (0, self.center_x - curb_offset), (self.end_y, self.center_x - curb_offset), self.curb_grey, curb_width)
		cv.line(self.img, (0, self.center_x + curb_offset), (self.end_y, self.center_x + curb_offset), self.curb_grey, curb_width)
		cv.line(self.img, (self.center_y - curb_offset, 0), (self.center_y - curb_offset, self.end_x), self.curb_grey, curb_width)
		cv.line(self.img, (self.center_y + curb_offset, 0), (self.center_y + curb_offset, self.end_x), self.curb_grey, curb_width)

		#Center Lines
		cv.line(self.img, (0, self.center_x), (self.end_y, self.center_x), self.white, center_width)
		cv.line(self.img, (self.center_y, 0), (self.center_y, self.end_x), self.white, center_width)

		#Roads
		road_offset = int((center_width + road_width)/2)
		cv.line(self.img, (0, self.center_x - road_offset), (self.end_y, self.center_x - road_offset), self.asphalt_grey, road_width)
		cv.line(self.img, (0, self.center_x + road_offset), (self.end_y, self.center_x + road_offset), self.asphalt_grey, road_width)
		cv.line(self.img, (self.center_y - road_offset, 0), (self.center_y - road_offset, self.end_x), self.asphalt_grey, road_width)
		cv.line(self.img, (self.center_y + road_offset, 0), (self.center_y + road_offset, self.end_x), self.asphalt_grey, road_width)

		# Intersection
		self.pts_inter = np.array([[[self.center_y - road_width, self.center_x - road_width], [self.center_y - road_width, self.center_x + road_width], [self.center_y + road_width, self.center_x - road_width], [self.center_y + road_width, self.center_x + road_width]]], np.int32)
		cv.fillPoly(self.img, self.pts_inter, self.asphalt_grey)

		#Stop Lines
		cv.line(self.img, (self.center_y - road_width, self.center_x), (self.center_y - road_width, self.center_x + road_width), self.white, center_width)
		cv.line(self.img, (self.center_y + road_width, self.center_x), (self.center_y + road_width, self.center_x - road_width), self.white, center_width)
		cv.line(self.img, (self.center_y, self.center_x + road_width), (self.center_y + road_width, self.center_x + road_width), self.white, center_width)
		cv.line(self.img, (self.center_y, self.center_x - road_width), (self.center_y - road_width, self.center_x - road_width), self.white, center_width)

		# Text
		font = cv.FONT_HERSHEY_SIMPLEX
		cv.putText(self.img, 'V2Victory', (0, self.end_x - 10), font, 1, self.white, 2, cv.LINE_AA)

		full_curb_offset = int(center_width/2 + road_width + curb_width)
		self.light_w = 50
		self.light_l = 150
		light_thickness = 5

		self.circle_r = 18
		self.circle_width = 5

		if self.is_signal:
			# N/S Traffic Light
			ns_traffic_light = np.array([[[self.center_y + full_curb_offset, self.center_x - 2*self.light_l], [self.center_y + full_curb_offset + self.light_w, self.center_x - 2*self.light_l], [self.center_y + full_curb_offset + self.light_w, self.center_x - self.light_l], [self.center_y + full_curb_offset, self.center_x - self.light_l]]], np.int32)
			cv.fillPoly(self.img, ns_traffic_light, self.asphalt_grey)
			cv.rectangle(self.img, (self.center_y + full_curb_offset + int(light_thickness/2), self.center_x - 2*self.light_l), (self.center_y + full_curb_offset + int(light_thickness/2) + self.light_w, self.center_x - self.light_l), self.white, light_thickness)
			self.ns_circle_x = self.center_x - self.circle_r - light_thickness - self.light_l
			self.ns_circle_y = self.center_y + full_curb_offset + int(self.light_w/2)
			cv.circle(self.img, (self.ns_circle_y, self.ns_circle_x - 2*self.light_w), self.circle_r, self.white, self.circle_width)
			cv.circle(self.img, (self.ns_circle_y, self.ns_circle_x - self.light_w), self.circle_r, self.white, self.circle_width)
			cv.circle(self.img, (self.ns_circle_y, self.ns_circle_x), self.circle_r, self.white, self.circle_width)

			# W/E Traffic Light
			we_traffic_light = np.array([[[self.center_y + self.light_l, self.center_x + full_curb_offset], [self.center_y + 2*self.light_l, self.center_x + full_curb_offset], [self.center_y + 2*self.light_l, self.center_x + full_curb_offset + self.light_w], [self.center_y + self.light_l, self.center_x + full_curb_offset + self.light_w]]], np.int32)
			cv.fillPoly(self.img, we_traffic_light, self.asphalt_grey)
			cv.rectangle(self.img, (self.center_y + self.light_l, self.center_x + full_curb_offset), (self.center_y + 2*self.light_l, self.center_x + full_curb_offset + self.light_w), self.white, light_thickness)
			self.we_circle_x = self.center_x + full_curb_offset + int(self.light_w/2)
			self.we_circle_y = self.center_y + self.circle_r + light_thickness + self.light_l
			cv.circle(self.img, (self.we_circle_y, self.we_circle_x), self.circle_r, self.white, self.circle_width)
			cv.circle(self.img, (self.we_circle_y + self.light_w, self.we_circle_x), self.circle_r, self.white, self.circle_width)
			cv.circle(self.img, (self.we_circle_y + 2*self.light_w, self.we_circle_x), self.circle_r, self.white, self.circle_width)

		else: #stop sign
			#draw circles with stop in them to simplify
			self.circle_r = 50
			ne_circle = (self.center_y + full_curb_offset + self.circle_r, self.center_x - full_curb_offset - self.circle_r)
			sw_circle = (self.center_y - full_curb_offset - self.circle_r, self.center_x + full_curb_offset + self.circle_r)
			cv.circle(self.img, ne_circle, self.circle_r, self.red, -1)
			cv.circle(self.img, sw_circle, self.circle_r, self.red, -1)

			font = cv.FONT_HERSHEY_SIMPLEX
			cv.putText(self.img, 'STOP', (ne_circle[0] - 40, ne_circle[1] + 10), font, 1, self.white, 2, cv.LINE_AA)
			cv.putText(self.img, 'STOP', (sw_circle[0] - 40, sw_circle[1] + 10), font, 1, self.white, 2, cv.LINE_AA)

		self.default_img = self.img

	def draw_vehicle(self, pstn, heading):
		# Draw vehicle portion of code
		# Given vehicle position and heading, plot oblong rectangle centred around given point
		# Draw arrow according to heading
		veh_width = 3*self.scale
		veh_thick = 2
		arrow_angle = 15

		arrow_length = veh_width*3
		arrow_width = round(veh_width/3)
		arrow_tip_length = veh_width
		theta = arrow_angle*math.pi/180

		pstn_corners = []
		pstn_corners.append([int(round(self.center_y - pstn[1] - veh_width/2)), int(round(self.center_x - pstn[0] - veh_width/2))])
		pstn_corners.append([int(round(self.center_y - pstn[1] + veh_width/2)), int(round(self.center_x - pstn[0] - veh_width/2))])
		pstn_corners.append([int(round(self.center_y - pstn[1] + veh_width/2)), int(round(self.center_x - pstn[0] + veh_width/2))])
		pstn_corners.append([int(round(self.center_y - pstn[1] - veh_width/2)), int(round(self.center_x - pstn[0] + veh_width/2))])
		pstn_corners_np = np.array(pstn_corners, np.int32)
		cv.rectangle(self.img, (pstn_corners[0][0], pstn_corners[0][1]), (pstn_corners[2][0], pstn_corners[2][1]), self.red, veh_thick)

		arrow_start = (self.center_y - pstn[1], self.center_x - pstn[0])
		arrow_end = (int(round(self.center_y - pstn[1] - arrow_length*math.sin(heading))), int(round(self.center_x - pstn[0] - arrow_length*math.cos(heading))))
		cv.line(self.img, arrow_start, arrow_end, self.red, 2)
		cv.fillPoly(self.img, [pstn_corners_np], self.white)	# Moved here so the line doesn't fill the box
		left_arrow_start = arrow_end
		right_arrow_start = arrow_end
		left_arrow_end = (int(round(arrow_start[0] - (arrow_length - arrow_tip_length)*math.sin(heading + theta))), int(round(arrow_start[1] - (arrow_length - arrow_tip_length)*math.cos(heading + theta))))
		right_arrow_end = (int(round(arrow_start[0] - (arrow_length - arrow_tip_length)*math.sin(heading - theta))), int(round(arrow_start[1] - (arrow_length - arrow_tip_length)*math.cos(heading - theta))))
		cv.line(self.img, left_arrow_start, left_arrow_end, self.red, 2)
		cv.line(self.img, right_arrow_start, right_arrow_end, self.red, 2)

	def update_plot(self):
		self.create_track()
		if(self.step < self.steps):
			split_line = self.parsed[self.step].split(";")

			#write time
			sim_time = split_line[0]
			font = cv.FONT_HERSHEY_SIMPLEX
			time_str = 'Time: '+ sim_time
			cv.putText(self.img, time_str, (self.end_y - 200, 50), font, 1, self.white, 2, cv.LINE_AA)

			num_vehicles = len(split_line) - 2

			if self.is_signal:
				# Update traffic light
				intersection = split_line[1].split(",")
				# N/S traffic light
				filled_r = self.circle_r - int(self.circle_width/2)
				if(intersection[0] == "R"):
					cv.circle(self.img, (self.ns_circle_y, self.ns_circle_x - 2*self.light_w), filled_r, self.red, -1)
				elif(intersection[0] == "Y"):
					cv.circle(self.img, (self.ns_circle_y, self.ns_circle_x - self.light_w), filled_r, self.yellow, -1)
				else:
					cv.circle(self.img, (self.ns_circle_y, self.ns_circle_x), filled_r, self.green, -1)

				# W/E traffic light
				if(intersection[1] == "R"):
					cv.circle(self.img, (self.we_circle_y, self.we_circle_x), filled_r, self.red, -1)
				elif(intersection[1] == "Y"):
					cv.circle(self.img, (self.we_circle_y + self.light_w, self.we_circle_x), filled_r, self.yellow, -1)
				else:
					cv.circle(self.img, (self.we_circle_y + 2*self.light_w, self.we_circle_x), filled_r, self.green, -1)

			for i in range(2, num_vehicles + 2):
				vehicle = split_line[i].split(",")
				x_coord = self.scale*int(float(vehicle[1]))
				y_coord = self.scale*int(float(vehicle[2]))
				heading = int(float(vehicle[3]))
				pstn = (x_coord, y_coord)

				self.draw_vehicle(pstn, (heading + 0)*math.pi/180)

			self.step = self.step + 1
		else:
			sys.exit()

	def plot(self):
		while(1):
			cv.imshow('img',self.img)
			k = cv.waitKey(self.delay_ms)
			if(k == 113):
				break
			self.update_plot()

def main():
	delay_ms = 50
	scale = 6
	plot = Plotter("vis.log", delay_ms, scale)
	plot.plot()

main()