import serial
import time
import random
import os
import glob
import math
import re
import numpy
import threading
from xyz import XYZ

# shortcut
def createXYZ(port='COM3', baud=9600, limits=[700, 500, 100]):
	xyz = XYZState(limits)
	xyz.startup(port, baud)
	return xyz

def stress_test():
	startTime = time.time()
	
	s = XYZState()
	s.startup()		#TODO devise a mechanism to identify which device is on which port
	#s.moveTo(0, 0, 0)
	
	camera = Device()
	
	times = 10
	
	for n in xrange(times):
		#make a large move forward, then a bunch of little moves back (400 total)
		max_move = 400.0
		steps = 100
		xquantum = max_move / steps
		yquantum = max_move / steps
		zquantum = 0
		
		#make a bunch of little moves in the opposite direction
		for i in xrange(steps):
			s.move_by(xquantum, yquantum, zquantum)
		
		#make a big move
		s.move_by(-xquantum*steps, -yquantum*steps, -zquantum*steps)
			
		print 'lap number', n
		time.sleep(5)				#wait for arm to stop shaking
		camera.saveSnapshot(str(n).zfill(2)+'.jpg')

def random_test():
	startTime = time.time()
	
	s = XYZState()
	s.startup(False)		#TODO devise a mechanism to identify which device is on which port
	#s.moveTo(0, 0, 0)
	
	camera = Device()
	
	max_move = 400
	laps = 20
	steps = 5
	
	for n in xrange(laps):

		#move to a number of random positions
		for i in xrange(steps):
			nextpos = (random.randint(0, max_move), random.randint(0, max_move), 0)
			s.moveTo(*nextpos)
		
		#move to origin
		s.moveTo(0, 0, 0)
			
		print 'lap number', n
		sleeptime = 1 + 3*random.random()	#sleep randomly between 1 and 4 seconds
		time.sleep(sleeptime)				#wait for arm to stop shaking
		camera.saveSnapshot(str(n).zfill(2)+'-'+str(sleeptime)+'.jpg')
	
	#self.moveTo_origin()
	#s.shutdown()
	#endTime = time.time()"""

"""
def calculate_variance(image):
	width = image.shape[0]
	height = image.shape[1]
	intensity_mean = 0
	
	for x in xrange(width):
		for y in xrange(height):
			#rgb = image.getpixel((x, y))
			#intensity_mean += (rgb[0] + rgb[1] + rgb[2]) / 3.0
			intensity_mean += image[x][y]
		
	intensity_mean /= width * height
	intensity_variance = 0
	
	for x in xrange(width):
		for y in xrange(height):
			#rgb = image.getpixel((x, y))
			#intensity = (rgb[0] + rgb[1] + rgb[2]) / 3.0
			intensity = image[x][y]
			d = intensity - intensity_mean
			intensity_variance += d * d
			
	return intensity_variance / intensity_mean
"""

def calculate_contrast(image, num_points = 60, neighbor_pixels = 1):
	""" Returns a number indicating how in-focus the image is, by calculating the contrast at the given (approximate) number of points. These points are in a grid near the center of the image. """
	
	#returns the difference between a pixel's intensity and the intensity of its neighbors 
	def neighbor_variance(image, x, y, neighbor_pixels):
		intensity = image[x][y]
		diffsum = 0
		for x2 in xrange(x-neighbor_pixels, x+neighbor_pixels+1):
			for y2 in xrange(y-neighbor_pixels, y+neighbor_pixels+1):
				diffsum += abs(intensity - image[x2][y2])
		return diffsum
	
	width = image.shape[0]
	height = image.shape[1]
	distance_between = int(math.sqrt(width * height / 4.0 / float(num_points)))	#yields approximately num_points but not exactly

	contrast = 0
	for x in xrange(distance_between + width/4, width*3/4, distance_between+1):
		for y in xrange(distance_between + height/4, height*3/4, distance_between+1):
			contrast += neighbor_variance(image, x, y, neighbor_pixels)
	
	#horizontal line
	#for x in xrange(neighbor_pixels, width-neighbor_pixels):
	#	contrast += neighbor_variance(image, x, height/2, neighbor_pixels)
	
	#vertical line
	#for y in xrange(neighbor_pixels, height-neighbor_pixels):
	#	contrast += neighbor_variance(image, width/2, y, neighbor_pixels)
		
	return contrast

def autofocus(xyz, camera, minpos=0, maxpos=50):

	#create a matrix of the pixel intensities in the image, which is faster to access than the image
	def image_to_matrix(image):
		pixels = numpy.empty(image.size)
		for x in xrange(image.size[0]):
			for y in xrange(image.size[1]):
				rgb = image.getpixel((x, y))
				pixels[x][y] = (rgb[0] + rgb[1] + rgb[2]) / 3.0
		return pixels

	#debug code
	folder = 'autofocus_debug'
	try:
		os.mkdir(folder)
	except:
		files = glob.glob(os.path.join(folder, '*'))
		for f in files: os.remove(f)
	
	speed = 100
	#if cuspos is None: curpos = minpos

	num_steps = 5
	min_focal_step = .1
	
	#loop until our focal adjustments are smaller than a threshold
	while True:
		zinc = (maxpos - minpos) / float(num_steps)
		if zinc < min_focal_step:
			break		#done
		
		focuses = []	#list of pairs: (z-position, contrast-score)
		best_index = 0
		z = minpos
		xyz.moveTo(z=z)
		time.sleep(3)
		
		while z <= maxpos:
			xyz.moveTo(z=z, speed=speed)#*maxpos-minpos)
			time.sleep(1)	#wait for camera to stop shaking
			image = camera.getImage()
			pixels = image_to_matrix(image)
			
			#calculate contrast and record the height of the best contrast
			contrast = calculate_contrast(pixels)
			if len(focuses) > 0 and contrast > focuses[best_index][1]:
				best_index = len(focuses)
			
			focuses.append((z, contrast))
				
			#save the image for debugging
			image.save(os.path.join(folder, str(z) + '-n1.' + str(contrast) + '.jpg'))
			print z, contrast
			
			z += zinc
			
		#find new range to explore
		if best_index == 0:
			maxpos = focuses[1][0]			#TODO check to make sure this exists...
		elif best_index == len(focuses)-1:
			minpos = focuses[-2][0]			#TODO check to make sure this exists...
		else:
			minpos = focuses[best_index-1][0]
			maxpos = focuses[best_index+1][0]
	
	print 'best position: ', focuses[best_index]
	xyz.moveTo(z=focuses[best_index][0])
	pdb.set_trace()
		
"""	
	best_pos = minpos
	best_contrast = 0
	
	#mm_per_second = 5
	#pics_per_second = 10
	#start_time = time.time()
	
	xyz.moveTo(z=minpos, speed=speed)#, 60*mm_per_second, False)
	
	for z in xrange(minpos, maxpos, 1):
		xyz.moveTo(z=z, speed=speed)
		#time.sleep(1.0 / pics_per_second)
		#time_pos = time.time() - start_time
		time.sleep(2)
		time_pos = z
		image = camera.getImage()
		pixels = image_to_matrix(image)
		
		#calculate contrast and record the height of the best contrast
		neighbor1 = calculate_contrast(pixels)
		if neighbor1 > best_contrast:
			best_contrast = neighbor1
			best_pos = z
			
		#save the image for debugging
		image.save(os.path.join(folder, str(time_pos) + '-n1.' + str(neighbor1) + '.jpg'))
		print time_pos, neighbor1
	
	print 'best position: ', best_pos
	xyz.moveTo(z=best_pos)
"""

def random_timed_test(xyz, camera):
	
	def worker_func(duration, done_signal, xyz, camera):
		speed = 10000
		max_move_x = 700
		max_move_y = 500
		max_move_z = 60
		random_stops_per_lap = 5
		lap_count = 0

		folder = str(start_time) + '/'
		os.mkdir(folder)

		camera.saveSnapshot(folder + str(lap_count).zfill(5) + '.jpg')
		
		while duration < 0 or time.time() < start_time + duration:

			#move to a number of random positions
			for i in xrange(random_stops_per_lap):
				if done_signal.is_set(): break
				x = random.randint(0, max_move_x)
				y = random.randint(0, max_move_y)
				z = random.randint(0, max_move_z)
				xyz.moveTo(x, y, z, speed)
			
			xyz.moveTo_origin(speed)
			if done_signal.is_set(): break
			
			#wait for arm to stop shaking and save a picture to disk
			print 'lap number', lap_count
			time.sleep(5)				#wait seconds for arm to stop shaking
			lap_count += 1
			camera.saveSnapshot(folder + str(lap_count).zfill(5) + '.jpg')

		xyz.moveTo_origin(speed)
		if done_signal.is_set(): print '\nFinished.'
		else: print '\nFinished -- press Enter to exit.'
		exit()							#janky, should rejoin main thread
	
	try:
		duration = float(raw_input('Enter a number of minutes to run, or -1 to run FOREVER: '))
	except:
		print '\nError: invalid number.  Exiting.'
		exit()
		
	if duration == 0:
		print '\n0 entered -- exiting.'
		exit()
		
	duration *= 60			#minutes convert to seconds
	start_time = time.time()
	
	#camera = Device()
	#xyz = XYZState()
	#xyz.startup(False)
	
	done_signal = threading.Event()
	thread = threading.Thread(target = worker_func, args = (duration, done_signal, xyz, camera))
	thread.start()
	
	raw_input('\n		 ******** Press Enter to end. ********\n\n')
	print '\n\nEnter received -- shuttting down.\n\n'
	done_signal.set()
	thread.join()
