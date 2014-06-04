import serial
import time
import random
import os
import glob
import math
import re
import numpy
import threading
#from VideoCapture import Device


class Claw:
    def __init__(self):
        pass
    
    def startup(self, port, baud=9600):
        """ initiate connection to the microcontroller """
        self.s = serial.Serial(port, baud)
        self.s.readline()            # block until ready

    def grip(self, angle):
        self.s.write('c' + str(angle) + '\n')
        self.s.readline()

    def tilt(self, angle):
        self.s.write('t' + str(angle) + '\n')
        self.s.readline()
        

# motor driver for XYZ hardware
# dimensions defined as follows:
# X-axis is perpendicular to the cross bar; Z is perpendicular to the floor
class XYZState:
    __default_speed__ = 12000
    __pos_pattern__ = re.compile('.Pos:(-?\d+[.]\d+),(-?\d+[.]\d+),(-?\d+[.]\d+)')
    
    def __init__(self, limits = [0, 0, 0]):
        self.s = None                            # serial port object
        self.abs_move = None
        
        # vectors follow the format [X, Y, Z] where Z is the vertical
        # self.steps_per_cm = [40, 40, 40]
        # number of steps per centimeter in each dimension
        self.pos = [0, 0, 0]                    # current position
        self.origin = [0, 0, 0]                    # minimum coordinates
        self.limits = list(limits)                # maximum coordinates
    
    def startup(self, port, baud=9600, numInitialReads=3):
        """ initiate connection to the arm microcontroller """
        self.s = serial.Serial(port, baud)
	time.sleep(2)
        # block by reading: GRBL will reply when it is ready
        #for i in xrange(numInitialReads):
        #    self.s.readline()

        self.s.write("$X\n")        # unlock
        self.s.readline()
        self.s.readline()
        self.set_movement_mode(True)
        #self.home_origin()
        self.set_origin()            # set the current position as the origin (GRBL sometimes starts with z not 0)
        #TODO?
        #self.calibrate()
        
    def set_movement_mode(self, absolute_mode = True):
        if self.abs_move == absolute_mode:
            return        # do nothing
        
        self.abs_move = absolute_mode
        if absolute_mode:
            self.s.write("G90\n")        # absolute movement mode
        else:
            self.s.write("G91\n")        # relative movement mode
        self.s.readline()
        
    def shutdown(self):
        self.s.close()
        
    def position(self):
        return list(self.pos)    # copy the list so caller can't unintentionally modify our internal one
        
    def block_until_idle(self):
        pollcount = 0
        while True:
            self.s.write("?")
            status = self.s.readline()
            if status.startswith('<Idle'): break
            pollcount += 1
            time.sleep(.01)        # poll every 10 ms
        # print 'pollcount = {}\n'.format(pollcount)
        print status
        
    def readpos(self):
        self.s.write("?")
        status = self.s.readline()
        print status
        #tuples = self.__pos_pattern__.findall(status)
        #self.pos = list(tuples[1])
        #print self.pos
   
    def move_to(self, x=None, y=None, z=None, speed=__default_speed__, block_until_complete=True):
        """ move to a given absolute position, and return when movement completes """
        if x is None and y is None and z is None: return
        
        self.set_movement_mode(absolute_mode = True)
        
        #check against self.limits
        
        gcode = 'G1'
        letters = 'xyz'
        pos = (x, y, z)
        newpos = list(self.pos)
        
        #create gcode string and update position list for each argument that isn't None
        for i in range(3):
            if pos[i] is not None:
                #if pos[i] < 0 or pos[i] >= self.limits[i]:
                #    #error
                #    print self.limits[i] +'=' + pos[i] + ' position outside limit: \n'
                #    return
                gcode += ' ' + letters[i] + str(pos[i])
                newpos[i] = pos[i]

        gcode += ' f' + str(speed)
        gcode += '\n'
        
        self.s.write(gcode)
        self.s.readline()
        
        #update position if success
        # TODO check to make sure it's actually a success
        self.pos = newpos
        
        if block_until_complete:
            self.block_until_idle()

    def move_rel(self, dx=None, dy=None, dz=None, speed=__default_speed__, block_until_complete=True):
        """ move a given distance, and return when movement completes
        :param dx, dy, dz: distance to move
        :param speed: units uncertain
        :param block_until_complete: whether to return immediately, or wait for the movement to complete
        """

        self.set_movement_mode(absolute_mode = False)

        gcode = 'G1'
        letters = 'xyz'
        d = (dx, dy, dz)
        newpos = list(self.pos)
        
        #create gcode string and update position list for each argument that isn't None (TODO: if successful?)
        for i in range(3):
            if d[i] is not None:
                gcode += ' ' + letters[i] + str(d[i])
                newpos[i] += d[i]
        
        gcode += ' f' + str(speed)
        gcode += '\n'
        
        self.s.write(gcode)
        self.s.readline()
        
        #update position if success
        # TODO check to make sure it's actually a success
        self.pos = newpos
        
        if block_until_complete:
            self.block_until_idle()

    def move_to_origin(self, speed=__default_speed__):
        """ move to starting position, and return when movement completes """
        self.move_to(*self.origin, speed=speed)
        self.pos = list(self.origin)
            
    def set_origin(self, x=None, y=None, z=None):
        """set current position to be (0,0,0), or a custom (x,y,z)"""
        
        #grab current position if not supplied
        if x is None: x = 0#self.pos[0]
        if y is None: y = 0#self.pos[1]
        if z is None: z = 0#self.pos[2]
        
        self.pos = [x, y, z]
        
        gcode = "G92 x{} y{} z{}\n".format(x, y, z)
        self.s.write(gcode)
        self.s.readline()

    #def move_to_(self, vector):
    #    self.move_to(vector.x, vector.y, vector.z)
    #    response_topic.publish(control_msgs.status(True))

def defaultXYZ(port='COM3', baud=9600, limits=[700, 500, 100]):
    xyz = XYZState(limits)
    xyz.startup(port, baud)
    return xyz

def stress_test():
    startTime = time.time()
    
    s = XYZState()
    s.startup()        #TODO devise a mechanism to identify which device is on which port
    #s.move_to(0, 0, 0)
    
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
        time.sleep(5)                #wait for arm to stop shaking
        camera.saveSnapshot(str(n).zfill(2)+'.jpg')

def random_test():
    startTime = time.time()
    
    s = XYZState()
    s.startup(False)        #TODO devise a mechanism to identify which device is on which port
    #s.move_to(0, 0, 0)
    
    camera = Device()
    
    max_move = 400
    laps = 20
    steps = 5
    
    for n in xrange(laps):

        #move to a number of random positions
        for i in xrange(steps):
            nextpos = (random.randint(0, max_move), random.randint(0, max_move), 0)
            s.move_to(*nextpos)
        
        #move to origin
        s.move_to(0, 0, 0)
            
        print 'lap number', n
        sleeptime = 1 + 3*random.random()    #sleep randomly between 1 and 4 seconds
        time.sleep(sleeptime)                #wait for arm to stop shaking
        camera.saveSnapshot(str(n).zfill(2)+'-'+str(sleeptime)+'.jpg')
    
    #self.move_to_origin()
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
    distance_between = int(math.sqrt(width * height / 4.0 / float(num_points)))    #yields approximately num_points but not exactly

    contrast = 0
    for x in xrange(distance_between + width/4, width*3/4, distance_between+1):
        for y in xrange(distance_between + height/4, height*3/4, distance_between+1):
            contrast += neighbor_variance(image, x, y, neighbor_pixels)
    
    #horizontal line
    #for x in xrange(neighbor_pixels, width-neighbor_pixels):
    #    contrast += neighbor_variance(image, x, height/2, neighbor_pixels)
    
    #vertical line
    #for y in xrange(neighbor_pixels, height-neighbor_pixels):
    #    contrast += neighbor_variance(image, width/2, y, neighbor_pixels)
        
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
            break        #done
        
        focuses = []    #list of pairs: (z-position, contrast-score)
        best_index = 0
        z = minpos
        xyz.move_to(z=z)
        time.sleep(3)
        
        while z <= maxpos:
            xyz.move_to(z=z, speed=speed)#*maxpos-minpos)
            time.sleep(1)    #wait for camera to stop shaking
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
            maxpos = focuses[1][0]            #TODO check to make sure this exists...
        elif best_index == len(focuses)-1:
            minpos = focuses[-2][0]            #TODO check to make sure this exists...
        else:
            minpos = focuses[best_index-1][0]
            maxpos = focuses[best_index+1][0]
    
    print 'best position: ', focuses[best_index]
    xyz.move_to(z=focuses[best_index][0])
    pdb.set_trace()
        
"""    
    best_pos = minpos
    best_contrast = 0
    
    #mm_per_second = 5
    #pics_per_second = 10
    #start_time = time.time()
    
    xyz.move_to(z=minpos, speed=speed)#, 60*mm_per_second, False)
    
    for z in xrange(minpos, maxpos, 1):
        xyz.move_to(z=z, speed=speed)
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
    xyz.move_to(z=best_pos)
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
                xyz.move_to(x, y, z, speed)
            
            xyz.move_to_origin(speed)
            if done_signal.is_set(): break
            
            #wait for arm to stop shaking and save a picture to disk
            print 'lap number', lap_count
            time.sleep(5)                #wait seconds for arm to stop shaking
            lap_count += 1
            camera.saveSnapshot(folder + str(lap_count).zfill(5) + '.jpg')

        xyz.move_to_origin(speed)
        if done_signal.is_set(): print '\nFinished.'
        else: print '\nFinished -- press Enter to exit.'
        exit()                            #janky, should rejoin main thread
    
    try:
        duration = float(raw_input('Enter a number of minutes to run, or -1 to run FOREVER: '))
    except:
        print '\nError: invalid number.  Exiting.'
        exit()
        
    if duration == 0:
        print '\n0 entered -- exiting.'
        exit()
        
    duration *= 60            #minutes convert to seconds
    start_time = time.time()
    
    #camera = Device()
    #xyz = XYZState()
    #xyz.startup(False)
    
    done_signal = threading.Event()
    thread = threading.Thread(target = worker_func, args = (duration, done_signal, xyz, camera))
    thread.start()
    
    raw_input('\n         ******** Press Enter to end. ********\n\n')
    print '\n\nEnter received -- shuttting down.\n\n'
    done_signal.set()
    thread.join()
    
    
global xyz
global claw
global liquid
#global camera

xyz = XYZState()
xyz.startup('/dev/ttyACM2')

claw = Claw()
#claw.startup('/dev/ttyACM2')

liquid = XYZState()
liquid.startup('/dev/ttyACM1', 9600, 1)

def shutdown():
    claw.shutdown()
    liquid.shutdown()
    xyz.shutdown()

claw_tilt_up = 80
claw_tilt_down = 16
claw_grip_open = 120
claw_grip_closed = 90
#claw.tilt(claw_tilt_up)     #put claw in default position
#claw.grip(claw_grip_open)

safe_z = 0
plate_safe_z = -50
plate_disp_z = -60
resevoir_draw_z = -72

clear_resevoir_pos = [0, 150, safe_z]
red_resevoir_pos = [0, 0, safe_z]
blue_resevoir_pos = [0, 320, safe_z]
plate_well_top_left = [-230, 250, safe_z]

claw_plate_raise_z = 40
incubator_pos_in = [-320, 250, claw_plate_raise_z]
incubator_pos_out = [-120, 250, claw_plate_raise_z]

well_delta_x = 8.3
well_delta_y = 8.8
num_wells_x = 1
num_wells_y = 2

liquid_per_well = 5.0
total_liquid = num_wells_x * num_wells_y * liquid_per_well

def pippette(resevoir_pos):

    #helper for pipetting: move down, dispense, move up
    def dispense():
        xyz.move_to(z=plate_disp_z)
        liquid.move_rel(liquid_per_well+1.5)
        time.sleep(2)                       #wait for liquid to fall out
        xyz.move_to(z=plate_safe_z)

    #draw liquid
    xyz.move_to(*resevoir_pos)
    xyz.move_to(z=resevoir_draw_z)
    liquid.move_rel(-total_liquid)  #TODO store/update how much liquid held
    time.sleep(2)
    xyz.move_to(z=safe_z)

    #move to plate and dispense into wells
    xyz.move_to(*plate_well_top_left)
    xyz.move_to(z=plate_safe_z)
    for j in range(num_wells_y):
        for i in range(num_wells_x):
            x = plate_well_top_left[0] + i*well_delta_x
            y = plate_well_top_left[1] - j*well_delta_y
            #print x, y            
            xyz.move_to(x, y)
            dispense()

    #xyz.move_to(*plate_well_top_left)
    xyz.move_to(z=safe_z)    
    xyz.move_to_origin()

deploy_pos_for_claw = [0, 100, 0]
claw_plate_grab_z = 20
grab_plate_pos = [60, 0, claw_plate_grab_z]


#move to starting position above the height of the plate;
# grap the plate; place it in the incubator; withdraw claw
def incubate():
    xyz.move_to(*deploy_pos_for_claw)
    xyz.move_to(z=claw_plate_grab_z)
    claw.tilt(claw_tilt_down)
    xyz.move_to(*grab_plate_pos)
    claw.grip(claw_grip_closed)
    xyz.move_to(z=claw_plate_raise_z)
    xyz.move_to(*incubator_pos_in)
    claw.grip(claw_grip_open)
    xyz.move_to(*incubator_pos_out)


__rest_api_dict__ = {
    'move_to': XYZState.move_to,
    'position': XYZState.position
}

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

respPositionTopic = None

def cmd_move_callback(msg):
    rospy.loginfo(rospy.get_name() + ": " + str(msg))
    print msg.linear.x, msg.linear.y, msg.linear.z
    xyz.move_to(msg.linear.x, msg.linear.y, msg.linear.z, block_until_complete=True)

def query_position(msg):
    respPositionTopic.publish(Twist(*xyz.p))

def init_ros_listener():
    rospy.init_node('xyz_listener', anonymous=True)
    rospy.Subscriber('cmd_move_xyz', Twist, cmd_move_callback)
    rospy.Subscriber('query_position', String, query_position_callback)
    respPositionTopic = rospy.Publisher('resp_position', Twist)
    rospy.spin()

init_ros_listener()
