from xyz import XYZ

xyz = XYZ()
#xyz.startup('/dev/ttyACM2')

claw = Claw()
#claw.startup('/dev/ttyACM2')

# liquid handler: only use the first axis of the XYZ class to control the pump
liquid = XYZ()
#liquid.startup('/dev/ttyACM1', 9600, 1)

def shutdown():
	claw.shutdown()
	liquid.shutdown()
	xyz.shutdown()

################################################################################
#absolute positions
clawTiltUp = 80
clawTiltDown = 16
clawGripOpen = 120
clawGripClosed = 90
#claw.tilt(clawTiltUp)	 #put claw in default position
#claw.grip(clawGripOpen)

safeZ = 0
plateSafeZ = -50
plateDispZ = -60
resevoirDrawZ = -72

clearResevoirPos = [0, 150, safeZ]
redResevoirPos = [0, 0, safeZ]
blueResevoirPos = [0, 320, safeZ]
plateWellTopLeft = [-230, 250, safeZ]

clawPlateRaiseZ = 40
incubatorPosIn = [-320, 250, clawPlateRaiseZ]
incubatorPosOut = [-120, 250, clawPlateRaiseZ]

wellDeltaX = 8.3
wellDeltaY = 8.8
numWellsX = 1
numWellsY = 2

liquidPerWell = 5.0
totalLiquid = numWellsX * numWellsY * liquidPerWell

def pippette(resevoir_pos):

	#helper for pipetting: move down, dispense, move up
	def dispense():
		xyz.moveTo(z=plateDispZ)
		liquid.move_rel(liquidPerWell+1.5)
		time.sleep(2)					   #wait for liquid to fall out
		xyz.moveTo(z=plateSafeZ)

	#draw liquid
	xyz.moveTo(*resevoir_pos)
	xyz.moveTo(z=resevoirDrawZ)
	liquid.move_rel(-totalLiquid)  #TODO store/update how much liquid held
	time.sleep(2)
	xyz.moveTo(z=safeZ)

	#move to plate and dispense into wells
	xyz.moveTo(*plateWellTopLeft)
	xyz.moveTo(z=plateSafeZ)
	for j in range(numWellsY):
		for i in range(numWellsX):
			x = plateWellTopLeft[0] + i*wellDeltaX
			y = plateWellTopLeft[1] - j*wellDeltaY
			#print x, y			
			xyz.moveTo(x, y)
			dispense()

	#xyz.moveTo(*plateWellTopLeft)
	xyz.moveTo(z=safeZ)	
	xyz.moveTo_origin()

deploy_pos_for_claw = [0, 100, 0]
claw_plate_grab_z = 20
grab_plate_pos = [60, 0, claw_plate_grab_z]


# move to starting position above the height of the plate;
# grap the plate; place it in the incubator; withdraw claw
def incubate():
	xyz.moveTo(*deploy_pos_for_claw)
	xyz.moveTo(z=claw_plate_grab_z)
	claw.tilt(clawTiltDown)
	xyz.moveTo(*grab_plate_pos)
	claw.grip(clawGripClosed)
	xyz.moveTo(z=clawPlateRaiseZ)
	xyz.moveTo(*incubatorPosIn)
	claw.grip(clawGripOpen)
	xyz.moveTo(*incubatorPosOut)
