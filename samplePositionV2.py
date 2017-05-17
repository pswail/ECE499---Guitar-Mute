# Sample Position V2 - 5x5x4 data collection with interpolation to 100x100x4 matrix

import numpy as np
from dxl.dxlchain import DxlChain

# Open the serial device
chain=DxlChain("COM8",rate=1000000)

def getVals(ServoID_0,ServoID_1,ServoID_2,ServoID_3,i,j,height,width):
	raw_input('Current sample: (%d , %d) out of (%d , %d). Press any key to continue...' % (i, j, height-1, width-1))
	# gets position values
	pos0_new = chain.get_position(ServoID_0)	# returns dictionary
	pos0_new = pos0_new.values()					# extracts val from dict
	pos1_new = chain.get_position(ServoID_1)
	pos1_new = pos1_new.values()
	pos2_new = chain.get_position(ServoID_2)
	pos2_new = pos2_new.values()
	pos3_new = chain.get_position(ServoID_3)
	pos3_new = pos3_new.values()
	
	pos_new = np.array([pos0_new[0], pos1_new[0], pos2_new[0], pos3_new[0]])
	return pos_new
	
def getStep(pos_new, pos_old):
	#print('pos new: ',pos_new)
	#print('pos old: ',pos_old)

	step0 = (pos_new[0] - pos_old[0])/20
	step1 = (pos_new[1] - pos_old[1])/20
	step2 = (pos_new[2] - pos_old[2])/20
	step3 = (pos_new[3] - pos_old[3])/20
	step = np.array([step0, step1, step2, step3])	# array of steps per servo
	return step

# interpolates 20 points of data in x direction
def array(pos,step,Arr,height,width):
	for i in range(0, 20):
		pos[0] = np.floor(pos[0] + i*step[0])
		pos[1] = np.floor(pos[1] + i*step[1])
		pos[2] = np.floor(pos[2] + i*step[2])
		pos[3] = np.floor(pos[3] + i*step[3])
		Arr[height][width+i] = pos			#update changing width
		#print('height, width+i:', height, width+i)
		#print('pos: ', pos)
	return Arr

# interpolates 20 points of data in y direction
def yarray(pos,step,Arr,height,width):
	for i in range(0, 20):
		#print('og pos: ',pos)
		pos[0] = np.floor(pos[0] + i*step[0])
		pos[1] = np.floor(pos[1] + i*step[1])
		pos[2] = np.floor(pos[2] + i*step[2])
		pos[3] = np.floor(pos[3] + i*step[3])
		Arr[height+i][width] = pos			#update changing height
		#print('height+i, width: ', height+i, width)
		#print('pos: ',pos)
		
	return Arr

def samplePos():
	# Constants
	# Servo IDs
	ServoID_0 = 10
	ServoID_1 = 20
	ServoID_2 = 30
	ServoID_3 = 40
	
	# width and height / data array
	width = 100
	height = 100
	posArr = np.zeros((width,height,4), dtype = np.float64) # change and re-sample to make larger array (more pixels)
	
	motors=chain.get_motor_list() # Discover all motors on the chain and return their IDs
	print motors
	raw_input("Got Motor IDs. Press any key to continue...")
	
	pos0_old = 0
	pos1_old = 0
	pos2_old = 0
	pos3_old = 0
	pos_old = np.array([pos0_old, pos1_old, pos2_old, pos3_old])
	
	heightCount = 0
	widthCount = 0
	
	for i in range(0, height/20):
		for j in range(0, width/20):
			pos_new = getVals(ServoID_0,ServoID_1,ServoID_2,ServoID_3,i,j,height,width)	# array of new pos values 
			if(pos_old[0] != 0):	# skip first iteration
				step = getStep(pos_new,pos_old)					# finds average step between sampled points
				#print('step: ', step)
				posArr = array(pos_old, step, posArr, heightCount, widthCount)		# interpolates data with steps and adds to posArr (20 values)
				print("posArr: ", posArr)
				
			pos_old = pos_new		# update vals
			widthCount += 20
		
		if(i != 0):		# skip first iteration
			for j in range(width):
				yPos_new = posArr[i][j]
				yPos_old = posArr[i-1][j]
				print('yPos_new: ', yPos_new)
				print('yPos_old: ', yPos_old)
				# yPos_new and old are wrong...
				
				stepy = getStep(yPos_new,yPos_old)
				posArr = yarray(yPos_old,stepy,posArr, heightCount, j)
				
		widthCount = 0
		heightCount += 20
			
	# write information to a file called ServoPos.txt
	head = 'This file contains an array of servo position info. Each element contains: [pos0, pos1, pos2, pos3]'
	np.savetxt("ServoPos.txt",posArr,fmt = '%s',header=head)
	
	chain.disable()
	return

samplePos()