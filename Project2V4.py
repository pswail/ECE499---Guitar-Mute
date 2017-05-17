# Robot Arm Drawing A Picture
# 
# This program uses image edge detection, a prescanned grid of servo positions, and matrix forward kinematics
# to make a robot draw a picture
# 
# The prescanned grid is created using samplePosition.py and extracted from the file ServoPos.txt
# 
# Author: Pearse Swail
# Last Modified: May 11, 2017
#------------------------------------------------------------------------------------------------#
import numpy as np
#import cv2
from dxl.dxlchain import DxlChain

# Open the serial device
chain=DxlChain("COM8",rate=1000000)

# image file: Size (pixels) - ______
img_name = "smile.png"

#--------------------------get ServoPos.txt position array (works)-------------------------------#
def getPosArr():
	file = open("ServoPos.txt",'r')
	file.readline()
	line = file.readline()
	posArr = []
	height = 0
	while(line != ''):
		line = line.split()
		#print(line)
		for i in range(0,len(line)):
			num = line[i][0:len(line[i])-1]
			num = num.replace('.','')
			#print('num: ',num)
			if((i % 5) != 0):
				#print(num[len(num)-1])
				if(num[len(num)-1] == ']'):
					
					num = num[0:len(num)-1]
				posArr.append(int(num))
		#print(posArr)
		height += 1
		line = file.readline()

	width = height 	# must be square matrix...
	depth = 4
	posArrFin = np.zeros((height,width,depth))
	for i in range(height):
		for j in range(width):
			for k in range(depth):
				posArrFin[i][j][k] = posArr[(8*i)+(4*j)+k]

	#print(posArrFin)
	return posArrFin

#--------------------------forward kinematics (matrix kinematics) (works, but unused)----------------#
# makes translation matrix
def trMat(x, y, z):
	trmatrix = np.matrix([[x], [y], [z]])
	return trmatrix

# makes rotational matrix
def rotMat(theta, rot_axis):
	if(rot_axis == 'z'):
		rotmatrix = np.matrix([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0],[0, 0, 1]])
	if(rot_axis == 'y'):
		rotmatrix = np.matrix([[np.cos(theta), 0, np.sin(theta)],[0, 1, 0],[-np.sin(theta), 0, np.cos(theta)]])
	if(rot_axis == 'x'):
		rotmatrix = np.matrix([[1, 0, 0],[0, np.cos(theta), -np.sin(theta)],[0, np.sin(theta), np.cos(theta)]])
	return rotmatrix

# makes transformation matrix
def tMat(rotmatrix, trmatrix):
	addrot = np.matrix([[rotmatrix.item(0), rotmatrix.item(1), rotmatrix.item(2), 0],[rotmatrix.item(3), rotmatrix.item(4), rotmatrix.item(5), 0],[rotmatrix.item(6), rotmatrix.item(7), rotmatrix.item(8), 0],[0, 0, 0, 1]])
	#print('addrot: ', addrot)
	currenttr = np.matrix([[1, 0, 0, trmatrix.item(0)],[0, 1, 0, trmatrix.item(1)],[0, 0, 1, trmatrix.item(2)],[0, 0, 0, 1]])
	#print('currenttr: ', currenttr)
	tmatrix = np.dot(addrot, currenttr)
	return tmatrix

def findx(l0, l1, l2, l3, l4, theta0, theta1, theta2, theta3):
	x = (-l1 + l2*np.cos(theta1) + l3*np.cos(theta1+theta2) + l4*np.cos(theta1+theta2+theta3))*np.sin(theta0)
	return x
	
def findy(l0,l1,l2,l3,l4,theta0,theta1,theta2,theta3):
	y = (-l1 + l2*np.cos(theta1) + l3*np.cos(theta1+theta2) + l4*np.cos(theta1+theta2+theta3))*np.cos(theta0)
	return y
	
def findz(l0,l1,l2,l3,l4,theta0,theta1,theta2,theta3):
	z = l0 + l2*np.sin(theta1) + l3*np.sin(theta1+theta2) + l4*np.sin(theta1+theta2+theta3)
	return z

def deg2rads(deg):
	rads = (np.pi/180)*deg
	return rads
	
def rads2deg(rads):
	deg = (180/np.pi)*rads
	return deg
	
def FK(tdeg0, tdeg1, tdeg2, tdeg3):
	#constants
	rot_axis0 = 'z'
	rot_axis1 = 'x'
	rot_axis2 = 'x'
	rot_axis3 = 'x'
	l0 = 4
	l1 = 1
	l2 = 2
	l3 = 3
	l4 = 1
	
	#inputs
	#thetas (degs and rads)
	trad0 = deg2rads(tdeg0)
	trad1 = deg2rads(tdeg1)
	trad2 = deg2rads(tdeg2)
	trad3 = deg2rads(tdeg3)
	
	x = findx(l0,l1,l2,l3,l4,trad0,trad1,trad2,trad3)
	print('x: ',x)
	y = findy(l0,l1,l2,l3,l4,trad0,trad1,trad2,trad3)
	print('y: ',y)
	z = findz(l0,l1,l2,l3,l4,trad0,trad1,trad2,trad3)
	print('z: ',z)
	
	x0 = 0
	y0 = -l1
	z0 = l0
	
	trmatrix0 = trMat(x0,y0,z0)
	rotmatrix0 = rotMat(trad0, rot_axis0)
	tmatrix0 = tMat(rotmatrix0,trmatrix0)
	print('tmatrix0: ', tmatrix0)
	
	x1 = 0
	y1 = l2
	z1 = 0
	
	trmatrix1 = trMat(x1,y1,z1)
	print('trmatrix: ', trmatrix1)
	rotmatrix1 = rotMat(trad1, rot_axis1)
	print('rotmatrix: ', rotmatrix1)
	tmatrix1 = tMat(rotmatrix1, trmatrix1)
	print('tmatrix1: ', tmatrix1)
	x2 = 0
	y2 = l3
	z2 = 0
	
	trmatrix2 = trMat(x2,y2,z2)
	#print('trmatrix: ', trmatrix)
	rotmatrix2 = rotMat(trad2, rot_axis2)
	#print('rotmatrix: ', rotmatrix)
	tmatrix2 = tMat(rotmatrix2, trmatrix2)
	print('tmatrix2: ', tmatrix2)
	
	x3 = 0
	y3 = l4
	z3 = 0
	
	trmatrix3 = trMat(x3,y3,z3)
	#print('trmatrix: ', trmatrix)
	rotmatrix3 = rotMat(trad3, rot_axis3)
	#print('rotmatrix: ', rotmatrix)
	tmatrix3 = tMat(rotmatrix3, trmatrix3)
	print('tmatrix3: ', tmatrix3)
	
	T = np.dot(tmatrix2, tmatrix3)
	T = np.dot(tmatrix1, T)
	T = np.dot(tmatrix0, T)
	print('T: ', T)
	
	thetaE = trad1 + trad2 + trad3
	
	return T

#--------------------------functions for drawing-------------------------------------#
# converts image to Canny line detected image, then organizes into np array (works)
def getImg(img_name):
	img = cv2.imread(img_name,0)
	edges = cv2.Canny(img,100,200)
	width, height = edges.shape[:2]

	i = 0
	j = 0
	pixels = np.zeros((width,height))
	for i in range(width):			# arranges pixels into a numpy array
		for j in range(height):
			px = edges[i,j]
			pixels[i][j] = px
			j += 1
		i += 1

	return pixels

# scans image for non-zero value pixels (works)
def scanImg(pixels):
	i = 0
	j = 0
	size = pixels.shape
	for i in range(size[0]):
		for j in range(size[1]):
			if (pixels[i][j] != 0):
				return (i,j)
			j += 1
		i += 1
	return None
	
# find adjacent non-zero values (works)
def adjacentVals(pointerOld, pixels):
	i = pointerOld[0]
	j = pointerOld[1]
	size = pixels.shape
	
	# increment all bounds (8 possibilities) and check if any != 0
	# set priority to... above? Then left? Then right? Then down?
	# left pixels
	if(i > 0):
		ileft = i-1
	else:
		ileft = i
	# right pixels
	if(i < range(size[0])):
		iright = i+1
	else:
		iright = i
	# top pixels
	if(j > 0):
		jup = j-1
	else:
		jup = j
	# bottom pixels
	if(j < range(size[1])):
		jdown = j+1
	else:
		jdown = j
	
	# surrounding pixel values
	left = pixels[ileft][j]
	upleft = pixels[ileft][jup]
	up = pixels[i][jup]
	upright = pixels[iright][jup]
	right = pixels[iright][j]
	downright = pixels[iright][jdown]
	down = pixels[i][jdown]
	downleft = pixels[ileft][jdown]
	
	if(up != 0):
		return (i,jup)
	elif(upleft != 0):
		return (ileft, jup)
	elif(left != 0):
		return (ileft, j)
	elif(downleft != 0):
		return (ileft, jdown)
	elif(down != 0):
		return (i, jdown)
	elif(downright != 0):
		return (iright, jdown)
	elif(right != 0):
		return (iright, j)
	return None

# gets angle in degrees from goal position (works)
def goalPos2angle(goalPos):
	tdeg = goalPos * (300/1023) # 0-1023 pos to 0d-300d
	return tdeg

# moves pen to goal position (works)
def movePen(goalPos, raisePen):
	# servo IDs
	servoID_0 = 10
	servoID_1 = 20
	servoID_2 = 30
	servoID_3 = 40
	# extract new goalPos for each servo
	goalPos0 = goalPos[0]
	goalPos1 = goalPos[1]
	goalPos2 = goalPos[2]
	goalPos3 = goalPos[3]
	
	# if flag is raised, lift pen slightly
	if(raisePen == 1):
		goalPos3 += 25 # might need to change to -=

	# get new angles in degrees
	# tdeg0 = goalPos2angle(goalPos0)
	# tdeg1 = goalPos2angle(goalPos1)
	# tdeg2 = goalPos2angle(goalPos2)
	# tdeg3 = goalPos2angle(goalPos3)
	
	# I don't even need forward kinematics... All I need with this method is the goal position.
	#T = FK(tdeg0, tdeg1, tdeg2, tdeg3)		# matrix forward kinematics. Returns transformation matrix
	
	# may need to make function that moves it more smoothly, in steps. Not one motor at a time.
	chain.goto(servoID_3,goalPos3,speed=50) # Motor servoID is sent to position
	chain.goto(servoID_2,goalPos2,speed=50) # Motor servoID is sent to position
	chain.goto(servoID_1,goalPos1,speed=50) # Motor servoID is sent to position
	chain.goto(servoID_0,goalPos0,speed=50) # Motor servoID is sent to position

	return
	
# draws picture
def draw():
	posArr = getPosArr()				# get position array from ServoPos.txt
	print("got position array!")
	pixels = getImg(img_name)			# get image
	print("got image!")
	
	# initialize pen
	pointerOld = (0,0)
	raisePen = 1						# flag to raise pen
	movePen(pointerOld, raisePen)

	while True:					# loop until entire picture is drawn
		print("scanning image...")
		pointerNew = scanImg(pixels)		# detect non-zero values
		print("scanned image!")
		if(pointerNew == None):
			break

		goalPos = posArr[pointerNew[0]][pointerNew[1]]		# goal position from ServoPos.txt for pointerNew
		print("moving pen...")		
		movePen(goalPos, raisePen)							# moves pen to new goalPos
		print("pen moved!")
		
		raisePen = 0								# flag to lower pen
		pixels[pointerNew[0]][pointerNew[1]] = 0	# clear pixel (pixel drawn)
		pointerOld = pointerNew
		#print(pointerOld)

		while True:											# check for adjacent values
			pointerNew = adjacentVals(pointerOld, pixels)	# detect adjacent non-zero values
			if(pointerNew == None):
				raisePen = 1								# raise pen, if no adjacent values
				break

			goalPos = posArr[pointerNew[0]][pointerNew[1]]		# goal position from ServoPos.txt for pointerNew
			movePen(goalPos, raisePen)			# move to said values
			pixels[pointerNew[0]][pointerNew[1]] = 0		# clear pixel (pixel drawn)
			pointerOld = pointerNew

	raisePen = 1
	movePen((0,0),raisePen)
	chain.disable() # disable motors
	return

motors=chain.get_motor_list() # Discover all motors on the chain and return their IDs
print motors
raw_input("Got Motor IDs. Press any key to continue...")
movePen([597, 586, 336, 591],0)
raw_input("wait")
movePen([602, 613, 294, 591],0)
raw_input("wait")
movePen([630, 592, 327, 592],0)
raw_input("wait")
movePen([624, 565, 366, 577],0)

