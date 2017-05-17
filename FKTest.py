# matrix kinematics
import numpy as np

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

tdeg0 = 0
tdeg1 = 0
tdeg2 = 0
tdeg3 = 0
FK(tdeg0, tdeg1, tdeg2, tdeg3)
