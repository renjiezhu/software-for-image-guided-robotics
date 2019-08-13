#Motor arm mixing, accounts for coupling in motion of each degree of freedom
#Wants radians and mm?
import numpy as np

def MotorArmMixing(armTheta): #armTheta is a 4x1 angle input for desired arm configuration
	#Pulley diameters (mm)
	Db = 16.5
	Ds = 8.72
	Dm = 13.88
	Dl = 18
	Dj1 = 21
	Dj2 = 21.5
	Dj3 = 13.5

	motorTheta_armTheta = np.eye(4) * np.array([Db/Dj1, Db/Dj2, Db/Dj3, Db])
	motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [Dl/Dj2, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]), motorTheta_armTheta)
	motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [Dm/Dj3, -Dm/Dj3, 1, 0], [0, 0, 0, 1]]), motorTheta_armTheta)
	motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [-Ds, Ds, Dl, 1]]), motorTheta_armTheta)

	armTheta_motorTheta = np.linalg.inv(motorTheta_armTheta) #is a 4x4 matrix for 4dof arm

	motor_values = np.dot(armTheta_motorTheta, armTheta) #takes in arm thetas and gives appropriate motor thetas

	return motor_values