import adafruit_mpl3115a2
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import adafruit_bno055
import Adafruit_BBIO.PWM as PWM
import time
import board
import os

from enum import Enum
class Vehicle_Status(Enum):
	ON_PAD = 0
	BOOST = 1
	GLIDE = 2
	APOGEE = 3
	DONE = 4

##################################
##################################
## launch day software checklist
pad_pressure = 102250
launch_date = "05-19-2022"
# servo pin check
servoPin = "P8_13"
# switch to true for SD storage
SD = False
##################################
##################################

# 6.9s -- burn phase
# 19.4s -- coast phase
time_bo = 8
time_apo = 22
# auto abort time at no senser reading
time_end = 120

# system parameters
tolerance = 1e-2
targetHeight = 1371.6 # 4500 ft
fdata = [0, 0, 0]
adata = [0, 0, 0]
hdata = [0, 0, 0]
# control from 0 to 10
error = 1e6
errorPrev = 0
errorIntegrate = 0
# pGain = 9e4
# iGain = 5.4e4
# dGain = 2.7e4
pGain = 1e5
iGain = 6e4
dGain = 6e4
# Cd parameters
cdUpper = 0
cdBottom = 1

# /mnt/SD/ is the mount point of the SD card
savePath = '/mnt/SD/'
fileName = "DataLog_"+launch_date+".txt"

if not SD:
	savePath = '/home/debian'

###################################
ledonePath = '/sys/class/leds/beaglebone:green:usr1'
ledBrightnessFile = 'brightness'

brightness = 1
ledGapTime = 0.5

###################################
# threshold limits
boost_a_threshold = 5
boost_height_threshold = 20
glide_a_threshold = -0.9

# deployment angle limits
MAX_ANGLE = 60
MIN_ANGLE = 0

# altimeter initialization count limit
count_limit = 50

# attitude fail safe limit
fail_angle_limit = 30
allow_angle_limit = 15

# contingency deployment array - test launch 1
# cda = [25, 0, 50, 0, 75, 0, 40, 70, 100, 0]
###################################

###################################
## bus address
# mpl3115a2 default i2c bus
MPL_BUS = 0x60
# imu default i2c bus V3+ uses BNO055
#LSM_BUS = 0x6a
LSM_BUS = 0x28
###################################

# servo setup parameter 
duty_min = 3
duty_max = 14.5

## constants
g0 = 9.8066
# deployment in percentage
INIT_DEPLOYMENT = 0 
APOGEE_FSM_CHANGE = 5
FSM_DONE_SURFACE_ALTITUDE = 200
ALTI_DEPL_THRESHOLD = 0.5
RATE_ALTI_DEPL = 1/50


## systems initialization
def init():

	global altimeter
	global imu
	global duty_span
	global file
	global ledFileName
	global i2c

	# create sensor object, communicating via default I2C bus
	i2c = board.I2C()

	## SD card logger setup
	# construct completed file location
	completedName = os.path.join(savePath, fileName)
	# create a new file
	file = open(completedName, "a")
	# write header on the logger
	SDwrite("Launch Date: " + launch_date)

	## altimeter setup
	# create and initialize sensor
	altimeter = altimeter_set(i2c)

	## IMU setup
	# create and initialize IMU
	# imu = LSM6DSOX(i2c, address=LSM_BUS)
	imu = imu_set(i2c)

	## servo setup
	duty_span = duty_max - duty_min
	# servo might have a different parity
	PWM.start(servoPin, (100 - duty_min), 60.0, 1)

	## blick the beaglebone LED 1
	ledFileName = os.path.join(ledonePath, ledBrightnessFile)

	status = Vehicle_Status.ON_PAD
	return status


## Data formatting and logging 
def SDwrite(string):

	global file

	log_time = time.time() - start_time
	file.write("{0:0.3f}".format(log_time) + ", ")
	file.write(string + "\n")


## Provide on pad average altitude
## run at start
def alti_initialize(init_start_time):

	time.sleep(1)

	count_altitude_read = 0
	average_altitude = 0

	while (count_altitude_read < count_limit):
		# on pad average altitude
		altitude = altimeter.altitude
		if count_altitude_read == 0 or count_altitude_read < count_limit:
		# (count_altitude_read < 100 and abs(altitude - average_altitude) < 0.8):

			count_altitude_read += 1
			average_altitude = (average_altitude * (count_altitude_read - 1)\
			 + altitude)/count_altitude_read

	SDwrite("pad altitude initialization complete - {0:0.3f}".format(average_altitude))
	alti_init_time = time.time() - init_start_time
	return average_altitude


def altimeter_set(i2c):

	global alti_Setup_fail

	try:
		altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=MPL_BUS)
		altimeter.sealevel_pressure = pad_pressure

		alti_Setup_fail = False
		return altimeter

	except Exception as err:
		SDwrite(str(err))

		alti_Setup_fail = True
		return None


def imu_set(i2c):

	global imu_Setup_fail

	try:
		imu = adafruit_bno055.BNO055_I2C(i2c, address=LSM_BUS)

		imu_Setup_fail = False
		return imu

	except Exception as err:
		SDwrite(str(err))

		imu_Setup_fail = True
		return None


## euler angle attitude safe detection
## angle to the vertial < fail_angle_limit
def attitude_safe(euler):

	global atti_fail
	
	pitch = euler[1]
	yaw = euler[2]
	angle_2 = pitch ** 2 + yaw ** 2
	atti_fail = False

	if angle_2 > fail_angle_limit ** 2:
		SDwrite("AttiFail")

		atti_fail = True


def deploy_percentage_to_angle(percentage):

	return (MAX_ANGLE - MIN_ANGLE) / 100.0 * percentage + MIN_ANGLE


def servo_actuate(deployment):

	angle_f = float(deployment)
	duty = 100 - ((angle_f / 180) * duty_span + duty_min)
	PWM.set_duty_cycle(servoPin, duty)


def output(altitude, deployment, acceleration, gyro, euler_angle):

	if (not alti_Setup_fail and not alti_fail):

		SDwrite("{0:0.3f}".format(altitude)) # (meters)

	SDwrite(" {0:0.2f}".format(deployment)) # (degrees)

	if (not imu_Setup_fail and not imu_fail):

		SDwrite("%.2f %.2f %.2f" % (acceleration)) # X Y Z (m/s^2)
		SDwrite("%.2f %.2f %.2f" % (gyro)) # X Y Z (radians/s)
		SDwrite("%.2f %.2f %.2f" % (euler_angle)) # latter two is pitch and yaw


# controller logics and helper implementation
## 2nd order Butterworth low pass filter
def step(x):
	global fdata
	fdata[0] = fdata[1]
	fdata[1] = fdata[2]
	fdata[2] = (1.099322143901515503e-2 * x) + (-0.72624607052130896179 * fdata[0]) + (1.68227318476524834168 * fdata[1])
	return (fdata[0] + fdata[2]) + 2 * fdata[1]

## 2nd order Butterworth low pass filter
def astep(x):
	global adata
	adata[0] = adata[1]
	adata[1] = adata[2]
	adata[2] = (1.099322143901515503e-2 * x) + (-0.72624607052130896179 * adata[0]) + (1.68227318476524834168 * adata[1])
	return (adata[0] + adata[2]) + 2 * adata[1]

## 2nd order Butterworth low pass filter
def hstep(x):
	global hdata
	hdata[0] = hdata[1]
	hdata[1] = hdata[2]
	hdata[2] = (1.099322143901515503e-2 * x) + (-0.72624607052130896179 * hdata[0]) + (1.68227318476524834168 * hdata[1])
	return (hdata[0] + hdata[2]) + 2 * hdata[1]

## search for the target coefficient value
def findTargetCD(h0, v0, C2, blow, bhigh):

	division = 10

	p1Start = blow
	percentale1 = (bhigh - blow) / division

	minValue1 = C2
	minIndex1 = -1
	for i in range(0,division):

		value = abs(eqn(h0, v0, p1Start + i*percentale1) - C2)
		if value < minValue1:
			minValue1 = value
			minIndex1 = i

	p2Start = p1Start + (minIndex1-0.5) * percentale1
	p2End = p1Start + (minIndex1+0.5) * percentale1
	percentale2 = percentale1 / division

	minValue2 = minValue1
	minIndex2 = -1
	for i in range(0,division):

		value = abs(eqn(h0, v0, p2Start + i*percentale2) - C2) 
		if value < minValue2:
			minValue2 = value
			minIndex2 = i

	return p2Start + minIndex2 * percentale2

## perpregate the equation of motion
# Cd is the total coefficient of Drag such that D = Cd * v^2
def eqn(h0, v0, Cd):

	# num1 = gSur/(gSur + v0 ** 2 * Cd)
	# num2 = math.log(num1) / (2*Cd)
	# print(h0 - num2)
	# return h0 - num2

	v = v0
	h = h0
	hPrev = h0
	dt = 5e-2

	while v > tolerance and hPrev <= h:

		hPrev = h
		h = h + v*dt
		v = v + (- g0 - v ** 2 * Cd)*dt
		Cd = Cd * rho(h)/rho(h0)

	return h

## standard atmosphere fit
def rho(h):
	return 0.000000003679762*h^2-0.000116132142857*h+1.224730952380953


def pid(CdF,h0,v0):
	targetCd = findTargetCD(h0, v0, targetHeight, cdUpper, cdBottom,dt)

	SDwrite("CDF: {0:f}".format(CdF))
	SDwrite("TCD: {0:f}".format(targetCd))
	SDwrite("TAL: {0:f}".format(eqn(h0,v0,targetCd)))

	if not error == 1e6:
			errorPrev = error

	error = targetCd - CdF
	errorIntegrate += dt * error
	if dt == 0:
		errorSlope = 0
	else:
		errorSlope = (error - errorPrev) / dt
	return pGain * error + iGain * errorIntegrate + dGain * errorSlope


# customized based on the mission
def actuation_plan(status, h, dh, z_a):

	global fail_time
	global deploy_time
	global altimeter
	global imu

	atti_fail = False

	deployment = deploy_percentage_to_angle(0)
	
	if (not imu_fail and not alti_fail and not atti_fail):

		## for failure termination
		fail_time = time.time()

		## below is custmized based on missions
		## | | | | | | | | | | | | | |
		## v v v v v v v v v v v v v v
		if status is Vehicle_Status.GLIDE:

			hF = hstep(h)
			dhF = astep(dh)
			z_aF = step(z_a)

			v = dhF / dt
			CdF = (z_aF - g0) / (v ** 2)
			deployment = deploy_percentage_to_angle(pid(CdF, h, v))

			# # full deployment during coasting
			# deployment = deploy_percentage_to_angle(0)
			# if time.time() - deploy_time < 1:
			# 	deployment = deploy_percentage_to_angle(100)


		elif status is Vehicle_Status.APOGEE:

			deployment = deploy_percentage_to_angle(100)

		else: 

			deploy_time = time.time()

		print(deploy_time)

	else:
		
		if alti_fail:
			altimeter = altimeter_set(i2c)
			SDwrite("altimRes")

		if imu_fail:
			imu = imu_set(i2c)
			SDwrite("imuRes")

	return deployment


def main():

	global start_time
	global fail_time
	global imu_fail
	global alti_fail
	global altimeter
	global imu
	global file
	global ledFileName
	global brightness

	start_time = time.time()
	fail_time = start_time
	relog_time = start_time
	led_time = start_time
	measurementTime = start_time

	try:
		status = init()
	except Exception as err:
		SDwrite(str(err))

	## altimeter initial reading and altitude setup
	on_PAD_altitude = 0
	on_PAD_fail = False
	if not alti_Setup_fail:

		try:
			on_PAD_altitude = alti_initialize(time.time())

		except Exception as err:
			SDwrite(str(err))

			on_PAD_fail = True

	## altimeter setup
	apogee_altitude = 0
	previous_altitude = on_PAD_altitude
	altitude = on_PAD_altitude
	alti_fail = False

	iter_post_apo = 0

	## imu setup
	acceleration = (0,0,0)
	gyro = (0,0,0)
	z_a = 0
	euler_angle = (0,0,0)
	imu_fail = False

	deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

	while True:

		SDwrite(status.name)

		if not imu_Setup_fail:

			try:
				acceleration = imu.acceleration
				z_a = acceleration[2]
				gyro = imu.gyro
				euler_angle = imu.euler

				imu_fail = False

			except Exception as err:
				SDwrite(str(err))

				imu_fail = True

		deltaAltitude = altitude - previous_altitude
		previous_altitude = altitude

		if not alti_Setup_fail:

			try:
				altitude = altimeter.altitude
				if on_PAD_fail:
					on_PAD_altitude = altitude
					on_PAD_fail = False

				alti_fail = False

			except Exception as err:
				SDwrite(str(err))

				alti_fail = True

		dt = time.time() - measurementTime
		measurementTime = time.time()

		if (not alti_Setup_fail and not imu_Setup_fail):

			if apogee_altitude < altitude:
				apogee_altitude = altitude

			if status is Vehicle_Status.ON_PAD:

				if (z_a >= boost_a_threshold * g0 and altitude >= boost_height_threshold + on_PAD_altitude): # or time.time() - start_time >= 30:
					liftoff_time = time.time()
					SDwrite("\n\nLift Off Mark at -- {0:0.3f}\n\n".format(liftoff_time - start_time))
					status = Vehicle_Status.BOOST
				
				# reset initial deployment
				deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

				# relogging detection every minute
				if time.time() - relog_time > 60 and status is Vehicle_Status.ON_PAD:

					file.close()
					# completedName = os.path.join(savePath, fileName)
					# os.remove(completedName)
					file = open(completedName, "w")
					relog_time = time.time()
					print("OverWR Success")

			elif status is Vehicle_Status.BOOST:

				if (z_a <= glide_a_threshold * g0 or time.time() - liftoff_time >= time_bo):
					boost_time = time.time()
					status = Vehicle_Status.GLIDE

			elif status is Vehicle_Status.GLIDE:

				if (altitude < apogee_altitude - APOGEE_FSM_CHANGE or time.time() - liftoff_time >= time_bo + time_apo):
					status = Vehicle_Status.APOGEE
					SDwrite("APO: {0:0.2f}".format(apogee_altitude))

			elif status is Vehicle_Status.APOGEE:

				if altitude <= FSM_DONE_SURFACE_ALTITUDE + on_PAD_altitude:
					status = Vehicle_Status.DONE
					break

			attitude_safe(euler_angle)
			deployment = actuation_plan(status, altitude, deltaAltitude, z_a, dt)
		
		else:

			if time.time() - fail_time >= time_end:
				status = Vehicle_Status.DONE
				break

			if alti_fail or alti_Setup_fail:
				altimeter = altimeter_set(i2c)
				SDwrite("altimRes")

			if imu_fail or imu_Setup_fail:
				imu = imu_set(i2c)
				SDwrite("imuRes")

			deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

		servo_actuate(deployment)

		output(altitude, deployment, acceleration, gyro, euler_angle)

		if time.time() - led_time > ledGapTime:
			ledFile = open(ledFileName, "w")
			ledFile.write(str(brightness))

			led_time = time.time()
			ledFile.close()
			brightness = (brightness + 1) % 2

		time.sleep(0.01)

	# end ...
	file.close()



if __name__ == "__main__":
	main()