import krpc
import math
import numpy as np
import time
import csv
import pandas as pd
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from math import sin
from math import cos
from math import pi

conn = krpc.connect()
vessel = conn.space_center.active_vessel
flight_info = vessel.flight()

# Celestical body the flight occurs
cBody = vessel.orbit.body
gSur = cBody.surface_gravity

# Reference frame creation
create_relative = conn.space_center.ReferenceFrame.create_relative
create_hybrid = conn.space_center.ReferenceFrame.create_hybrid

surRF = vessel.surface_reference_frame
bodyRF = cBody.reference_frame
# Body surface spherical tracking reference frame
bodySurRF = create_hybrid(
	position = bodyRF,
	rotation = surRF)

# Landing site reference frame - VAB building rooftop
# Define the landing site as the top of the VAB
landing_latitude = -(0+(5.0/60)+(48.38/60/60))
landing_longitude = -(74+(37.0/60)+(12.2/60/60))
landing_altitude = 111
# (orientation: x=zenith, y=north, z=east)
landing_position = cBody.surface_position(
    landing_latitude, landing_longitude, cBody.reference_frame)
q_long = (
    0,
    sin(-landing_longitude * 0.5 * pi / 180),
    0,
    cos(-landing_longitude * 0.5 * pi / 180)
)
q_lat = (
    0,
    0,
    sin(landing_latitude * 0.5 * pi / 180),
    cos(landing_latitude * 0.5 * pi / 180)
)
landing_reference_frame = \
    create_relative(
        create_relative(
            create_relative(
                cBody.reference_frame,
                landing_position,
                q_long),
            (0, 0, 0),
            q_lat),
        (landing_altitude, 0, 0))

def dragCoefficient(v, aForce, la, lon, alti):
	pos = cBody.position_at_altitude(la, lon, alti, bodySurRF)
	rho = cBody.atmospheric_density_at_position(pos, bodySurRF)

	cdx = aForce[0]/v[0]**2/rho
	cdy = aForce[1]/v[1]**2/rho
	cdz = aForce[2]/v[2]**2/rho

	return(cdx, cdy, cdz)


## 2nd order Butterworth low pass filter
fdata = [0, 0, 0]
tF = []
cdFiltered = []

def step(x):
	global fdata
	fdata[0] = fdata[1]
	fdata[1] = fdata[2]
	fdata[2] = (1.099322143901515503e-2 * x) + (-0.72624607052130896179 * fdata[0]) + (1.68227318476524834168 * fdata[1])
	return (fdata[0] + fdata[2]) + 2 * fdata[1]

cfdata = [0, 0, 0]
def cstep(x):
	global cfdata
	cfdata[0] = cfdata[1]
	cfdata[1] = cfdata[2]
	cfdata[2] = (1.099322143901515503e-2 * x) + (-0.72624607052130896179 * cfdata[0]) + (1.68227318476524834168 * cfdata[1])
	return (cfdata[0] + cfdata[2]) + 2 * cfdata[1]

tolerance = 1e-2
targetHeight = 4400
cdTarget = []

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

	# bmid = (bhigh + blow) / 2

	# low = eqn(h0, v0, blow) - C2
	# mid = eqn(h0, v0, bmid) - C2
	# high = eqn(h0, v0, bhigh) - C2

	# print([low, mid, high])

	# if abs((bhigh - blow)/blow) < tolerance:
	# 	return bmid

	# elif low * mid < 0:
	# 	return findTargetCD(h0, v0, C2, blow, bmid)

	# else:
	# 	return findTargetCD(h0, v0, C2, bmid, blow)

def eqn(h0, v0, Cd):

	# num1 = gSur/(gSur + v0 ** 2 * Cd)
	# num2 = math.log(num1) / (2*Cd)
	# print(h0 - num2)
	# return h0 - num2

	v = v0
	h = h0
	rho_para = -0.000115* h0  + 1.225
	hPrev = h0
	dt = 5e-2

	while v > tolerance and hPrev <= h:

		hPrev = h
		h = h + v*dt
		v = v + (- gSur - v ** 2 * Cd)*dt
		Cd = Cd * (-0.000115* h  + 1.225) / rho_para

	print(h)
	return h


v = vessel.velocity(bodySurRF)
la = flight_info.latitude
lon = flight_info.longitude
h = flight_info.surface_altitude
aForce = flight_info.aerodynamic_force

vessel.auto_pilot.rcs = True
t0 = conn.space_center.ut

## target at the controller module setup
parts = vessel.parts.all
controller = None
controlModule = None
for p in parts:
	if p.name == "controller1000":
		controller = p

modules = controller.modules
for m in modules:
	if m.name == "ModuleRoboticController":
		controlModule = m

print(controlModule.fields)
controlPosition = controlModule.set_field_float("Play Position",0)
print(controlModule.fields)

## test target and PID parameters
targetCD = 4.5e-5
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


t = []
cd = []
height = []
velocity = []
estimatedHeight = []
festimatedHeight = []
plt.figure()

t1 = conn.space_center.ut
deltaT = 0
while True:

	mass = vessel.mass
	
	h0 = flight_info.mean_altitude
	v = vessel.velocity(bodySurRF)
	D = flight_info.drag
	
	v0 = v[0]
	Cdk = -1*D[0]/v0**2/mass

	t2 = conn.space_center.ut
	deltaT = t2 - t1
	t1 = conn.space_center.ut

	if vessel.situation == vessel.situation.flying and flight_info.g_force < 1:

		CdkF = step(Cdk)
		tF.append(t1 - t0)
		cdFiltered.append(CdkF)

		# find target Cd
		targetCD = findTargetCD(h0, v0, targetHeight, 1e-5, 6e-5)
		estimatedHeight.append(eqn(h0, v0, targetCD))
		print("raw target Cd is")
		print(targetCD)

		targetCD = cstep(targetCD)
		festimatedHeight.append(eqn(h0, v0, targetCD))

		cdTarget.append(targetCD)

		if not error == 1e6:
			errorPrev = error

		error = targetCD - CdkF
		errorIntegrate += deltaT * error
		if deltaT == 0:
			errorSlope = 0
		else:
			errorSlope = (error - errorPrev) / deltaT
		controlValue = pGain * error + iGain * errorIntegrate + dGain * errorSlope

		print("control is")
		print(controlValue)
		
		controlValue = max(min(10, controlValue), 0)
		controlPosition = controlModule.set_field_float("Play Position",controlValue)

	if v0 < -10:
		break

	print("cd is")
	print(Cdk)
	print()

	t.append(t1 - t0)
	cd.append(Cdk)
	height.append(h0)
	velocity.append(v0)

	time.sleep(0.1)

dict = {'h': height, 'v': velocity, 'cd': cd, 'time': t}
df = pd.DataFrame(dict)
df.to_csv('state.csv')
	
plt.plot(t,cd, 'b', label='True Cd')
plt.plot(tF,cdFiltered, 'g', label='Filtered Cd')
plt.plot(tF,cdTarget, 'r', label='Target Cd')
plt.title("Drag Coefficient History")
plt.xlabel("time (s)")
plt.ylabel("Drag Coefficient (1)")

plt.legend()

plt.figure()
plt.plot(t,height)
plt.title("Altitude History")
plt.xlabel("time (s)")
plt.ylabel("Actually Altitude (m)")

plt.figure()
plt.plot(tF,estimatedHeight, 'b', label='Estimated Apogee')
plt.plot(tF,festimatedHeight, 'g', label='Filtered Estimated Apogee')
plt.title("Estimated Apogee")
plt.xlabel("time (s)")
plt.ylabel("Altitude (m)")

plt.legend()

plt.show()