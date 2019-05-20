#!/bin/python

##
# Looping between two points - Joint Controler Evaluation
#
# @author Rafael
##

# Meka API Imports
import time
import m3.toolbox as m3t
import m3.rt_proxy
import m3.humanoid as m3h
import scipy.linalg as LA
import numpy as nu
import yaml
import rosbag
import subprocess

def testSetup():
    '''Meka Start Routine'''
    # Connect to the m3rt_server
    global proxy
    proxy = m3.rt_proxy.M3RtProxy()
    proxy.start()

    bot_name=m3t.get_robot_name()
    if bot_name == "":
        print 'Error: no robot components found:', bot_names

    global bot
    bot=m3.humanoid.M3Humanoid(bot_name)

    # Now that we are connected to the m3rt_server
    # We need to say we want to get status/cmd

    # Get status
    proxy.subscribe_status(bot)

    # Send command
    proxy.publish_command(bot)

    proxy.make_operational_all()

    bot.set_motor_power_on()

    humanoid_shm_names=proxy.get_available_components('m3humanoid_shm')
    if len(humanoid_shm_names) > 0:
        proxy.make_safe_operational(humanoid_shm_names[0])

    # Some Poses for testing
    global angles_calib
    global angles_kojiref
    global angles_zero
    angles_calib = [33.0093847, 2.7316443 , -3.5048929, 98.8776785 , -0.8857203,    48.0334749, 3.2318867]
    angles_kojiref = [0, 17.0531787, 0, 90.0, 108.2084705, 0, -5.3142586]
    angles_zero = [0,0,0,0,0,0,0]

    # Start Position -----------------
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angles_kojiref)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [0.3]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[0.7]*bot.get_num_dof('right_arm'))

    # Update command and status by calling proxy
    proxy.step()
    time.sleep(3)


def terminate_process_and_children(p):
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.get_children():
        sub_process.send_signal(subprocess.signal.SIGINT)

    p.wait()  # we wait for children to terminate
#    p.send_signal(subprocess.signal.SIGINT)

def testTwoPoint():
    '''Inner Control Evaluation'''

    dtheta = 1
    dt = 0.1

    while True:
        timeStart = time.time()

        # Joint angles Position 1
        angle_test = [0, 0, 0, 90.0, 108.2084705, 0, -5.3142586]

        # Set the mode Theta with gravity compensation
        bot.set_mode_theta_gc('right_arm')
        # Set Desired Joint angles
        bot.set_theta_deg('right_arm',angle_test)
        # Set Stiffiness
        bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        bot.set_slew_rate_proportion('right_arm',[0.7,0.7,0.7,0.7,0.7,0.7,0.7])

        proxy.step()
        time.sleep(dt)

        timeStep1 = time.time() - timeStart
        timeCurrent = time.time()

        # Joint angles Position 2
        angle_test = [0, 0, dtheta, 90.0, 108.2084705, 0, -5.3142586]

        # Set the mode Theta with gravity compensation
        bot.set_mode_theta_gc('right_arm')
        # Set Desired Joint angles
        bot.set_theta_deg('right_arm',angle_test)
        # Set Stiffiness
        bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        bot.set_slew_rate_proportion('right_arm',[0.7,0.7,0.7,0.7,0.7,0.7,0.7])

        proxy.step()
        time.sleep(dt)

        timeStep2 = time.time() - timeCurrent

        print( str(timeStep1-dt) + " " + str(timeStep2-dt) + " " + str(dt) )

def testTwoPointTrajectory():
    
    dtheta = 1
    dt = 0.1

    while True:
        timeStart = time.time()

        # Joint angles Position 1
        angle_test = [0, 0, 0, 90.0, 108.2084705, 0, -5.3142586]

        # Set the mode Theta with gravity compensation
        bot.set_mode_theta_gc('right_arm')
        # Set Desired Joint angles
        bot.set_theta_deg('right_arm',angle_test)
        # Set Stiffiness
        bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        bot.set_slew_rate_proportion('right_arm',[0.7,0.7,0.7,0.7,0.7,0.7,0.7])

        proxy.step()
        time.sleep(dt)

        timeStep1 = time.time() - timeStart
        timeCurrent = time.time()

        # Joint angles Position 2
        angle_test = [0, 0, dtheta, 90.0, 108.2084705, 0, -5.3142586]

        # Set the mode Theta with gravity compensation
        bot.set_mode_theta_gc('right_arm')
        # Set Desired Joint angles
        bot.set_theta_deg('right_arm',angle_test)
        # Set Stiffiness
        bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        bot.set_slew_rate_proportion('right_arm',[0.7,0.7,0.7,0.7,0.7,0.7,0.7])

        proxy.step()
        time.sleep(dt)

        timeStep2 = time.time() - timeCurrent

        print(str(timeStep1-dt) + " " + str(timeStep2-dt) + " " + str(dt) )

def testTimeDelayStatus():
    # Get Current Position
    proxy.step()
    angle = bot.get_theta_deg('right_arm')

    timeStart = time.time()
    timeStep = 0

    while True:
        proxy.step()
	newAngle = bot.get_theta_deg('right_arm')
        if ( (newAngle != angle).all() ):
             timeStep = time.time() - timeStart
             timeStart = time.time()
             print( str( timeStep ) )

def testTimeDelayProxyStep():
    # Get Current Position
    proxy.step()
    angle = bot.get_theta_deg('right_arm')

    timeStart = time.time()
    timeStep = 0

    while True:
        proxy.step()
        newAngle = bot.get_theta_deg('right_arm')
        timeStep = time.time() - timeStart
        timeStart = time.time()
        print( str( timeStep ) )

def testHoldPosition():
    # Get Current Position
    proxy.step()
    angle = bot.get_theta_deg('right_arm')

    timeStart = time.time()
    timeStep = 0

    while True:
        proxy.step()
        newAngle = bot.get_theta_deg('right_arm')
        bot.set_theta_deg('right_arm',newAngle)
        timeStep = time.time() - timeStart
        timeStart = time.time()
        print( str( timeStep ) )

def testHoldPositionReference():
    # Get Current Position
    proxy.step()
    angle = bot.get_theta_deg('right_arm')

    timeStart = time.time()
    timeStep = 0

    while True:
        proxy.step()
        newAngle = bot.get_theta_deg('right_arm')
        bot.set_theta_deg('right_arm',angle)
        timeStep = time.time() - timeStart
        timeStart = time.time()
        print( str( timeStep ) )

def testNoCommandPrecisionEncoder():
    # Get Current Position
    proxy.step()
    angleRef = bot.get_theta_deg('right_arm')
    angle = angleRef

    timeStart = time.time()
    timeStep = 0

    while True:
        proxy.step()
        # Get Joint Angles
        newAngle = bot.get_theta_deg('right_arm')
        dAngle0 = LA.norm(newAngle-angle)
        angle = newAngle
        # Get Times
        timeStep = time.time() - timeStart
        timeStart = time.time()
        # Print
        print( str( dAngle0 ) )

def testTwoPointTrajectorySmartDelay():
    
    dtheta = 20
    dt = 0.1
    errorTolerance = 12

    while True:
        timeStart = time.time()

        # Joint angles Position 1
        angle_test = [0, 0, 0, 90.0, 108.2084705, 0, -5.3142586]

        # Set the mode Theta with gravity compensation
        bot.set_mode_theta_gc('right_arm')
        # Set Desired Joint angles
        bot.set_theta_deg('right_arm',angle_test)
        # Set Stiffiness
        bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
        # Send Command        
        proxy.step()
        
        # Wait until it reach position
        errorStep1 = 100
        angleStep1 = bot.get_theta_deg('right_arm')
        while (errorStep1 > errorTolerance):
            proxy.step()
            newAngle = bot.get_theta_deg('right_arm')
            errorStep1 = LA.norm(angle_test-newAngle)
            angleStep1 = newAngle

        timeStep1 = time.time() - timeStart
        timeCurrent = time.time()

        # Joint angles Position 2
        angle_test = [0, 0, dtheta, 90.0, 108.2084705, 0, -5.3142586]

        # Set the mode Theta with gravity compensation
        bot.set_mode_theta_gc('right_arm')
        # Set Desired Joint angles
        bot.set_theta_deg('right_arm',angle_test)
        # Set Stiffiness
        bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
        # Send Command        
        proxy.step()

        # Wait until it reach position
        errorStep2 = 100
        angleStep2 = bot.get_theta_deg('right_arm')
        while (errorStep2 > errorTolerance):
            proxy.step()
            newAngle = bot.get_theta_deg('right_arm')
            errorStep2 = LA.norm(angle_test-newAngle)
            angleStep2 = newAngle

        timeStep2 = time.time() - timeCurrent

        print(str(dtheta/timeStep1) + " " + str(timeStep1) + " " + str(errorStep1) + " " + str(timeStep2) + " " + str(errorStep2) )

def testJointLowLevelControlResponse():
    
    dtheta = 45
    dt = 0.1
    errorTolerance = 1
    speedLimit = nu.rad2deg([4.6,4.6,4.5,4.5,2.0,3.4,3.4]) # from documentation

    # Joint angles Position 1
    angle_test = [0, 0, 0, 90.0, 108.2084705, 0, -5.3142586]
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angle_test)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
    # Send Command        
    proxy.step()

    time.sleep(5)

    print("start")

    for i in range(7):
        for j in range(3):
            timeStart = time.time()

            # Joint angles Position 1
            angle_test = [0, 0, 0, 90.0, 108.2084705, 0, -5.3142586]

            # Get Current Angle
            currentAngle = bot.get_theta_deg('right_arm')
            # Set the mode Theta with gravity compensation
            bot.set_mode_theta_gc('right_arm')
            # Set Desired Joint angles
            bot.set_theta_deg('right_arm',angle_test)
            # Set Stiffiness
            bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
            # Set Joint Speed [0 to 1]
            bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
            # Send Command        
            proxy.step()
            
            # Wait until it reach position
            dAngle = nu.subtract(currentAngle, angle_test)
            timeWaiting = nu.abs(nu.divide(dAngle,speedLimit))
            time.sleep(2)

            timeStep1 = time.time() - timeStart
            timeCurrent = time.time()

            # Joint angles Position 2
            angle_test[i] =  angle_test[i] + dtheta

            # Get Current Angle
            currentAngle = bot.get_theta_deg('right_arm')
            # Set the mode Theta with gravity compensation
            bot.set_mode_theta_gc('right_arm')
            # Set Desired Joint angles
            bot.set_theta_deg('right_arm',angle_test)
            # Set Stiffiness
            bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
            # Set Joint Speed [0 to 1]
            bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
            # Send Command        
            proxy.step()

            # Wait until it reach position
            dAngle = nu.subtract(currentAngle, angle_test)
            timeWaiting = nu.abs(nu.divide(dAngle,speedLimit))
            time.sleep(2)

            timeStep2 = time.time() - timeCurrent

            print( str(timeStep1) + " " + str(timeStep2) )

    # Joint angles Position 1
    angle_test = [0, 0, 0, 90.0, 108.2084705, 0, -5.3142586]
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angle_test)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
    # Send Command        
    proxy.step()

def testJointLowLevelControlResponseFromZeroPos():
    dtheta = 45
    dt = 0.1
    errorTolerance = 1
    speedLimit = nu.rad2deg([4.6,4.6,4.5,4.5,2.0,3.4,3.4]) # from documentation

    # Joint angles Position 1
    angle_test = [0, 0, 0, 0, 0, 0, 0]
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angle_test)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[0.3]*bot.get_num_dof('right_arm'))
    # Send Command        
    proxy.step()

    time.sleep(5)

    print("start")

    for i in range(7):
        rosbagProc = subprocess.Popen(["rosbag","record","/humanoid_state","-o","testJoint"+str(i)+"Control"],stdout=subprocess.PIPE)
        for j in range(3):
            timeStart = time.time()

            # Joint angles Position 1
            angle_test = [0, 0, 0, 0, 0, 0, 0]

            # Get Current Angle
            currentAngle = bot.get_theta_deg('right_arm')
            # Set the mode Theta with gravity compensation
            bot.set_mode_theta_gc('right_arm')
            # Set Desired Joint angles
            bot.set_theta_deg('right_arm',angle_test)
            # Set Stiffiness
            bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
            # Set Joint Speed [0 to 1]
            bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
            # Send Command        
            proxy.step()
            
            # Wait until it reach position
            dAngle = nu.subtract(currentAngle, angle_test)
            timeWaiting = nu.abs(nu.divide(dAngle,speedLimit))
            time.sleep(2)

            timeStep1 = time.time() - timeStart
            timeCurrent = time.time()

            # Joint angles Position 2
            angle_test[i] =  angle_test[i] + dtheta

            # Get Current Angle
            currentAngle = bot.get_theta_deg('right_arm')
            # Set the mode Theta with gravity compensation
            bot.set_mode_theta_gc('right_arm')
            # Set Desired Joint angles
            bot.set_theta_deg('right_arm',angle_test)
            # Set Stiffiness
            bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
            # Set Joint Speed [0 to 1]
            bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
            # Send Command        
            proxy.step()

            # Wait until it reach position
            dAngle = nu.subtract(currentAngle, angle_test)
            timeWaiting = nu.abs(nu.divide(dAngle,speedLimit))
            time.sleep(2)

            timeStep2 = time.time() - timeCurrent

            print( str(timeStep1) + " " + str(timeStep2) )
        terminate_process_and_children(rosbagProc)

    # Joint angles Position 1
    angle_test = [0, 0, 0, 0, 0, 0, 0]
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angle_test)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [1.0]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[1.0]*bot.get_num_dof('right_arm'))
    # Send Command        
    proxy.step()

def testProportionalControl():
    '''Meka Start Routine'''

    # Start Position -----------------
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angles_zero)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [0.3]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[0.7]*bot.get_num_dof('right_arm'))

    # Update command and status by calling proxy
    proxy.step()
    time.sleep(3)


    while(True):
        # Set the mode ThetaDot with gravity compensation
        bot.set_mode_vel_gc('right_arm')
        # Set Stiffiness
        bot.set_stiffness('right_arm', [0.1]*bot.get_num_dof('right_arm'))
        # Set Joint Speed [0 to 1]
        angle = bot.get_theta_deg('right_arm')
        error = angles_kojiref - angle
        # Set Velocity
        if(LA.norm(error) > 1):
            bot.set_thetadot_deg('right_arm', [0.1]*error)
        else:
            bot.set_thetadot_deg('right_arm', [-5.0]*bot.get_num_dof('right_arm'))


def testLowLevelControl():
    '''Meka Start Routine'''

    # Start Position -----------------
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_theta_deg('right_arm',angles_zero)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [0.3]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[0.7]*bot.get_num_dof('right_arm'))

    # Update command and status by calling proxy
    proxy.step()
    time.sleep(3)


    # Destination Position -----------------
    # Set the mode Theta with gravity compensation
    bot.set_mode_theta_gc('right_arm')
    # Set Desired Joint angles
    bot.set_thetadot_deg('right_arm',angles_zero)
    # Set Stiffiness
    bot.set_stiffness('right_arm', [0.3]*bot.get_num_dof('right_arm'))
    # Set Joint Speed [0 to 1]
    bot.set_slew_rate_proportion('right_arm',[0.7]*bot.get_num_dof('right_arm'))

    # Update command and status by calling proxy
    proxy.step()
    time.sleep(3)

try:
    testSetup()
#    testTimeDelayStatus()
#    testTimeDelayProxyStep()
#    testHoldPositionReference()
#    testTwoPoint()
#    testTwoPointTrajectorySmartDelay()
#    testJointLowLevelControlResponse()
#    testJointLowLevelControlResponseFromZeroPos()
#    testProportionalControl()

except KeyboardInterrupt:
    print('\n\nKeyboard exception received. Exiting...')

    # Move slowly to home position for safety sake
    bot.set_mode_theta_gc('right_arm')
    bot.set_theta_deg('right_arm',angles_zero)
    bot.set_stiffness('right_arm', [0.3]*bot.get_num_dof('right_arm'))
    bot.set_slew_rate_proportion('right_arm',[0.2]*bot.get_num_dof('right_arm'))
    proxy.step()
    time.sleep(5) # Wait the movement to finish

