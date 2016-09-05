#!/usr/bin/env python
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import tf

def printHelp():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callbackStop(msg):
    global leftSpeed
    global rightSpeed
    global rearSpeed
    global frontSpeed


    leftSpeed = 0
    rightSpeed = 0
    rearSpeed = 0
    frontSpeed = 0
    newSpeedData = True

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global frontSpeed   #w3
    global rearSpeed    #w4
    global newSpeedData
    #Speeds are assumed to come in float in [-1,1] for each tire. The values need to be transformed to values in [0,127]
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    leftSpeed = msg.data[0]
    rightSpeed = msg.data[1]
    frontSpeed = (rightSpeed - leftSpeed)/2.0
    rearSpeed = (leftSpeed - rightSpeed)/2.0

    maxValue = 0;
    if(math.fabs(leftSpeed) > maxValue):
        maxValue = math.fabs(leftSpeed);
    if(math.fabs(rightSpeed) > maxValue):
        maxValue = math.fabs(rightSpeed);
    if(math.fabs(frontSpeed) > maxValue):
        maxValue = math.fabs(frontSpeed);
    if(math.fabs(rearSpeed) > maxValue):
        maxValue = math.fabs(rearSpeed);

    if maxValue > 1.0:
        leftSpeed /= maxValue;
        rightSpeed /= maxValue;
        frontSpeed /= maxValue;
        rearSpeed /= maxValue;
    

    newSpeedData = True

def callbackCmdVel(msg):
    global leftSpeed    #w1
    global rightSpeed   #w2
    global frontSpeed   #w3
    global rearSpeed    #w4
    global newSpeedData

    leftSpeed = msg.linear.x - msg.angular.z * 0.48/2.0
    rightSpeed = msg.linear.x + msg.angular.z * 0.48/2.0
    frontSpeed = msg.linear.y + msg.angular.z * 0.48/2.0
    rearSpeed = msg.linear.y - msg.angular.z * 0.48/2.0

    maxValue = 0;
    if(math.fabs(leftSpeed) > maxValue):
        maxValue = math.fabs(leftSpeed);
    if(math.fabs(rightSpeed) > maxValue):
        maxValue = math.fabs(rightSpeed);
    if(math.fabs(frontSpeed) > maxValue):
        maxValue = math.fabs(frontSpeed);
    if(math.fabs(rearSpeed) > maxValue):
        maxValue = math.fabs(rearSpeed);

    if maxValue > 1.0:
        leftSpeed /= maxValue;
        rightSpeed /= maxValue;
        frontSpeed /= maxValue;
        rearSpeed /= maxValue;
        

    #if leftSpeed > 1:
    #    leftSpeed = 1
    #elif leftSpeed < -1:
    #    leftSpeed = -1

    #if rightSpeed > 1:
    #    rightSpeed = 1
    #elif rightSpeed < -1:
    #    rightSpeed = -1

    #if frontSpeed > 1:
    #    frontSpeed = 1
    #elif frontSpeed < -1:
    #    frontSpeed = -1
        
    #if rearSpeed > 1:
    #    rearSpeed = 1
    #elif rearSpeed < -1:
    #    rearSpeed = -1

    #print "leftSpeed: " + str(leftSpeed) + " rightSpeed: " + str(rightSpeed) + " frontSpeed: " + str(frontSpeed) + " rearSpeed: " + str(rearSpeed)
    newSpeedData = True

def calculateOdometry(currentPos, leftEnc, rightEnc, rearEnc, frontEnc): #Encoder measurements are assumed to be in ticks
    leftEnc = leftEnc / 158891.2 #From ticks to meters
    rightEnc = rightEnc /158891.2
    rearEnc = rearEnc / 336857.5
    frontEnc = frontEnc / 336857.5
    deltaTheta = (rightEnc - leftEnc + frontEnc - rearEnc)/0.48/2.0 #0.48 is the robot diameter
   
    if math.fabs(deltaTheta) >= 0.00001:
        rgX = (leftEnc + rightEnc)/(2*deltaTheta)
        rgY = (rearEnc + frontEnc)/(2*deltaTheta)
        deltaX = rgX*math.sin(deltaTheta) + rgY*(1-math.cos(deltaTheta))
        deltaY = rgX*(1-math.cos(deltaTheta)) + rgY*math.sin(deltaTheta)
    else:
        deltaX = (leftEnc + rightEnc)/2
        deltaY = (rearEnc + frontEnc)/2
    currentPos[0] += deltaX * math.cos(currentPos[2]) - deltaY * math.sin(currentPos[2])
    currentPos[1] += deltaX * math.sin(currentPos[2]) + deltaY * math.cos(currentPos[2])
    currentPos[2] += deltaTheta
    return currentPos
    

def main():
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."

    ###Connection with ROS
    rospy.init_node("omni_mobile_base")
    pubOdometry = rospy.Publisher("mobile_base/odometry", Odometry, queue_size = 1)

    subStop = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    subSpeeds = rospy.Subscriber("/hardware/mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    subCmdVel = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callbackCmdVel)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    
    ###Variables for setting tire speeds
    global leftSpeed
    global rightSpeed
    global frontSpeed
    global rearSpeed
    global newSpeedData
    leftSpeed = 0
    rightSpeed = 0
    frontSpeed = 0
    rearSpeed = 0
    newSpeedData = False
    speedCounter = 5
    ###Variables for odometry
    robotPos = [0, 0, 0]
   
    while not rospy.is_shutdown():
        if newSpeedData:
            newSpeedData = False
            speedCounter = 5
        else:
            speedCounter -= 1
            if speedCounter == 0:
                leftSpeed = 0
                rightSpeed = 0
                frontSpeed = 0
                rearSpeed = 0

            if speedCounter < -1:
                speedCounter = -1
        encoderLeft = leftSpeed * 0.05 * 158891.2
        encoderRight = rightSpeed * 0.05 * 158891.2
        encoderFront = frontSpeed * 0.05 * 336857.5
        encoderRear = rearSpeed * 0.05 * 336857.5
        ###Odometry calculation
        robotPos = calculateOdometry(robotPos, encoderLeft, encoderRight, encoderRear, encoderFront)
        #print "Encoders: " + str(encoderLeft) + "  " + str(encoderRight)
        ##Odometry and transformations
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "odom"
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = robotPos[0]
        ts.transform.translation.y = robotPos[1]
        ts.transform.translation.z = 0
        ts.transform.rotation = tf.transformations.quaternion_from_euler(0, 0, robotPos[2])
        br.sendTransform((robotPos[0], robotPos[1], 0), ts.transform.rotation, rospy.Time.now(), ts.child_frame_id, ts.header.frame_id)
        msgOdom = Odometry()
        msgOdom.header.stamp = rospy.Time.now()
        msgOdom.pose.pose.position.x = robotPos[0]
        msgOdom.pose.pose.position.y = robotPos[1]
        msgOdom.pose.pose.position.z = 0
        msgOdom.pose.pose.orientation.x = 0
        msgOdom.pose.pose.orientation.y = 0
        msgOdom.pose.pose.orientation.z = math.sin(robotPos[2]/2)
        msgOdom.pose.pose.orientation.w = math.cos(robotPos[2]/2)
        pubOdometry.publish(msgOdom)
        ###Reads battery and publishes the corresponding topic
        
        rate.sleep()
    #End of while
#end of main()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass