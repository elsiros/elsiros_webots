
# Module is designed by Vladimir Tkach



import math
from .ball_Approach_calc import ball_Approach_Calc


def uprint(*text):
    print(*text )

def normalize_rotation(yaw):
    if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
    if yaw > math.pi : yaw -= (2 * math.pi)
    if yaw < -math.pi : yaw += (2 * math.pi)
    if yaw > 0.5 : yaw = 0.5
    if yaw < -0.5 : yaw = -0.5
    return yaw

def steps(motion, x1,y1,u1,x2,y2,u2): #returns list of steps [[step forward/back,step left/right,degrees of rotation,the number of steps]]
    x1 *= 1000
    y1 *= 1000
    x2 *= 1000
    y2 *= 1000
    stepLength = 64
    first_step_yield = motion.first_step_yield    #46
    cycle_step_yield = motion.cycle_step_yield    #96 #78.2
    rotation_angle = 12
    rotation_angle_yield = 13.2
    a=[]
    b=[]
    Dx=x2-x1 #calculating the x length
    Dy=y2-y1 #calculating the y length
    S=(Dx**2+Dy**2)**0.5 #calculating the distance to destination
    if S > (first_step_yield/4+cycle_step_yield*3/4):
        b=[stepLength/4,0,0,1]
        a.append(b)
        b=[stepLength/2,0,0,1]
        a.append(b)
    n=(S-first_step_yield)/cycle_step_yield+1 #calculating the number of steps
    n1=math.floor(n)+1 #calculating the number of full steps
    #Ds=S-(first_step_yield+cycle_step_yield*(n1-1)) #calculating the remains of the distance
    #steplenght_T=Ds/(first_step_yield+cycle_step_yield) * stepLength #calculating the length of the little steps
    if Dx == 0:
        if Dy > 0: uB = math.pi/2
        if Dy < 0: uB = -math.pi/2
        if Dy == 0: uB = u1
    else:
        uB=math.atan(Dy/Dx)  #calculating B angle in radians
        #uB=uB*180/math.pi #makeing degrees from radians
    if Dx < 0: uB = uB + math.pi
    U=uB-u1 #calculating the rotation angle before starting to go
    if U > math.pi : U = U - math.pi *2
    if U < -math.pi : U = U + math.pi *2
    #uprint ('U =', U)
    U2=u2-uB #calculating the rotation angle when the robot is on the destinatiob point
    if U2 > math.pi : U2 = U2 - math.pi *2
    if U2 < -math.pi : U2 = U2 + math.pi *2
    #print(U2)
    Rotates=int(abs(U)/rotation_angle_yield) #calculating the full number of rotates
    if Rotates == 0: Rotation_Angle = 0
    else: Rotation_Angle = U/(rotation_angle_yield*Rotates)* rotation_angle
    if Rotates>0 : a.append([0,0,Rotation_Angle,Rotates])

    if S!=0:
        stepLength = stepLength * (S/(first_step_yield*1.25+cycle_step_yield*(n1-1)+ cycle_step_yield*0.75 ))
        #print('stepLength =', stepLength)
        b=[stepLength,0,0,n1+2]
        a.append(b)
        #b=[steplenght_T,0,0,2]
        #a.append(b)
    Rotates=int(math.ceil(abs(U2)/rotation_angle_yield))
    #Rotates=int(abs(U2//rotation_angle_yield))+1 #calculating the angle at which the robot must look when he is near the ball
    Rotation_Angle = U2/(rotation_angle_yield*Rotates)* rotation_angle
    if Rotates>0 : a.append([0,0,Rotation_Angle,Rotates])
    return b, uB         #returns a-list value and walk direction

def ball_Approach( motion, local , glob, ball_coord):
    x1 =  glob.pf_coord[0]
    y1 =  glob.pf_coord[1]
    destination = ball_Approach_Calc(glob, ball_coord)                          # computation of destination points
    stop_points = len(destination)
    for stop_point in range (stop_points):
        if stop_point > 0:                                                      # in case of 2 or more stop points x1 and y1 are re-appointed
            x1,y1,u1 = destination[stop_point-1]
        u1 =  glob.pf_coord[2]                                                              #motion.imu_body_yaw()
        x2,y2,u2 = destination[stop_point]
        step_Seq, walk_Direction = steps(motion, x1 , y1, u1, x2, y2, u2)
        if stop_point == 0: motion.turn_To_Course(walk_Direction)
        stepLength, sideLength, rotation, cycleNumber = step_Seq
        if stepLength == 0 and sideLength == 0: continue
        motion.walk_Initial_Pose()
        for cycle in range(cycleNumber):
            motion.refresh_Orientation()
            rotation1 = rotation
            if rotation == 0:
                rotation1 = (walk_Direction - motion.imu_body_yaw())*1
                rotation1 = normalize_rotation(rotation1)
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/4
            if cycle ==1 : stepLength1 = stepLength/2
            motion.walk_Cycle(stepLength1, sideLength,rotation1,cycle,cycleNumber)
        motion.walk_Final_Pose()
        motion.turn_To_Course(math.radians(u2))


if __name__=="__main__":
    print('This is not main module!')

