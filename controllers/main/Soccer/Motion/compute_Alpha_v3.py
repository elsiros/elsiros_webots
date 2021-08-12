"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev.
Module can be used for Inverted Kinematics for legs of Robokit-1 robot.
Advantage of module against other IK implementations is fast and repeatable 
calculation benchmark.
Result is achieved due to mixed analytic/numerical calculation method.
Module is designed for 6 DOF robot leg. From 6 angles one angle is calculated 
using numerical iterations, other 5 angles are obtained through polynom
roots formula calculation. This way prowides fast benchmark and repeatability.
Algorithm being implemented in C language with integration into firnware of 
OpenMV is capable to calculated angles for robot legs within time less than 1ms.
Multiple IK solutions are filtered through applying of angle limits within calculation.
This yields less time for calculation.
usage: create class Alpha instance and call method compute_Alpha_v3 with arguments.
Returns list of 0, 1 or 2 lists of servo angles. List of 0 elements means that
IK was not solved. List of 1 list means 1 possible solition is detected.
List of 2 lists means that plurality of solutions was not filtered by provided arguments.


"""
import math, time

#@micropython.native
class Alpha():

    def compute_Alpha_v3(self, xt,yt,zt,x,y,z,w, sizes, limAlpha):
        """
        usage: list: angles = self.compute_Alpha_v3(float: xt, float: yt, float: zt,
                              float: x, float: y, float: z, float: w,
                              list: sizes, list: limAlpha)
        angles: list of floats angles in radians of servos which provide target positioning and
                orientation of robots' foot
        xt:     target x coordinate of foots' center point
        yt:     target y coordinate of foots' center point
        zt:     target z coordinate of foots' center point
        x:      x coordinate of vector of orientation of foot
        y:      y coordinate of vector of orientation of foot
        z:      z coordinate of vector of orientation of foot
        w:      rotation in radians of foot around vector of orientation
        sizes:  list of sizes defining distances between servo axles in biped implementation
        limAlpha: list of limits [minimum, maximum] of servomotors measured in number of encoder ticks
                  of Kondo series 2500 servomotors.
        Target coordinates are measured in local robot coordinate system XYZ with ENU orientation. 
        [0,0,0] point of coordinate system is linked to pelvis of robot.
        Foot orientation vector has length 1. Base of vector is at bottom of foot and tip of vector
        is directed down when foot is on floor.   

        """
        from math import sqrt,cos,sin,asin,fabs,tan,atan
        #t1_start =time.perf_counter_ns()
        a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 = sizes
        limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10 = limAlpha
        alpha5 = w
        cos5 = math.cos(alpha5)
        sin5 = math.sin(alpha5)
        nor = math.sqrt(x*x+y*y+z*z)
        x = x/nor
        y = y/nor
        z = z/nor
        xtp = xt * cos5 + (yt + a5) * sin5
        ytp = (yt + a5) * cos5 - xt * sin5
        ztp = zt
        xp =  x * cos5 + y * sin5
        yp = y * cos5 - x * sin5
        zp = z
        var = [-1,1]
        angles = []
        lim1a= limAlpha6[0]*0.00058909
        lim2a = limAlpha6[1]*0.00058909
        ind = 1
        step1 = (lim2a-lim1a)/10
        testalpha6 =[]
        for i in range (11):
            alpha6 = lim1a+i*step1
            cos= math.cos(alpha6)
            sin= math.sin(alpha6)
            testalpha6.append( ((ytp+b5)*cos+ztp*sin -c10)*((yp*cos+zp*sin)**2-
                (zp*cos-yp*sin)**2 -xp*xp)-a10-b10*(yp*cos+zp*sin)/math.sqrt((zp*cos-yp*sin)**2+xp*xp))
        #print(testalpha6)
        points = []
        for i in range(10):
            if (testalpha6[i]>0 and testalpha6[i+1]<0)or(testalpha6[i]<0 and testalpha6[i+1]>0): points.append(i)
        k=0
        if len(points)==0:
            for i in range(11):
                if (math.fabs(testalpha6[i]) < math.fabs(testalpha6[k])): k=i
            if k==10: points.append(9)
            else:
                if (math.fabs(testalpha6[k-1]) < math.fabs(testalpha6[k+1])): points.append(k-1)
                else: points.append(k)
        alpha6m = []
        for j in range(len(points)):
            lim1=lim1a+points[j]*step1
            lim2=lim1+step1
            while (True):
                step = (lim2-lim1)/10
                testalpha6 =[]
                for i in range (11):
                    alpha6 = lim1+i*step
                    cos= math.cos(alpha6)
                    sin= math.sin(alpha6)
                    testalpha6.append( ((ytp+b5)*cos+ztp*sin-c10)*((yp*cos+zp*sin)**2-
                        (zp*cos-yp*sin)**2 -xp*xp)-a10-b10*(yp*cos+zp*sin)/math.sqrt((zp*cos-yp*sin)**2+xp*xp))
                k=0
                for i in range(11):
                    if (math.fabs(testalpha6[i]) < math.fabs(testalpha6[k])): k = i
                if k==0: k2=1
                elif k==10: k2 = 9
                else:
                    if (math.fabs(testalpha6[k-1]) < math.fabs(testalpha6[k+1])): k2=k-1
                    else: k2=k+1
                alpha6 = lim1+k*step
                if k>k2:
                    lim1 = lim1+k2*step
                    lim2 = lim1+ step
                else:
                    lim1 = lim1+k*step
                    lim2 = lim1+ step
                if (lim2-lim1 < 0.00025): break
                ind = ind + 1
                if ind> (limAlpha6[1]- limAlpha6[0]): break
            alpha6m.append(alpha6)
        alpha10m =[]
        kk=0
        #t1_stop =time.perf_counter_ns()
        #print('time t1 elapsed= ',(t1_stop-t1_start))
        for i in range (len(alpha6m)):
            tan6 = math.tan(alpha6m[i-kk])
            alpha10 = math.atan((-yp-zp*tan6)/math.sqrt((zp-yp*tan6)**2+xp*xp*(1+tan6*tan6)))
            if limAlpha10[0] < alpha10*1698 and alpha10*1698<limAlpha10[1]: alpha10m.append(alpha10)
            else:
                alpha6m.pop(i-kk)
                kk=kk+1
        #print('alpha10m = ', alpha10m)
        kk=0
        #t1_stop =time.perf_counter_ns()
        #print('time t2 elapsed= ',(t1_stop-t1_start))
        for ii in range (len(alpha6m)):
            cos6 = math.cos(alpha6m[ii-kk])
            sin6 = math.sin(alpha6m[ii-kk])
            alpha987 = math.atan(-xp/(zp*cos6- yp*sin6))
            sin987 = math.sin(alpha987)
            cos987 = math.cos(alpha987)
            K1 = a6*sin987+xtp*cos987+(ztp*cos6-(ytp+b5)*sin6)*sin987
            K2 = a9+a6*cos987+(ztp*cos6-(ytp+b5)*sin6)*cos987-xtp*sin987+b10/math.cos(alpha10m[ii-kk])+((ytp+b5)*cos6+ztp*sin6-c10)*math.tan(alpha10m[ii-kk])
            m = (K1*K1+K2*K2+a8*a8-a7*a7)/(2*a8)


            temp1 = K1*K1*m*m-(K1*K1+K2*K2)*(m*m-K2*K2)
            if temp1>=0 :
                temp2 = (-K1*m + math.sqrt(temp1))/(K1*K1+K2*K2)
                temp3 = (-K1*m - math.sqrt(temp1))/(K1*K1+K2*K2)
                if math.fabs(temp2) <= 1 and math.fabs(temp3) <= 1:
                    alpha91 = math.asin(temp2)
                    alpha92 = math.asin(temp3)
                else:
                    alpha6m.pop(ii-kk)
                    alpha10m.pop(ii-kk)
                    kk=kk+1
                    continue
            else:
                alpha6m.pop(ii-kk)
                alpha10m.pop(ii-kk)
                kk=kk+1
                continue
            alpha81 = math.atan((K1+a8*math.sin(alpha91))/(K2+a8*math.cos(alpha91))) - alpha91
            alpha82 = math.atan((K1+a8*math.sin(alpha92))/(K2+a8*math.cos(alpha92))) - alpha92
            alpha71 = alpha91+alpha81- alpha987
            alpha72 = alpha92+alpha82- alpha987
            #t1_stop =time.perf_counter_ns()
            #print('time t3 elapsed= ',(t1_stop-t1_start))
            temp71 = alpha71*1698<limAlpha7[0] or alpha71*1698>limAlpha7[1]
            temp72 = alpha72*1698<limAlpha7[0] or alpha72*1698>limAlpha7[1]
            temp81 = alpha81*1698<limAlpha8[0] or alpha81*1698>limAlpha8[1]
            temp82 = alpha82*1698<limAlpha8[0] or alpha82*1698>limAlpha8[1]
            temp91 = alpha91*1698<limAlpha9[0] or alpha91*1698>limAlpha9[1]
            temp92 = alpha92*1698<limAlpha9[0] or alpha92*1698>limAlpha9[1]
            if (temp71 and temp72) or (temp81 and temp82) or (temp91 and temp92) or ((temp71 or temp81 or temp91) and (temp72 or temp82 or temp92)):
                alpha6m.pop(ii-kk)
                alpha10m.pop(ii-kk)
                kk=kk+1
                continue
            else:
                if not (temp71 or temp81 or temp91):
                    ang =()
                    ang =alpha10m[ii-kk],alpha91,alpha81,alpha71,alpha6m[ii-kk],alpha5
                    angles.append(ang)
                if not (temp72 or temp82 or temp92):
                    ang =()
                    ang =alpha10m[ii-kk],alpha92,alpha82,alpha72,alpha6m[ii-kk],alpha5
                    angles.append(ang)
        #t1_stop =time.perf_counter_ns()
        #print('time t4 elapsed= ',(t1_stop-t1_start))
        return angles

if __name__ == "__main__":
    a5 = 21.5  # мм расстояние от оси симметрии до оси сервы 5
    b5 = 18.5  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
    c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
    a6 = 42    # мм расстояние от оси сервы 6 до оси сервы 7
    a7 = 65.5  # мм расстояние от оси сервы 7 до оси сервы 8
    a8 = 63.8  # мм расстояние от оси сервы 8 до оси сервы 9
    a9 = 35.5  # мм расстояние от оси сервы 9 до оси сервы 10
    a10= 25.4  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
    b10= 16.4  # мм расстояние от оси сервы 10 до низа стопы
    c10 = 12   # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
    sizes = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]

    limAlpha5 = [-2667, 2667]
    limAlpha6 = [-3000,  740]
    limAlpha7 = [-3555, 3260]
    limAlpha8 = [-4150, 1777]
    limAlpha9 = [-4000, 2960]
    limAlpha10 =[-2815,   600]
    limAlpha = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]

    clock1 = time.clock()
    clock1.tick()
    compute_Alpha_v3(0,-54.3,-200,0,0,-1,0, sizes, limAlpha)
    print('time elapsed in compute_Alpha:', clock1.avg())



