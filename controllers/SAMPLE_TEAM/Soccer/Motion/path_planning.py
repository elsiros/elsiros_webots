"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.
module can be used for optimized path planing of Robokit-1 robot. 
usage: create class PathPlan type object instance and call method path_calc_optimum.
Optionally module can be launched stand alone for purpose of tuning and observing result of 
path planing. Being launched stand alone module draws soccer field with player (white circle),
ball (orange circle), obstacles (black circles). Circles are movable by mouse dragging.
After each stop of mouse new path is drawing.  
"""
import sys
import os
import math
import array
import json
import random
import logging



goalPostRadius = 0.15       # Radius to walk around a goal post (in m).
ballRadius = 0.1           # Radius to walk around the ball (in m).
uprightRobotRadius = 0.2  # Radius to walk around an upright robot (in m).
roundAboutRadiusIncrement = 0.15


class PathPlan:
    """
    Plans optimized path of humanoid robot from start coordinate to target coordinate.
    Coordinates are taken together with orientation.
    Path is composed from initial arc, final arc and connecting line. 
    Connecting line must be tangent to arcs. 
    In case of obstacles on path line additional arc is added in order to go around obstacle.
    Only one obstacle can be avoided reliably. Avoiding of second obstacle is not guaranteed.
    Therefore there are used evaluations of prices of variants of path. The Path with cheaper
    price is returned. Collision with obstacle in far distance is cheaper than collision with 
    obstacle in near distance.
    During Path heuristic various radiuses of arcs are considered. Arc with zero radius means
    turning without changing coordinate.

    """
    def __init__(self, glob):
        self.glob = glob
        self.posts = [self.glob.landmarks["post1"][0], self.glob.landmarks["post2"][0], self.glob.landmarks["post3"][0], self.glob.landmarks["post4"][0]]
        self.goal_bottoms = [[[self.posts[0][0]+0.10, self.posts[0][1]],[self.posts[1][0]+0.10, self.posts[1][1]]],
                             [[self.posts[2][0]-0.10, self.posts[2][1]],[self.posts[3][0]-0.10, self.posts[3][1]]],
                             [[self.posts[0][0], self.posts[0][1]],[self.posts[0][0]+0.35, self.posts[0][1]]],
                             [[self.posts[1][0], self.posts[1][1]],[self.posts[1][0]+0.35, self.posts[1][1]]],
                             [[self.posts[2][0], self.posts[2][1]],[self.posts[2][0]-0.35, self.posts[2][1]]],
                             [[self.posts[3][0], self.posts[3][1]],[self.posts[3][0]-0.35, self.posts[3][1]]]]
        

    def coord2yaw(self, x, y):
        if x == 0:
            if y > 0 : yaw = math.pi/2
            else: yaw = -math.pi/2
        else: yaw = math.atan(y/x)
        if x < 0: 
            if yaw > 0: yaw -= math.pi
            else: yaw += math.pi
        return yaw

    def intersection_line_segment_and_line_segment(self, x1, y1, x2, y2, x3, y3, x4, y4 ):
        """
        Checks if 2 line segments have common point.
        Returns:
            True - if there is common point
            False - if not.

        x = x1 + (x2 - x1) * t1    t1 - paramentric coordinate
        y = y1 + (y2 - y1) * t1
        x = x3 + (x4 - x3) * t2    t2 - paramentric coordinate
        y = y3 + (y4 - y3) * t2
        x1 + (x2 - x1) * t1 = x3 + (x4 - x3) * t2
        y1 + (y2 - y1) * t1 = y3 + (y4 - y3) * t2
        t1 = (x3 + (x4 - x3) * t2 - x1) / (x2 - x1)
        t1 = (y3 + (y4 - y3) * t2 - y1) / (y2 - y1)
        y1 + (y2 - y1) * (x3 + (x4 - x3) * t2 - x1) / (x2 - x1) = y3 + (y4 - y3) * t2
        (y2 - y1) * (x4 - x3)/ (x2 - x1) * t2 - (y4 - y3) * t2 = y3 - y1 - (y2 - y1) * (x3 - x1) / (x2 - x1)
        t2 = (y3 - y1 - (y2 - y1) * (x3 - x1) / (x2 - x1)) /((y2 - y1) * (x4 - x3)/ (x2 - x1) - (y4 - y3))
        if t1 == 0:
            t2 = (y1 - y3)/ (y4 - y3)
            t2 = (x1 - x3)/ (x4 - x3)
        """
        if x2 == x1:
            if x4 == x3:
                if x1 == x3 and max(y1,y2) >= min(y3,y4) and max(y3, y4) >= min(y1,y2):
                    return True
                else: return False
            elif y2 == y1:
                if x3 == x4:
                    if y3 == y4:
                        if x1 == x3 and y1 == y3:
                            return True
                        else: return False
                    else:
                        t2 = (y1 - y3)/ (y4 - y3)
                        if x1 == x3 and 0 <= round(t2, 4) <= 1:
                            return True
                        else: return False
                else:
                    dt =  (y1 - y3)/ (y4 - y3) - (x1 - x3)/ (x4 - x3)
                    if round(dt, 4) == 0:
                        return True
                    else: return False
            else:
                t2 = (x1 - x3)/(x4 - x3)
                t1 = (y3 - y1 + (y4 - y3) * t2) / (y2 - y1)
                if 0 <= round(t1, 4) <= 1 and 0 <= round(t2, 4) <= 1:
                    return True
                else: return False
        else:
            if (y2 - y1) * (x4 - x3) == (y4 - y3) * (x2 - x1):
                if max(x1,x2) >= min(x3,x4) and max(x3, x4) >= min(x1,x2): 
                    return True
                else: return False
            else:
                t2 = (y3 - y1 - (y2 - y1) * (x3 - x1) / (x2 - x1)) /((y2 - y1) * (x4 - x3)/ (x2 - x1) - (y4 - y3))
                t1 = (x3 + (x4 - x3) * t2 - x1) / (x2 - x1)
                if 0 <= round(t1, 4) <= 1 and 0 <= round(t2, 4) <= 1:
                    return True
                else: return False


    def intersection_line_segment_and_circle(self, x1, y1, x2, y2, xc, yc, R):
        """
        Checks if line segment and circle have common points.
        Returns:
            True - if there is common point
            False - if not.

        x = x1 + (x2 - x1) * t    t - paramentric coordinate
        y = y1 + (y2 - y1) * t
        R**2 = (x - xc)**2 + (y - yc)**2
        (x1 + (x2 - x1) * t - xc)**2 + (y1 + (y2 - y1) * t - yc)**2 - R**2 = 0
        ((x2 - x1) * t)**2 + (x1 - xc)**2 + 2 * (x2 - x1) * (x1 - xc) * t + 
        ((y2 - y1) * t)**2 + (y1 - yc)**2 + 2 * (y2 - y1) * (y1 - yc) * t - R**2 = 0
        ((x2 - x1)**2 + (y2 - y1)**2) * t**2 + (2 * (x2 - x1) * (x1 - xc) + 2 * (y2 - y1) * (y1 - yc)) * t +
        (x1 - xc)**2 + (y1 - yc)**2 - R**2 = 0
        a * t**2 + b * t + c = 0
        a = (x2 - x1)**2 + (y2 - y1)**2
        b = 2 * (x2 - x1) * (x1 - xc) + 2 * (y2 - y1) * (y1 - yc)
        c = (x1 - xc)**2 + (y1 - yc)**2 - R**2
        """
        a = (x2 - x1)**2 + (y2 - y1)**2
        b = 2 * (x2 - x1) * (x1 - xc) + 2 * (y2 - y1) * (y1 - yc)
        c = (x1 - xc)**2 + (y1 - yc)**2 - R**2
        successCode, t1, t2 = self.square_equation(a,b,c)
        if successCode:
            if 0 <= round(t1, 4) <= 1 or 0 <= round(t2, 4) <= 1 or (t1 > 1 and t2 < 0) or (t2 > 1 and t1 < 0):
               return True
        return False

    def intersection_circle_segment_and_circle(self, x1, y1, x2, y2, x0, y0, CW, xc, yc, R):
        R0sq = (x1 - x0)**2 + (y1 - y0)**2 
        if yc == y0:
            xp1 = xp2 = (R0sq - R**2 + xc**2 - x0**2)/(2 * (xc - x0))
            if R0sq - (xp1 - x0)**2 < 0: return False 
            yp1 = y0 + math.sqrt(R0sq - (xp1 - x0)**2)
            yp2 = y0 - math.sqrt(R0sq - (xp1 - x0)**2)
        else:
            A = (x0 - xc)/(yc - y0)
            B = (R0sq - R**2 + xc**2 + yc**2 - x0**2 - y0**2)/(2*(yc - y0))
            a = 1 + A**2
            b = 2 * (-A * y0 + A * B - x0)
            c = x0**2 - R0sq + (B - y0)**2
            succsessCode, xp1, xp2 = self.square_equation(a, b, c)
            if not succsessCode: return False
            yp1 = A * xp1 + B
            yp2 = A * xp2 + B
        alpha2 = self.coord2yaw(x2 - x0, y2 - y0)
        alpha1 = self.coord2yaw(x1 - x0, y1 - y0)
        alphap2 = self.coord2yaw(xp2 - x0, yp2 - y0)
        alphap1 = self.coord2yaw(xp1 - x0, yp1 - y0)
        if CW:
            if alpha1 < alpha2: 
                alpha1 += math.pi * 2
                if alphap1 < 0 : alphap1 += math.pi * 2
                if alphap2 < 0 : alphap2 += math.pi * 2
            if alpha2 <= alphap1 <= alpha1 or alpha2 <= alphap2 <= alpha1:
                return True
        else:
            if alpha2 < alpha1:
                alpha2 += math.pi * 2
                if alphap1 < 0 : alphap1 += math.pi * 2
                if alphap2 < 0 : alphap2 += math.pi * 2
            if alpha1 <= alphap1 <= alpha2 or alpha1 <= alphap2 <= alpha2:
                return True
        return False

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi 
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def delta_yaw(self, start_yaw, dest_yaw, CW):
        s = math.degrees(start_yaw)
        d = math.degrees(dest_yaw)
        if CW:
            if start_yaw < dest_yaw:
                start_yaw += math.pi * 2
        else:
            if dest_yaw < start_yaw:
                dest_yaw += math.pi * 2
        delta_yaw = dest_yaw - start_yaw
        #print('CW = ', CW, 'start_yaw = ', s, 'dest_yaw = ', d, 'delta_yaw = ', math.degrees(delta_yaw))
        return delta_yaw

    def path_calc_optimum(self, start_coord, target_coord):
        """
        Returns optimized humanoid robot path. 
        usage:
            list:             dest, list: centers, int: number_Of_Cycles = self.path_calc_optimum(list: start_coord, list: target_coord)
            dest:             list of destination point coordinates. Each coordinate is list or tuple of floats [x,y]. 
                              Each coordinate is starting or end point of path segment. Path comprises of following segments: 
                              circle segment, line segment, n*(circle segment, line segment), circle segment. Where n - iterable. 
            centers:          list of coordinates of circle centers of circle segments of path. Each coordinate is list or tuple of floats [x,y].
            number_Of_Cycles: integer which represents price of path. In case if value is >100 then collision with second obstacle on path
                              is not verified. 
            start_coord:      list or tuple of floats [x, y, yaw]
            target_coord:     list or tuple of floats [x, y, yaw]
        """
        #print('obstacles:', self.glob.obstacles)
        dest, centers, number_Of_Cycles = self.path_calc(start_coord, target_coord)
        if len(centers) > 0:
            x1, y1, x2, y2, cx, cy, R, CW = centers[0]
            if R <= 0.08:
                start_yaw = start_coord[2]
                dest_yaw = self.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1])
                delta_yaw = self.delta_yaw(start_yaw, dest_yaw, CW)
                if abs(delta_yaw) > math.pi: centers[0][7] = not CW
            x1, y1, x2, y2, cx, cy, R, CW = centers[len(centers)-1]
            if R <= 0.08:
                start_yaw = self.coord2yaw(dest[len(dest)-1][0] - dest[len(dest)-2][0], dest[len(dest)-1][1] - dest[len(dest)-2][1])
                dest_yaw = target_coord[2]
                delta_yaw = self.delta_yaw(start_yaw, dest_yaw, CW)
                if abs(delta_yaw) > math.pi: centers[len(centers)-1][7] = not CW
        return dest, centers, number_Of_Cycles

    def path_calc(self, start_coord, target_coord):
        x1, y1, yaw1 = start_coord 
        x2, y2, yaw2 = target_coord
        dest1, centers1, number_Of_Cycles1 = self.arc_path_internal( x1, y1, yaw1, x2, y2, yaw2)
        dest2, centers2, number_Of_Cycles2 = self.arc_path_external( x1, y1, yaw1, x2, y2, yaw2)
        if number_Of_Cycles1 < number_Of_Cycles2: return dest1, centers1, number_Of_Cycles1
        else: return dest2, centers2, number_Of_Cycles2
        return dest1, centers1

    def arc_path_external(self, x1, y1, yaw1, x2, y2, yaw2):
        number_Of_Cycles_min = 1000
        for i in range(10):
            for j in range(10):
                R1 = i * 0.05
                R2 = j * 0.05
                #R1 = 0.2
                #R2 = 0.2
                if (y2-y1) < 0:
                    xc1 = x1 - R1 * math.sin(yaw1)
                    yc1 = y1 + R1 * math.cos(yaw1)
                    xc2 = x2 - R2 * math.sin(yaw2)
                    yc2 = y2 + R2 * math.cos(yaw2)
                    CW1 = False
                    CW2 = False
                else:
                    xc1 = x1 + R1 * math.sin(yaw1)
                    yc1 = y1 - R1 * math.cos(yaw1)
                    xc2 = x2 + R2 * math.sin(yaw2)
                    yc2 = y2 - R2 * math.cos(yaw2)
                    CW1 = True
                    CW2 = True
                successCode, xp1, yp1 = self.external_tangent_line(True, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW1 )
                if successCode:
                    successCode, xp2, yp2 = self.external_tangent_line(False, R2, R1, x2, y2, xc2, yc2, xc1, yc1, CW2 )
                    if successCode:
                        dest = [[xp1,yp1], [xp2,yp2]]
                        centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1], [xp2, yp2, x2, y2, xc2, yc2, R2, CW2]]
                        nearestObstacle = self.check_Obstacle(xp1, yp1, xp2, yp2)
                        if nearestObstacle >= 0 :
                            roundAboutRadius = self.glob.obstacles[nearestObstacle][2] / 2 + roundAboutRadiusIncrement
                        #if self.intersection_line_segment_and_circle(xp1, yp1, xp2, yp2,
                        #                             self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                            for variant in range(2):
                                if variant == 0:
                                    CW = CW1
                                    successCode1, xp1, yp1 = self.external_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode2, xp2, yp2 = self.external_tangent_line(False,
                                           roundAboutRadius, R1, x2, y2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode3, xp3, yp3 = self.external_tangent_line(True,
                                           roundAboutRadius, R2, x1, y1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode4, xp4, yp4 = self.external_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode1 and successCode2 and successCode3 and successCode4) : continue
                                if variant == 1:
                                    CW = not CW1
                                    successCode5, xp1, yp1 = self.internal_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode6, xp2, yp2 = self.internal_tangent_line(False,
                                           roundAboutRadius, R1, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode7, xp3, yp3 = self.internal_tangent_line(True,
                                           roundAboutRadius, R2, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode8, xp4, yp4 = self.internal_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode5 and successCode6 and successCode7 and successCode8) : continue
                                dest = [[xp1,yp1], [xp2,yp2], [xp3,yp3], [xp4,yp4]]
                                centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1],
                                            [xp2, yp2, xp3, yp3, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], roundAboutRadius, CW],
                                            [xp4, yp4, x2, y2, xc2, yc2, R2, CW2]]
                                price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                                number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                                if number_Of_Cycles < number_Of_Cycles_min:
                                    number_Of_Cycles_min = number_Of_Cycles
                                    dest_min = dest
                                    centers_min = centers
                        else:
                            price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                            number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                            if number_Of_Cycles < number_Of_Cycles_min:
                                number_Of_Cycles_min = number_Of_Cycles
                                dest_min = dest
                                centers_min = centers
        if number_Of_Cycles_min == 1000: return [], [], number_Of_Cycles_min
        else: return dest_min, centers_min, number_Of_Cycles_min

    def check_Obstacle(self, xp1, yp1, xp2, yp2):
        nearestObstacle = -1
        obstacles = []
        distances = []
        for j in range(len(self.glob.obstacles)):
            roundAboutRadius = self.glob.obstacles[j][2] / 2 + roundAboutRadiusIncrement
            if self.intersection_line_segment_and_circle(xp1, yp1, xp2, yp2,
                                            self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius):
                obstacles.append(j)
                distances.append(math.sqrt((xp1 - self.glob.obstacles[j][0])**2 + (yp1 - self.glob.obstacles[j][1])**2))
        if obstacles != []:
            nearestObstacle = obstacles[distances.index(min(distances))]
        return nearestObstacle

    def check_Limits(self, x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest):
        permit = True
        for i in range(0, len(dest), 2):
            if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                        ballRadius + roundAboutRadiusIncrement): permit = False
        ind = len(dest) - 1
        if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                    ballRadius + roundAboutRadiusIncrement): permit = False
        if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                      self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius): permit = False
        if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius): permit = False

        for j in range(4):
            goalPostX, goalPostY = self.glob.landmarks["unsorted_posts"][j]
            if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    goalPostX, goalPostY, goalPostRadius): permit = False
            if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                    goalPostX, goalPostY, goalPostRadius): permit = False
            for i in range(0, len(dest), 2):
                if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         goalPostX, goalPostY, goalPostRadius): permit = False
        #if permit == False: print('permit denied')
        return permit

    def check_Price(self, x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers):
        price = 0
        for i in range(0, len(dest), 2):
            if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                         ballRadius + roundAboutRadiusIncrement): price += 200
        ind = len(dest) - 1
        if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    self.glob.ball_coord[0], self.glob.ball_coord[1],
                                                    ballRadius + roundAboutRadiusIncrement): price += 200
        for j in range(len(self.glob.obstacles)):
            roundAboutRadius = self.glob.obstacles[j][2] / 2 + roundAboutRadiusIncrement
            if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                          self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius): price += 200
            if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                        self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius): price += 100
            for i in range(0, len(dest), 2):
                    if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         self.glob.obstacles[j][0], self.glob.obstacles[j][1], roundAboutRadius): price += 300 - i * 100
        for j in range(4):
            goalPostX, goalPostY = self.posts[j]
            if self.intersection_circle_segment_and_circle(dest[ind][0], dest[ind][1], x2, y2, xc2, yc2, CW2,
                                                    goalPostX, goalPostY, goalPostRadius): price += 200
            if self.intersection_circle_segment_and_circle(x1, y1, dest[0][0], dest[0][1], xc1, yc1, CW1,
                                                    goalPostX, goalPostY, goalPostRadius): price += 300
            for i in range(0, len(dest), 2):
                if self.intersection_line_segment_and_circle(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                                                         goalPostX, goalPostY, goalPostRadius): price += 300 - i * 100

        for j in range(6):
            for i in range(0, len(dest), 2):
                if self.intersection_line_segment_and_line_segment(dest[i][0], dest[i][1], dest[i + 1][0], dest[i + 1][1], 
                   self.goal_bottoms[j][0][0], self.goal_bottoms[j][0][1], self.goal_bottoms[j][1][0], self.goal_bottoms[j][1][1]):
                   price += 300 # - i * 100
            for i in range(len(centers)):
                if self.intersection_line_segment_and_circle(self.goal_bottoms[j][0][0], self.goal_bottoms[j][0][1],
                   self.goal_bottoms[j][1][0], self.goal_bottoms[j][1][1], 
                   centers[i][4], centers[i][5], centers[i][6]): price += 300 #- i * 100

        return price

    def number_Of_Cycles_count(self, dest, centers, yaw1, yaw2):
        prop_yaw_glob1 = self.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1])
        x1, y1, x2, y2, cx, cy, R, CW = centers[0]
        prop_yaw_local1 = self.delta_yaw(yaw1, prop_yaw_glob1, CW)
        number_Of_Cycles = math.ceil(abs(prop_yaw_local1 / 0.2))
        while True:
            delta_yaw_step = prop_yaw_local1 / number_Of_Cycles
            stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.glob.cycle_step_yield * 1.1
            if stepLength <= 64: break
            else: number_Of_Cycles += 1
        for i in range(0,len(dest),2):
            distance = math.sqrt((dest[i+1][0] - dest[i][0])**2 + (dest[i+1][1] - dest[i][1])**2)
            number_Of_Cycles += math.ceil(abs(distance / (self.glob.cycle_step_yield/1000)))
            x1, y1, x2, y2, cx, cy, R, CW = centers[int(i/2+1)]
            if len(dest) == i+2 : 
                prop_yaw_local2 = self.delta_yaw(prop_yaw_glob1, yaw2, CW)
            else:
                prop_yaw_glob2 = self.coord2yaw(dest[i+3][0] - dest[i+2][0], dest[i+3][1] - dest[i+2][1])
                prop_yaw_local2 = self.delta_yaw(prop_yaw_glob1, prop_yaw_glob2, CW)
                prop_yaw_glob1 = prop_yaw_glob2
            number_Of_Cycles2 = math.ceil(abs(prop_yaw_local2 / 0.2))
            while True:
                delta_yaw_step = prop_yaw_local2 / number_Of_Cycles2
                stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.glob.cycle_step_yield * 1.1
                if stepLength <= 64: break
                else: number_Of_Cycles2 += 1
            number_Of_Cycles += number_Of_Cycles2
        return number_Of_Cycles


    def external_tangent_line(self, start, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW ):
        if R1 == 0: return True, x1, y1
        L = math.sqrt((xc2 - xc1)**2 + (yc2 - yc1)**2)
        if R1 == R2:
            xp1 = [xc1 + (yc2-yc1) * R1 /L, xc1 - (yc2-yc1) * R1/L]
            yp1 = [yc1 - (xc2-xc1) * R1/L, yc1 + (xc2-xc1) * R1/L]
        else:
            L1 = L * R1 / abs(R1 - R2)
            L2 = L * R2 / abs(R1 - R2)
            x0 = (R2 * xc1 - R1 * xc2) / (R2 - R1)
            y0 = (R2 * yc1 - R1 * yc2) / (R2 - R1) 
            if yc1 == y0:
                xp = (L1**2 - 2 * R1**2 + xc1**2 - x0**2)/(2 * xc1 - 2 * x0)
                xp1 = [xp, xp]
                tmp = R1**2 - (xp - xc1)**2
                if tmp < 0 : return False, 0, 0
                yp1 = [yc1 + math.sqrt(tmp), yc1 - math.sqrt(tmp)]
            else:
                A = (x0 - xc1) / (yc1 - y0)
                B = (L1**2 - 2 * R1**2 + xc1**2 + yc1**2 - x0**2 - y0**2) / (2 * yc1 - 2 * y0)
                ap = 1 + A**2
                bp = 2 * (-A * yc1 + A * B - xc1)
                cp = xc1**2 + (B - yc1)**2 - R1**2
                successCode, xp10, xp11 = self.square_equation(ap, bp, cp)
                if not successCode: 
                    return False, 0, 0
                xp1 =[xp10, xp11]
                yp1 = [A * xp1[0] + B, A * xp1[1] + B]
        #alpha_1 = self.coord2yaw(x1 - xc1, y1 - yc1)
        al0 = self.coord2yaw(xp1[0] - xc1, yp1[0] - yc1)
        al1 = self.coord2yaw(xp1[1] - xc1, yp1[1] - yc1)
        if CW: da = -math.pi/2
        else: da = math.pi/2
        directToOtherEnd = self.coord2yaw(xc2 - xc1, yc2 - yc1)
        alpha_p1 = [abs(self.norm_yaw(da + al0 - directToOtherEnd)),
                    abs(self.norm_yaw(da + al1 - directToOtherEnd))]
        if start:
            ind = alpha_p1.index(min(alpha_p1))
        else: ind = alpha_p1.index(max(alpha_p1))
        return True, xp1[ind], yp1[ind]

    def arc_path_internal(self, x1, y1, yaw1, x2, y2, yaw2):
        number_Of_Cycles_min = 1000
        for i in range(10):
            for j in range(10):
                R1 = i * 0.05
                R2 = j * 0.05
                #R1 = 0.2
                #R2 = 0.2
                if (y2-y1) < 0:
                    xc1 = x1 + R1 * math.sin(yaw1)
                    yc1 = y1 - R1 * math.cos(yaw1)
                    xc2 = x2 - R2 * math.sin(yaw2)
                    yc2 = y2 + R2 * math.cos(yaw2)
                    CW1 = True
                    CW2 = False
                else:
                    xc1 = x1 - R1 * math.sin(yaw1)
                    yc1 = y1 + R1 * math.cos(yaw1)
                    xc2 = x2 + R2 * math.sin(yaw2)
                    yc2 = y2 - R2 * math.cos(yaw2)
                    CW1 = False
                    CW2 = True
                successCode, xp1, yp1 = self.internal_tangent_line(True, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW1)
                if successCode:
                    successCode, xp2, yp2 = self.internal_tangent_line(False, R2, R1, x2, y2, xc2, yc2, xc1, yc1, CW2)
                    if successCode:
                        dest = [[xp1,yp1], [xp2,yp2]]
                        centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1], [xp2, yp2, x2, y2, xc2, yc2, R2, CW2]]
                        nearestObstacle = self.check_Obstacle(xp1, yp1, xp2, yp2)
                        if nearestObstacle >= 0 :
                            roundAboutRadius = self.glob.obstacles[nearestObstacle][2] / 2 + roundAboutRadiusIncrement
                        #if self.intersection_line_segment_and_circle(xp1, yp1, xp2, yp2,
                        #                             self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                            for variant in range(2):
                                if variant == 0:
                                    CW = CW1
                                    successCode1, xp1, yp1 = self.external_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode2, xp2, yp2 = self.external_tangent_line(False,
                                           roundAboutRadius, R1, x2, y2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode3, xp3, yp3 = self.internal_tangent_line(True,
                                           roundAboutRadius, R2, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode4, xp4, yp4 = self.internal_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode1 and successCode2 and successCode3 and successCode4) : continue
                                if variant == 1:
                                    CW = not CW1
                                    successCode5, xp1, yp1 = self.internal_tangent_line(True,
                                           R1, roundAboutRadius, x1, y1, xc1, yc1, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW1)
                                    successCode6, xp2, yp2 = self.internal_tangent_line(False,
                                           roundAboutRadius, R1, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc1, yc1, CW)
                                    successCode7, xp3, yp3 = self.external_tangent_line(True,
                                           roundAboutRadius, R2, xp2, yp2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], xc2, yc2, CW)
                                    successCode8, xp4, yp4 = self.external_tangent_line(False,
                                           R2, roundAboutRadius, x2, y2, xc2, yc2, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], CW2)
                                    if not (successCode5 and successCode6 and successCode7 and successCode8) : continue
                                dest = [[xp1,yp1], [xp2,yp2], [xp3,yp3], [xp4,yp4]]
                                centers = [[x1, y1, xp1, yp1, xc1, yc1, R1, CW1],
                                            [xp2, yp2, xp3, yp3, self.glob.obstacles[nearestObstacle][0], self.glob.obstacles[nearestObstacle][1], roundAboutRadius, CW],
                                            [xp4, yp4, x2, y2, xc2, yc2, R2, CW2]]
                                price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                                number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                                if number_Of_Cycles < number_Of_Cycles_min:
                                    number_Of_Cycles_min = number_Of_Cycles
                                    dest_min = dest
                                    centers_min = centers
                        else:
                            price = self.check_Price(x1, y1, x2, y2, xp1, yp1, xp2, yp2, xc1, yc1, CW1, xc2, yc2, CW2, dest, centers)
                            number_Of_Cycles = self.number_Of_Cycles_count(dest, centers, yaw1, yaw2) + price
                            if number_Of_Cycles < number_Of_Cycles_min:
                                number_Of_Cycles_min = number_Of_Cycles
                                dest_min = dest
                                centers_min = centers
        if number_Of_Cycles_min == 1000: return [], [], number_Of_Cycles_min
        else: return dest_min, centers_min, number_Of_Cycles_min

    def internal_tangent_line(self, start, R1, R2, x1, y1, xc1, yc1, xc2, yc2, CW ):
        if R1 == 0: return True, x1, y1
        L = math.sqrt((xc2 - xc1)**2 + (yc2 - yc1)**2)
        L1 = L * R1/(R1 +R2)
        x3 = xc1 + (xc2 - xc1) * R1 / (R1 + R2)
        y3 = yc1 + (yc2 - yc1) * R1 / (R1 + R2)
        if round(y3, 4) != round(yc1, 4):
            A = - (x3 - xc1) / (y3 - yc1)
            B = ( 2 * R1**2 - L1**2 - xc1**2 + x3**2 - yc1**2 + y3**2)/ 2 /(y3 - yc1)
            a = 1 + A**2
            b = 2 * A *(B - yc1) - 2 * xc1
            c = xc1**2 + (B - yc1)**2 - R1**2
            succsessCode, xp10, xp11 = self.square_equation(a, b , c)
            if not succsessCode: 
                return False, 0, 0
            xp1 =[xp10, xp11]
            yp1 = [A * xp1[0] + B, A * xp1[1] + B]
        else:
            tmp1 = ( R1**2 - L1**2 - xc1**2 + x3**2 - yc1**2 + y3**2)/ 2 /(x3 - xc1)
            xp1 = [tmp1,tmp1]
            ttt = R1**2 - (tmp1 - xc1)**2
            if ttt < 0: return False, 0, 0
            tmp2 = math.sqrt(ttt)
            yp1 = [yc1 + tmp2, yc1 - tmp2]
        #alpha_1 = self.coord2yaw(x1 - xc1, y1 - yc1)
        al0 = self.coord2yaw(xp1[0] - xc1, yp1[0] - yc1)
        al1 = self.coord2yaw(xp1[1] - xc1, yp1[1] - yc1)
        if CW: da = -math.pi/2
        else: da = math.pi/2
        directToOtherEnd = self.coord2yaw(xc2 - xc1, yc2 - yc1)
        alpha_p1 = [abs(self.norm_yaw(da + al0 - directToOtherEnd)),
                    abs(self.norm_yaw(da + al1 - directToOtherEnd))]
        if start:
            ind = alpha_p1.index(min(alpha_p1))
        else: ind = alpha_p1.index(max(alpha_p1))
        return True, xp1[ind], yp1[ind]


    def square_equation(self, a,b,c):
        D = b**2 - 4 * a * c
        if D < 0: return False, 0, 0
        return True, (-b + math.sqrt(D))/(2 * a), (-b - math.sqrt(D))/(2 * a)




if __name__ == '__main__':
    import wx
    class Glob:
        def __init__(self):
            self.COLUMNS = 18
            self.ROWS = 13
            self.pf_coord =    [0.288127801937314, -0.1567013686848328, 0.23820464086940252]  #[-0.4, 0.0 , 0] # [0.5, 0.5 , -math.pi * 3/4]
            #self.ball_coord = [-0.132, 0.957]        #[0, 0]
            #self.obstacles = [[0.4, 0.025, 0.2], [0.725, -0.475, 0.2]]  #[[0, 0, 0.15], [0.4, 0.025, 0.2], [0.725, -0.475, 0.2]]
            self.ball_coord = [0.9000400221229459, -0.64993850847569]
            self.obstacles = [[-1.40272770229783, 0.00015398849619637726, 0.2], [1.4679380912231852, -0.31451040267467, 0.2], [0.35446599449729804, -0.1052386048459205, 0.2]]
            self.landmarks = {"post1": [[ 1.8, -0.6 ]], "post2": [[ 1.8, 0.6 ]], "post3": [[ -1.8, 0.6 ]], "post4": [[ -1.8, -0.6 ]],
                              "unsorted_posts": [[ 1.8, 0.6 ],[ 1.8, -0.6 ],[ -1.8, 0.6 ],[ -1.8, -0.6 ]],
                              "FIELD_WIDTH": 2.6, "FIELD_LENGTH": 3.6 }
            self.params = {'CYCLE_STEP_YIELD': 103.5}
            self.cycle_step_yield = 103.5
            current_work_directory = os.getcwd()
            current_work_directory = current_work_directory.replace('\\', '/')
            current_work_directory += '/'
            self.strategy_data = array.array('b',(0 for i in range(self.COLUMNS * self.ROWS * 2)))
            self.import_strategy_data(current_work_directory)

        def import_strategy_data(self, current_work_directory):
            with open(current_work_directory + "Init_params/strategy_data.json", "r") as f:
                loaded_Dict = json.loads(f.read())
            if loaded_Dict.get('strategy_data') != None:
                strategy_data = loaded_Dict['strategy_data']
            for column in range(self.COLUMNS):
                for row in range(self.ROWS):
                    index1 = column * self.ROWS + row
                    power = strategy_data[index1][2]
                    yaw = int(strategy_data[index1][3] * 40)  # yaw in radians multiplied by 40
                    self.strategy_data[index1*2] = power
                    self.strategy_data[index1*2+1] = yaw

    class Forward:
        def __init__(self, glob):
            self.glob = glob

        def direction_To_Guest(self):
            if self.glob.ball_coord[0] < 0: 
                return 0
            elif self.glob.ball_coord[0] > 0.8 and abs(self.glob.ball_coord[1]) > 0.6:
                return math.atan(-self.glob.ball_coord[1]/(1.8-self.glob.ball_coord[0]))
            elif self.glob.ball_coord[0] < 1.5 and abs(self.glob.ball_coord[1]) < 0.25:
                if (1.8-self.glob.ball_coord[0]) == 0: return 0
                else: 
                    if abs(self.glob.ball_coord[1]) < 0.2:
                        return math.atan((math.copysign(0.5, self.glob.ball_coord[1])-
                                                           self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
                    else: 
                        return math.atan((0.5* (round(random.random(),0)*2 - 1)-
                                                           self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
            else:
                return math.atan(-self.glob.pf_coord[1]/(2.4-self.glob.pf_coord[0]))

    class Forward_Vector_Matrix:
        def __init__(self, glob):
            self.glob = glob
            #self.direction_To_Guest = 0
            self.kick_Power = 1
        

        def direction_To_Guest(self):
            if abs(self.glob.ball_coord[0])  >  self.glob.landmarks["FIELD_LENGTH"] / 2:
                ball_x = math.copysign(self.glob.landmarks["FIELD_LENGTH"] / 2, self.glob.ball_coord[0])
            else: ball_x = self.glob.ball_coord[0]
            if abs(self.glob.ball_coord[1])  >  self.glob.landmarks["FIELD_WIDTH"] / 2:
                ball_y = math.copysign(self.glob.landmarks["FIELD_WIDTH"] / 2, self.glob.ball_coord[1])
            else: ball_y = self.glob.ball_coord[1]
            col = math.floor((ball_x + self.glob.landmarks["FIELD_LENGTH"] / 2) / (self.glob.landmarks["FIELD_LENGTH"] / self.glob.COLUMNS))
            row = math.floor((- ball_y + self.glob.landmarks["FIELD_WIDTH"] / 2) / (self.glob.landmarks["FIELD_WIDTH"] / self.glob.ROWS))
            if col >= self.glob.COLUMNS : col = self.glob.COLUMNS - 1
            if row >= self.glob.ROWS : row = self.glob.ROWS -1
            direction_To_Guest = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2 + 1] / 40
            self.kick_Power = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2]
            print('direction_To_Guest = ', math.degrees(direction_To_Guest))
            return direction_To_Guest

    class Example(wx.Frame):

        def __init__(self, *args, **kw):
            #super().__init__(*args, **kw)
            super(Example, self).__init__(*args, **kw)
            self.glob = Glob()
            self.p = PathPlan(self.glob)
            self.f = Forward_Vector_Matrix(self.glob)
            self.isLeftDown = False
            self.InitUI()

        def InitUI(self):

            self.Bind(wx.EVT_PAINT, self.OnPaint)
            self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
            self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)
            self.Bind(wx.EVT_MOTION, self.OnMove)

            self.SetTitle('Lines')
            self.Centre()

        def OnPaint(self, e):
            self.dc = wx.PaintDC(self)
            #dc.SetBackground(wx.Brush('#1ac500'))
            #dc.SetPen(wx.Pen('#d4d4d4'))
            self.SetClientSize(800, 600)
            self.dc.SetBrush(wx.Brush('#1ac500'))
            self.dc.DrawRectangle(0, 0, 800, 600)
            pen = wx.Pen('#ffffff', 10, wx.SOLID)
            pen.SetJoin(wx.JOIN_MITER)
            self.dc.SetPen(pen)
            self.dc.DrawRectangle(40, 40, 720, 520)
            self.dc.DrawRectangle(0, 200, 40, 200)
            self.dc.DrawRectangle(760, 200, 40, 200)
            self.dc.DrawRectangle(40, 160, 40, 280)
            self.dc.DrawRectangle(720, 160, 40, 280)
            self.dc.DrawCircle(400,300,60)
            self.dc.DrawLine(400,45,400,555)
            self.dc.DrawLine(390,300,410,300)
            self.dc.DrawLine(210,300,230,300)
            self.dc.DrawLine(220,290,220,310)
            self.dc.DrawLine(570,300,590,300)
            self.dc.DrawLine(580,290,580,310)
        
            self.dc.SetAxisOrientation(True, True)
            self.dc.SetDeviceOrigin(400, 300)

            pen1 = wx.Pen('#000000', 1, wx.SOLID)
            self.dc.SetPen(pen1)
            self.dc.SetBrush(wx.Brush('#ffffff'))
            self.dc.DrawCircle(int(self.glob.pf_coord[0] *200), int(self.glob.pf_coord[1] *200) , 20)  #robot
            self.dc.SetBrush(wx.Brush('#000000'))
            for obstacle in self.glob.obstacles:
                self.dc.DrawCircle(int(obstacle[0] *200), int(obstacle[1] *200), int(obstacle[2] *100))      # obstacle
            self.dc.SetBrush(wx.Brush('#ff0000'))
            self.dc.DrawCircle(int(self.glob.ball_coord[0] *200), int(self.glob.ball_coord[1] *200), 8)   # ball
            target_yaw = self.f.direction_To_Guest()
            dest = []
            centers = []
            number_Of_Cycles = 1000
            #for i in range(4):
            #    target_x = self.glob.ball_coord[0] - (0.11 + i * 0.05) * math.cos(target_yaw)
            #    target_y = self.glob.ball_coord[1] - (0.11 + i * 0.05) * math.sin(target_yaw)
            #    target_coord = [target_x, target_y, target_yaw]
            #    dest1, centers1, number_Of_Cycles1 = self.p.path_calc_optimum(self.glob.pf_coord, target_coord)
            #    if number_Of_Cycles1 <= number_Of_Cycles:
            #        dest = dest1
            #        centers = centers1
            #        number_Of_Cycles = number_Of_Cycles1
            for i in range(5):
                for j in range(2):
                    target_x = self.glob.ball_coord[0] - (0.21 + j * 0.05) * math.cos(target_yaw - 0.8 + i * 0.4)
                    target_y = self.glob.ball_coord[1] - (0.21 + j * 0.05) * math.sin(target_yaw - 0.8 + i * 0.4)
                    target_coord = [target_x, target_y, target_yaw]
                    dest1, centers1, number_Of_Cycles1 = self.p.path_calc_optimum(self.glob.pf_coord, target_coord)
                    if i != 2: number_Of_Cycles1 += 50
                    if number_Of_Cycles1 <= number_Of_Cycles:
                        dest = dest1
                        centers = centers1
                        number_Of_Cycles = number_Of_Cycles1
            print('number_Of_Cycles= ', number_Of_Cycles)
            #print('centers =', centers)
            if len(dest)==0: 
                print('Impossible')
                return
            #print(d)
            for i in range(0,len(dest),2):
                self.dc.DrawLine((int(dest[i][0] *200), int(dest[i][1] *200)), (int(dest[i+1][0] *200), int(dest[i+1][1] *200)))
            if self.p.intersection_line_segment_and_circle(dest[0][0], dest[0][1], dest[1][0], dest[1][1], self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
               print('intersection with line')
            for i in range(len(centers)):
                x1, y1, x2, y2, cx, cy, R, CW = centers[i]
                self.draw_arc(x1*200, y1*200, x2*200, y2*200, cx*200, cy*200, CW)
            if len(dest) == 2:
                if self.p.intersection_circle_segment_and_circle(dest[1][0], dest[1][1], target_x, target_y,
                                                              centers[1][0], centers[1][1], CW, self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                    print('intersection with circle')
            if len(dest) == 4:
                if self.p.intersection_circle_segment_and_circle(dest[3][0], dest[3][1], target_x, target_y,
                                                              centers[1][0], centers[1][1], CW, self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                    print('intersection with circle')
            if self.p.intersection_circle_segment_and_circle(self.glob.pf_coord[0], self.glob.pf_coord[1], dest[0][0],
                                                          dest[0][1], centers[0][0], centers[0][1], CW,
                                                          self.glob.obstacles[0][0], self.glob.obstacles[0][1], uprightRobotRadius):
                print('intersection with circle')

            #dc.Bind()

        def draw_arc(self, x1, y1, x2, y2, cx, cy, CW):
            self.dc.SetBrush(wx.Brush('#ff0000', wx.BRUSHSTYLE_TRANSPARENT))
            if CW: self.dc.DrawArc(int(x2), int(y2), int(x1), int(y1), int(cx), int(cy))
            else: self.dc.DrawArc(int(x1), int(y1), int(x2), int(y2), int(cx), int(cy))

        def OnLeftDown(self, event):
            #dc = wx.ClientDC(self.staticBMP)
            pos = event.GetLogicalPosition(self.dc)
            self.isLeftDown = True
            if math.sqrt((pos[0]-self.glob.pf_coord[0]*200)**2 + (pos[1]-self.glob.pf_coord[1]*200)**2) <= 20: 
                self.moving_object = -1       #'robot'
                self.dx = self.glob.pf_coord[0]*200 - pos[0]
                self.dy = self.glob.pf_coord[1]*200 - pos[1]
            elif math.sqrt((pos[0]-self.glob.ball_coord[0]*200)**2 + (pos[1]-self.glob.ball_coord[1]*200)**2) <= 8: 
                self.moving_object = -2    #'ball'
                self.dx = self.glob.ball_coord[0]*200 - pos[0]
                self.dy = self.glob.ball_coord[1]*200 - pos[1]
            else: self.isLeftDown = False
            if not self.isLeftDown:
                for obstacle in self.glob.obstacles:
                    if math.sqrt((pos[0]-obstacle[0]*200)**2 + (pos[1]-obstacle[1]*200)**2) <= 20:
                        self.isLeftDown = True
                        self.moving_object = self.glob.obstacles.index(obstacle)      #'obstacle'
                        self.dx = obstacle[0]*200 - pos[0]
                        self.dy = obstacle[1]*200 - pos[1]
                        break
            #dc.DrawCircle(pos[0], pos[1], 5)
            a = 1
        
        def OnLeftUp(self, event):
            self.isLeftDown = False

        def OnMove(self, event):
            if self.isLeftDown:
                pos = event.GetLogicalPosition(self.dc)
                if self.moving_object == -1:                         #'robot'
                    self.glob.pf_coord = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200, self.glob.pf_coord[2]]
                    self.Refresh()
                if self.moving_object == -2:                          # 'ball'
                    self.glob.ball_coord = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200]
                    #self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
                    self.Refresh()
                if self.moving_object >= 0:                             # 'obstacle'
                    self.glob.obstacles[self.moving_object] = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200, self.glob.obstacles[self.moving_object][2]]
                    self.Refresh()

    def main():

        app = wx.App()
        ex = Example(None)
        ex.Show()
        app.MainLoop()

    main()