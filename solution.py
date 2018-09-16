#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {student full name}
# {student id}
# {student email}

import math

def solution(car):

    #controls=[math.pi/4.0, math.pi/4.0, -math.pi/4.0]
    #times=[0,1,2,3]

    # initial conditions
    x, y = car.x0, car.x0
    theta = 0
    phi = math.pi/4.0
    t = 0

    # numerical integration
    xl, yl, thetal, phil, tl = [x], [y], [theta], [], [t]
    while tl[-1] < 15:
        xn, yn, thetan = car.step(xl[-1], yl[-1], thetal[-1], phi)
        print("pos itr {} : {}".format(tl[-1],yn))
        xl.append(xn)
        yl.append(yn)
        thetal.append(theta)
        phil.append(phi)
        tl.append(tl[-1] + car.dt)

    return phil, tl

    #return controls, times
