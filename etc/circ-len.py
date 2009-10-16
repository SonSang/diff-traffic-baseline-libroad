import numpy
import math
import scipy
import scipy.linalg
import scipy.integrate as integ

import pylab

def axis_angle_matrix(theta, axis):
    c = math.cos(theta)
    s = math.sin(theta)
    ux, uy, uz = axis[0], axis[1], axis[2]
    return numpy.array(
        [[ux*ux + (1.0-ux*ux)*c, ux*uy*(1.0-c) -  uz*s,  ux*uz*(1.0-c) +  uy*s, 0.0],
         [ux*uy*(1.0-c) +  uz*s, uy*uy + (1.0-uy*uy)*c,  uy*uz*(1.0-c) -  ux*s, 0.0],
         [ux*uz*(1.0-c) -  uy*s, uy*uz*(1.0-c) +  ux*s,  uz*uz + (1.0-uz*uz)*c, 0.0],
         [0.0, 0.0, 0.0, 1.0]])

def cross(nb, nf):
    """[ x  y  z]
       [bx by bz]
       [fx fy fz]"""
    return numpy.array([
        nb[1]*nf[2] - nf[1]*nb[2],
        nb[2]*nf[0] - nf[2]*nb[0],
        nb[0]*nf[1] - nf[0]*nb[1]])

def circle_frame(theta, matrix, radius):
    s = math.sin(theta)
    c = math.cos(theta)
    numpy.dot(matrix, numpy.array([radius*c, radius*s, 0.0, 1.0]))
    return (numpy.dot(matrix, numpy.array([radius*c, radius*s, 0.0, 1.0]))[:3],
            numpy.dot(matrix[0:3,0:3], numpy.array([-s, c, 0.0])))

def circle_points(frame, radius, offset, arc, up=numpy.array([0.0, 0.0, 1.0])):
    nsteps = 800
    dtheta = arc/float(nsteps-1)
    points = []
    for i in xrange(nsteps):
        theta = i*dtheta
        (pos, tan) = circle_frame(theta, frame, radius)
        left = cross(up, tan)
        llen = scipy.linalg.norm(left)
        left /= llen

        points.append(pos + left*offset)

    return points

def circ_len(phi, r, l, arc):
    yangle = phi
    frame = numpy.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]])

    frame = numpy.dot(axis_angle_matrix(math.pi/2-arc, numpy.array([0.0, 0.0, 1.0])), frame)
    frame = numpy.dot(axis_angle_matrix(yangle, numpy.array([0.0, 1.0, 0.0])), frame)

    pts = circle_points(frame, r, l, arc)
    dist = 0
    for i in xrange(1,len(pts)):
        dist += scipy.linalg.norm(pts[i]-pts[i-1])
    return dist

def circ_arc(theta, phi, l, r):
    c_theta = math.cos(theta)
    s_theta = math.sin(theta)
    c_phi = math.cos(phi)
    s_phi = math.sin(phi)

    numerator = math.sqrt(s_phi**4*r**2*c_theta**4-math.sqrt(1-s_phi**2*c_theta**2)* (2*l*c_phi*s_phi**2*r*c_theta**2-2*l*c_phi*r)-2*s_phi**2*r**2*c_theta**2+r**2-l**2*s_phi**2+ l**2)

    denom = s_theta**2+c_phi**2*c_theta**2

    return numerator/denom

if __name__ == '__main__':
    radius = 1.0
    offset = 1.0

    phi = math.pi/2#math.pi/4
    arc = math.pi/3

    print (math.pi/2-arc/2, arc/2+math.pi/2)

    val = integ.quad(lambda x: circ_arc(x, phi, offset, radius),
                     math.pi/2-arc/2, arc/2+math.pi/2)

    print val

    print circ_len(phi, radius, -offset, arc)

