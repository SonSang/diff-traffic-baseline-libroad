import numpy
import math
import scipy
import scipy.linalg

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
    nsteps = 100
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

def plot_stuff():
    arc = math.pi/2
    nx = 1
    ny = 100
    nradii = 1
    radii =   [1.0]#numpy.linspace(-1, 1, nradii)
    yangles = numpy.linspace(0, math.pi/2, ny)
    xangles = [0.0]
    dists = numpy.zeros((nx, ny, nradii))
    for (ri, radius) in enumerate(radii):
        for (xi, xangle) in enumerate(xangles):
            for (yi, yangle) in enumerate(yangles):
                frame = numpy.array([[1.0, 0.0, 0.0, 0.0],
                                     [0.0, 1.0, 0.0, 0.0],
                                     [0.0, 0.0, 1.0, 0.0],
                                     [0.0, 0.0, 0.0, 1.0]])

                frame = numpy.dot(axis_angle_matrix(arc/2,  numpy.array([0.0, 0.0, 1.0])), frame)
                frame = numpy.dot(axis_angle_matrix(xangle, numpy.array([1.0, 0.0, 0.0])), frame)
                frame = numpy.dot(axis_angle_matrix(yangle, numpy.array([0.0, 1.0, 0.0])), frame)

                pts = circle_points(frame, 10.0, radius, arc)
                for i in xrange(1,len(pts)):
                    dists[xi,yi,ri] += scipy.linalg.norm(pts[i]-pts[i-1])


    pylab.clf()
    for x in xrange(nx):
        for r in xrange(nradii):
            pylab.plot(yangles, dists[x,:,r] - 10.0*arc)
    pylab.show()

def

if __name__ == '__main__':



