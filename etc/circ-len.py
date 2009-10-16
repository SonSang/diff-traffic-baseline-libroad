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
    nsteps = 100
    dtheta = arc/float(nsteps-1)
    points = []
    for i in xrange(nsteps):

        theta = i*dtheta
        (pos, tan) = circle_frame(theta, frame, radius)
        left = cross(up, tan)
        llen = scipy.linalg.norm(left)
        if(llen > 1e-6):
            left /= llen
        points.append(pos + left*offset)

    return points

def circ_len(phi, r, l, arc):
    yangle = phi
    frame = numpy.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]])

    frame = numpy.dot(axis_angle_matrix(math.pi/2, numpy.array([0.0, 0.0, 1.0])), frame)
    frame = numpy.dot(axis_angle_matrix(yangle, numpy.array([0.0, 1.0, 0.0])), frame)

    pts = circle_points(frame, r, l, arc)

    dist = 0
    for i in xrange(1,len(pts)):
        dist += scipy.linalg.norm(pts[i]-pts[i-1])
    return dist

def circ_arc(theta, phi, l, r):
    c2_theta = math.cos(theta)**2
    s2_theta = math.sin(theta)**2
    c_phi    = math.cos(phi)
    s2_phi   = math.sin(phi)**2

    numerator = math.sqrt((r**2)*(s2_phi*c2_theta*(s2_phi*c2_theta - 2) + 1)
                          + l*c_phi*(2*r*math.pow(1-s2_phi*c2_theta, 1.5) + l*c_phi))

    denom = s2_theta + c_phi**2*c2_theta

    return numerator/denom

def circ_point(theta, phi, r):
    c_theta = math.cos(theta)
    s_theta = math.sin(theta)
    c_phi = math.cos(phi)
    s_phi = math.sin(phi)

    return numpy.array([ r*s_theta*c_phi,
                         r*c_theta,
                         r*s_theta*s_phi ])

def offs_vec(theta, phi):
    c_theta = math.cos(theta)
    s_theta = math.sin(theta)
    c_phi = math.cos(phi)

    v = numpy.array([ s_theta,
                      c_theta*c_phi,
                      0 ])
    vlen = scipy.linalg.norm(v)
    v /= vlen
    return v

def offs_point(theta, phi, r, l):
    return circ_point(theta, phi, r) + l*offs_vec(theta, phi)

def plot_phi_len(radius, offset, phi, arc, num=100):
    circ_offs = 2*math.pi*(radius+offset)*(arc/(2*math.pi))
    circ = 2*math.pi*(radius)*(arc/(2*math.pi))

    dat = numpy.zeros((100, 2))
    for (ct,i) in enumerate(numpy.linspace(0, math.pi/2, dat.shape[0])):
        dat[ct,0] = i
        dat[ct,1] = integ.quad(lambda x: circ_arc(x, i, offset, radius),
                               math.pi/2-arc/2, math.pi/2+arc/2)[0]

    print 'circ+offs', circ_offs
    print 'circ', circ

    pylab.clf()
    pylab.plot(dat[:,0], dat[:,1])
    pylab.plot(dat[:,0], numpy.ones(dat.shape[0])*circ_offs)
    pylab.plot(dat[:,0], numpy.ones(dat.shape[0])*circ)
    # xcoeff = numpy.polyfit(dat[:,0], numpy.divide((dat[:, 1]-circ), circ_offs-circ), 4)
    # px = numpy.poly1d(xcoeff)
    pylab.show()

if __name__ == '__main__':
    plot_phi_len(1.0, 1.0, math.pi/2, math.pi)

