import numpy
import math
import scipy
import scipy.linalg
import scipy.integrate as integ
import time

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

class circ_arc_o(object):
    def __init__(self, phi, l, r):
        self.s2_phi   = math.sin(phi)**2
        self.r2 = r**2
        self.rt_coeff = 2*l*r*math.cos(phi)
        self.numer_expr = self.r2 + l**2*(1.0 - self.s2_phi)
    def __call__(self, theta):
        subexpr = self.s2_phi*math.cos(theta)**2
        subexpr1m = (1 - subexpr)

        numerator = math.sqrt(self.r2*subexpr*(subexpr-2)
                              + self.rt_coeff*math.sqrt(subexpr1m)*subexpr1m
                              + self.numer_expr)

        return numerator/subexpr1m

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

def plot_dist():
    dat = numpy.zeros((10, 10, 100, 2))
    for (pct,phi) in enumerate(numpy.linspace(0, 7*math.pi/16, dat.shape[0])):
        for (oct,offs) in enumerate(numpy.linspace(0, 10.0, dat.shape[1])):
            for (dct,i) in enumerate(numpy.linspace(0, math.pi, dat.shape[2])):
                dat[pct,oct,dct,0] = i
                dat[pct,oct,dct,1] = (integ.quad(lambda x: circ_arc(x, phi, 10.0, offs),
                                                 0, i)[0] - i)/offs

    pylab.clf()
    for i in xrange(dat.shape[1]):
        pylab.plot(dat[4,i,:,0], dat[3,i,:,1])
    pylab.show()

if __name__ == '__main__':
    # plot_phi_len(1.0, 1.0, math.pi/2, math.pi)

    # phi = 4*math.pi/9
    # rad = 1.0
    # rad_fact = 0.5
    # dat = numpy.zeros((10,20))
    # fact = numpy.linspace(0.1, 10.0, dat.shape[0])
    # for (rct,rad_fact) in enumerate(fact):
    #     for (oct,offs) in enumerate(numpy.linspace(-rad_fact, rad_fact, dat.shape[1])):
    #         dat[rct,oct] = integ.quad(lambda x: circ_arc(x, phi, rad, offs),
    #                                   0, math.pi)[0] - rad*math.pi

    # c = numpy.polyfit(offs_x, dat[0,:], 5)
    # p = numpy.poly1d(c)

    # pylab.clf()
    # for i in xrange(len(fact)):
    #     pylab.plot(offs_x, dat[i,:], '+-')
    #     pylab.plot(offs_x, p(numpy.divide(offs_x,fact[i])))
    # pylab.show()

    phi = math.pi/4
    rad = 1.0
    offs = 1.0

    pylab.clf()
    cv = numpy.vectorize(lambda x: circ_arc(x, phi, rad, offs))

    t1 = time.time()
    res = integ.quad(cv, 0, math.pi, limit=1, epsabs=1e-10, epsrel=1e-10, full_output=True)
    t2 = time.time()
    print '%s took %0.3f ms' % ("quad", (t2-t1)*1000.0)

    t1 = time.time()
    q_res = integ.quadrature(cv, 0, math.pi)
    t2 = time.time()
    print '%s took %0.3f ms' % ("quadrature", (t2-t1)*1000.0)

    x = numpy.linspace(0.0, math.pi, 21)

    t1 = time.time()
    t_res = integ.trapz(cv(x), x)
    t2 = time.time()
    print '%s took %0.3f ms' % ("trapz", (t2-t1)*1000.0)

    t1 = time.time()
    s_res = integ.simps(cv(x), x)
    t2 = time.time()
    print '%s took %0.3f ms' % ("simps", (t2-t1)*1000.0)

    # t1 = time.time()
    # r_res = integ.romb(cv(x), x[1]-x[0])
    # t2 = time.time()
    # print '%s took %0.3f ms' % ("romb", (t2-t1)*1000.0)

    t1 = time.time()
    z = cv(math.pi/2)
    t2 = time.time()
    print '%s took %0.3f ms' % ("one eval", (t2-t1)*1000.0)

    print res[0], res[1], res[2]['neval']
    print q_res
    print t_res
    print s_res
    # print r_res

    # fact = numpy.linspace(0.0, math.pi, 50)
    # pylab.plot(fact, cv(fact))
    # pylab.show()


