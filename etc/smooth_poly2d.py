#!/usr/bin/python

import numpy
import math
import pylab
import matplotlib
import scipy.linalg

def tan_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt(1 - dot*dot) / ( 1 + dot )

def cot_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return ( 1 + dot ) / math.sqrt(1 - dot*dot)

def orientation(nb, nf):
    return nb[0]*nf[1] - nf[0]*nb[1]

def rot_pi(v):
    return numpy.array([-v[1], v[0]])

def rot_n_pi(v):
    return numpy.array([v[1], -v[0]])

def circle(center, radius, angint, ccw, nsteps):
    angint2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in angint ]
    inorder = angint2[0] <= angint2[1]
    adist = abs(angint2[1] - angint2[0])
    if (not ccw and inorder) or (ccw and not inorder):
        adist = 2*math.pi - adist
    if not ccw:
        adist *= -1
    steps = ( math.fmod(angint2[0] + (adist)*x/float(nsteps-1), 2*math.pi) for x in xrange(nsteps) )
    return ( (radius*math.cos( theta ) + center[0], radius*math.sin( theta ) + center[1]) for theta in steps )

def smooth_corner(pm, pi, pp, radius=None):
    vb = pm - pi
    lb = scipy.linalg.norm(vb)
    nb = vb/lb

    vf = pp - pi
    lf = scipy.linalg.norm(vf)
    nf = vf/lf

    o = orientation(nb, nf)
    if o == 0.0:
        yield pm
        yield pi
        return

    if not radius:
        lf_low = lf < lb
        if lf_low:
            alpha = lf
        else:
            alpha = lb
        radius = alpha*tan_theta(nb, nf)
    else:
        alpha = radius*cot_theta(nb, nf)
        if(alpha > lb or alpha > lf):
            print "Warning: radius is too large!"
        lf_low = True

    tb = nb*alpha + pi
    tf = nf*alpha + pi

    rb = o*radius*rot_pi(nb)
    rf = o*radius*rot_n_pi(nf)

    center = tb + rb

    angle_b = math.atan2(rb[1], rb[0]) + math.pi
    angle_f = math.atan2(rf[1], rf[0]) + math.pi

    for i in circle(center, radius, (angle_b, angle_f), o < 0.0, 5):
        yield i

if __name__ == '__main__':
    pts = numpy.array([[0.1, 4.0], [3.9, 4.2], [4.0, 0.0],[0.0, 0.0], [0.0, 2.0], [-2.0, 2.0], [-2.0, -2.0], [0.0, -2.0], [6.0, -2.0], [6.0, 0.0]])

    out_pts = [pts[0]]
    for ct in xrange(1, len(pts)-1):
        out_pts += list(smooth_corner(pts[ct-1], pts[ct], pts[ct+1], 0.5))
    out_pts.append(pts[-1])

    pylab.clf()

    ax = pylab.axes([0,0,1,1], frame_on=False, xticks=[], yticks=[])
    ax.set_axis_off()

    for ct in xrange(len(pts)-1):
        linedata = zip(pts[ct], pts[ct+1])
        ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='black'))

    for ct in xrange(len(out_pts)-1):
        linedata = zip(out_pts[ct], out_pts[ct+1])
        ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue'))

    ax.axis('equal')
    oldax = ax.axis()
    xdim = (oldax[1]-oldax[0])
    ydim = (oldax[3]-oldax[2])
    ax.axis([oldax[0]-xdim*0.10, oldax[1]+xdim*0.10,
             oldax[2]+ydim*0.10, oldax[3]+ydim*0.10])
    ax.set_aspect('equal', 'box')
    pylab.show()

