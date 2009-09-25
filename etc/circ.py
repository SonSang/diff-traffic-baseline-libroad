#!/usr/bin/python

import numpy
import math
import pylab
import matplotlib
import scipy.linalg

def min_arc(a0, a1):
    a0 = (a0 + 360) % 360
    a1 = (a1 + 360) % 360
    if(a0 > a1):
        a0, a1 = a1, a0
    if(360-a1 + a0 < a1 - a0):
        a0, a1 = a1, a0
    return a0, a1

def tan_circ(plist, radius=None):
    p    = plist[1]
    back = plist[0] - plist[1]
    fwd  = plist[2] - plist[1]
    blen = scipy.linalg.norm(back)
    back /= blen
    flen = scipy.linalg.norm(fwd)
    fwd  /= flen

    det = back[1]*fwd[0] - back[0]*fwd[1]

    if(det > 0):
        back_t = numpy.array([ back[1], -back[0]])
        fwd_t  = numpy.array([ -fwd[1],  fwd[0]])
    elif det < 0:
        back_t = numpy.array([-back[1], back[0]])
        fwd_t  = numpy.array([  fwd[1], -fwd[0]])
    else:
        return (plist[2], None, None, None, None, plist[2])

    if not radius:
        radius = min(blen, flen) * (1-numpy.dot(fwd, back))/math.sqrt(1-numpy.dot(fwd, back)**2)
        alpha = min(blen, flen)
    else:
        alpha = radius*(fwd_t[0]-back_t[0])/(back[0] - fwd[0])
        # alpha = radius*(1+numpy.dot(fwd, back))/math.sqrt(1-numpy.dot(fwd, back)**2)

    # d0 = 2*radius/math.sqrt(1-numpy.dot(fwd, back))
    #print d0*math.sqrt(1+numpy.dot(fwd, back))/2

    angle0 = math.atan2(-back_t[1], -back_t[0])*180.0/math.pi
    angle1 = math.atan2( -fwd_t[1],  -fwd_t[0])*180.0/math.pi

    angle0, angle1 = min_arc(angle0, angle1)

    center0 = alpha * back + radius*back_t + p
    return (alpha * back + p, angle0, center0, radius, angle1, alpha * fwd + p)

if __name__ == '__main__':
    debug = True
    pts = numpy.array([[0.1, 4.0], [3.9, 4.2], [4.0, 0.0],[0.0, 0.0], [0.0, 2.0], [-2.0, 2.0], [-2.0, -2.0], [0.0, -2.0], [6.0, -2.0], [6.0, 0.0]])
#    pts = numpy.array([[1.0, -2.0], [2.0, 4.0], [8.0, -1.0]])
    lastpt = pts[0]
    pylab.clf()
    ax = pylab.subplot(111)
    for idx in xrange(1, len(pts)-1):
        rad = 0.5
        (p0, angle0, center, thisrad, angle1, p1) = tan_circ(pts[idx-1:idx+2], rad)

        if center == None:
            continue

        linedata = zip(lastpt, p0)
        l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=2.0 )
        ax.add_line(l0)

        if debug:
            linedata = zip(pts[idx-1], pts[idx])
            l1 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue', linewidth=1.0 )
            ax.add_line(l1)

        if debug:
            circle = matplotlib.patches.Circle(center, thisrad, ec='none', facecolor='#00ff00', fill=True)
            ax.add_patch(circle)
        circle = matplotlib.patches.Arc(center, 2*thisrad, 2*thisrad, 0.0, angle0, angle1, edgecolor='black', lw=2.0, fill=False)
        ax.add_patch(circle)

        lastpt = p1

    linedata = zip(lastpt, pts[-1])
    l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=2.0)
    ax.add_line(l0)

    if debug:
        linedata = zip(pts[-2], pts[-1])
        l1 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue', linewidth=1.0)
        ax.add_line(l1)

    ax.axis('auto')
    ax.axis('equal')

    pylab.show()
