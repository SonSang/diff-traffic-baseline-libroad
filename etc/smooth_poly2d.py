
#!/usr/bin/python

import numpy
import math
import pylab
import matplotlib
import scipy.linalg
import itertools as it

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

def circle_step(center, radius, angint, ccw, nsteps):
    angint2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in angint ]
    inorder = angint2[0] <= angint2[1]
    adist = abs(angint2[1] - angint2[0])
    if (not ccw and inorder) or (ccw and not inorder):
        adist = 2*math.pi - adist
    if not ccw:
        adist *= -1
    steps = ( math.fmod(angint2[0] + (adist)*x/float(nsteps-1), 2*math.pi) for x in xrange(nsteps) )
    return ( (radius*math.cos( theta ) + center[0], radius*math.sin( theta ) + center[1]) for theta in steps )

def circle_len(center, radius, angint, ccw, seg_len):
    angint2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in angint ]
    inorder = angint2[0] <= angint2[1]
    adist = abs(angint2[1] - angint2[0])
    if (not ccw and inorder) or (ccw and not inorder):
        adist = 2*math.pi - adist
    if not ccw:
        adist *= -1
    nsteps = int(math.ceil(radius*abs(adist)/seg_len))
    print nsteps
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
        return None

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

    rb = o*radius*rot_pi(nb)
    rf = o*radius*rot_n_pi(nf)

    center = tb + rb

    angle_b = math.atan2(rb[1], rb[0]) + math.pi
    angle_f = math.atan2(rf[1], rf[0]) + math.pi

    return  [center, radius, (angle_b, angle_f), o < 0.0]

def poly_to_circ(pts, radius=None):
    return it.ifilter(lambda x: x, (smooth_corner(pts[ct-1], pts[ct], pts[ct+1], radius) for ct in xrange(1, len(pts)-1)))

def smoothed_points(circles, res, offs=0):
    curr_dir = None
    for (center, rad, (angle_b, angle_f), dir) in circles:
        if dir:
            newrad = rad-offs
        else:
            newrad = rad+offs
        for i in circle_len(center, newrad, (angle_b, angle_f), dir, res):
            yield i

if __name__ == '__main__':
    pts = numpy.array([[0.1, 4.0], [3.9, 4.2], [4.0, 0.0],[0.0, 0.0], [0.0, 2.0], [-2.0, 2.0], [-2.0, -2.0], [0.0, -2.0], [6.0, -2.0], [6.0, 0.0]])

    circles = list(poly_to_circ(pts, 0.8))

    n0 = pts[1]-pts[0]
    n0 /= scipy.linalg.norm(n0)
    n0 = rot_pi(n0)

    offs = -0.4

    nend = pts[-1]-pts[-2]
    nend /= scipy.linalg.norm(nend)
    nend = rot_pi(nend)

    out_pts = [pts[0] + offs*n0] + list(smoothed_points(circles, 0.5, offs))  + [pts[-1] + offs*nend]

    offs = 0.4

    out_pts2 = [pts[0] + offs*n0] + list(smoothed_points(circles, 0.5, offs))  + [pts[-1] + offs*nend]

    pylab.clf()

    ax = pylab.axes([0,0,1,1], frame_on=False, xticks=[], yticks=[])
    ax.set_axis_off()

    for ct in xrange(len(pts)-1):
        linedata = zip(pts[ct], pts[ct+1])
        ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='black'))

    for ct in xrange(len(out_pts)-1):
        linedata = zip(out_pts[ct], out_pts[ct+1])
        ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue'))

    for ct in xrange(len(out_pts2)-1):
        linedata = zip(out_pts2[ct], out_pts2[ct+1])
        ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue'))

    ax.axis('equal')
    oldax = ax.axis()
    xdim = (oldax[1]-oldax[0])
    ydim = (oldax[3]-oldax[2])
    ax.axis([oldax[0]-xdim*0.10, oldax[1]+xdim*0.10,
             oldax[2]-ydim*0.10, oldax[3]+ydim*0.10])
    ax.set_aspect('equal', 'box')
    pylab.show()

