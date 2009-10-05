#!/usr/bin/python

import numpy
import math
import pylab
import matplotlib
import scipy.linalg

from matplotlib import rc

def polyline_fig(ax, pts):
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_axis_off()
    rc('text', usetex=True)

    for ct in xrange(len(pts)-1):
        linedata = zip(pts[ct], pts[ct+1])
        l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=1.0 )
        ax.add_line(l0)

        ax.text(pts[ct][0]-0.1, pts[ct][1]-0.1, "$p_{%d}$" % ct, fontsize=20, verticalalignment='top')
        cshape = matplotlib.patches.Circle(pts[ct], 0.025, ec='none', facecolor='#000000', fill=True)
        ax.add_patch(cshape)

    ct = len(pts)-1
    ax.text(pts[ct][0]+0.025, pts[ct][1]+0.025, "$p_{%d}$" % ct, fontsize=20)
    cshape = matplotlib.patches.Circle(pts[ct], 0.025, ec='none', facecolor='#000000', fill=True)
    ax.add_patch(cshape)

    ax.axis('equal')
    oldax = ax.axis()
    xdim = (oldax[1]-oldax[0])
    ydim = (oldax[3]-oldax[2])
    ax.axis([oldax[0]-xdim*0.05, oldax[1]+xdim*0.05,
             oldax[2]-ydim*0.05, oldax[3]+ydim*0.05])

    return ax

def min_arc(a0, a1):
    a0 = math.fmod(a0 + 2*math.pi, 2*math.pi)
    a1 = math.fmod(a1 + 2*math.pi, 2*math.pi)
    if(a0 > a1):
        a0, a1 = a1, a0
    if(2*math.pi-a1 + a0 < a1 - a0):
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

    angle0 = math.atan2(-back_t[1], -back_t[0])
    angle1 = math.atan2( -fwd_t[1],  -fwd_t[0])

    angle0, angle1 = min_arc(angle0, angle1)

    center0 = alpha * back + radius*back_t + p
    return (alpha * back + p, angle0, center0, radius, angle1, alpha * fwd + p)

def tan_circ_fig(ax, pts, debug=False):
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    rc('text', usetex=True)

    lastpt = pts[0]
    for idx in xrange(1, len(pts)-1):
        rad = 0.25
        (p0, angle0, center, thisrad, angle1, p1) = tan_circ(pts[idx-1:idx+2], rad)

        if center == None:
            continue

        linedata = zip(lastpt, p0)
        l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=1.0 )
        ax.add_line(l0)

        if debug:
            linedata = zip(pts[idx-1], pts[idx])
            l1 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue', linewidth=2.0 )
            ax.add_line(l1)

        if debug:
            circle = matplotlib.patches.Circle(center, thisrad, ec='none', facecolor='#00ff00', fill=True)
            ax.add_patch(circle)
            linedata = zip(p0, center)
            l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=1.0 )
            ax.add_line(l0)
            ax.text((p0[0]+center[0])*0.5, (p0[1]+center[1])*0.5, "$r_{%d}$" % idx, fontsize=15, verticalalignment='top')
            ax.text(center[0], center[1], "$c_{%d}$" % idx, fontsize=15, verticalalignment='top')

        circle = matplotlib.patches.Arc(center, 2*thisrad, 2*thisrad, 0.0, angle0*180.0/math.pi, angle1*180.0/math.pi, edgecolor='black', lw=1.0, fill=False)
        ax.add_patch(circle)

        lastpt = p1

    if debug:
        linedata = zip(pts[-2], pts[-1])
        l1 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue', linewidth=2.0)
        ax.add_line(l1)

    linedata = zip(lastpt, pts[-1])
    l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=1.0)
    ax.add_line(l0)

    ax.axis('equal')
    oldax = ax.axis()
    xdim = (oldax[1]-oldax[0])
    ydim = (oldax[3]-oldax[2])
    ax.axis([oldax[0]-xdim*0.05, oldax[1]+xdim*0.05,
             oldax[2]-ydim*0.05, oldax[3]+ydim*0.05])

    return ax

def vec_fig(ax, pts):
    ax.set_frame_on(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_axis_off()
    rc('text', usetex=True)

    pim = pts[0]
    pi  = pts[1]
    pip = pts[2]

    # Compute vectors
    vb = pim-pi
    vblen = scipy.linalg.norm(vb)
    vb /= vblen

    vf = pip-pi
    vflen = scipy.linalg.norm(vf)
    vf /= vflen

    # Draw p_i
    ax.text(pi[0]-0.05,  pi[1]-0.05,  r"$p_i$",   fontsize=15, verticalalignment='top')

    # Draw vectors
    # back vector
    ax.text(pim[0]+0.05, pim[1]+0.05, r"$\mathbf{v}^b_i$", fontsize=15, verticalalignment='top')
    a0 = matplotlib.patches.FancyArrowPatch(posA=(pi[0], pi[1]), posB=(pim[0], pim[1]), arrowstyle='->', mutation_scale=30, lw=3.0, ls='dashed', ec='blue')
    ax.add_line(a0)

    # normalized back vector
    a0 = matplotlib.patches.FancyArrowPatch(posA=pi, posB=vf+pi, arrowstyle='->', mutation_scale=30, lw=1.0)
    ax.add_line(a0)
    ax.text(pi[0]+vb[0]+0.05, pi[1] + vb[1]+0.05, r"$\mathbf{v}^b_i/|\mathbf{v}^b_i|$", fontsize=15, verticalalignment='top')

    # forward vector
    ax.text(pi[0]+1.2*vf[0], pi[1]+1.2*vf[1], r"$\mathbf{v}^f_i$", fontsize=15, verticalalignment='top')
    a1 = matplotlib.patches.FancyArrowPatch(posA=(pi[0], pi[1]), posB=(pi[0]+1.2*vf[0], pi[1]+1.2*vf[1]), arrowstyle='->', mutation_scale=30, lw=3.0, ls='dashed', ec='blue')
    ax.add_line(a1)

    # normalized forward vector
    a1 = matplotlib.patches.FancyArrowPatch(posA=pi, posB=vb+pi, arrowstyle='->', mutation_scale=30, lw=1.0)
    ax.add_line(a1)
    ax.text(pi[0]+vf[0]-0.05, pi[1] + vf[1]-0.05, r"$\mathbf{v}^f_i/|\mathbf{v}^f_i|$", fontsize=15, verticalalignment='top')

    # draw bisector
    mid = vb + vf
    a0 = matplotlib.patches.FancyArrowPatch(posA=pi, posB=mid+pi, arrowstyle='->', mutation_scale=30, lw=1.0, ls='dotted')
    ax.add_line(a0)

    ax.text(pi[0]+mid[0]+0.05, pi[1]+mid[1]-0.05, r"$\mathbf{b}_i$", fontsize=15, verticalalignment='top')

    # compute angles
    anglef = math.atan2(vf[1], vf[0])
    anglem = math.atan2(mid[1], mid[0])
    angleb = math.atan2(vb[1], vb[0])

    # draw angle arcs
    circle = matplotlib.patches.Arc(pi, 0.5, 0.5, 0.0, anglem*180.0/math.pi, angleb*180.0/math.pi, edgecolor='black', lw=1.0, fill=False)
    ax.add_patch(circle)

    ax.text(pi[0]+0.015, pi[1]+0.20, r"$\theta_i$", fontsize=15, verticalalignment='top')

    circle = matplotlib.patches.Arc(pi, 0.5, 0.5, 0.0, anglef*180.0/math.pi, anglem*180.0/math.pi, edgecolor='black', lw=1.0, fill=False)
    ax.add_patch(circle)

    ax.text(pi[0]+0.15,  pi[1]+0.1,  r"$\theta_i$", fontsize=15, verticalalignment='top')

    # helpful values
    alpha = 0.49
    rad = 0.68
    offs = 0.05

    # Draw radii
    st = pi + alpha*vb
    ax.text(st[0]-0.2, st[1]-0.1, r"$\alpha_i\mathbf{v}^b_i/|\mathbf{v}^b_i|$", fontsize=15)
    l0 = matplotlib.lines.Line2D((st[0], st[0]+rad*vb[1]),
                                 (st[1], st[1]-rad*vb[0]), color='black', linewidth=1.0)
    ax.add_line(l0)

    ax.text(st[0] + 0.5*rad*vb[1],
            st[1] - 0.5*rad*vb[0]+0.08,
            r"$r_i$", fontsize=15, verticalalignment='top')
    p = matplotlib.patches.RegularPolygon((pi[0] + (alpha - offs)*vb[0] + offs*vb[1],
                                           pi[1] + (alpha - offs)*vb[1] - offs*vb[0]),
                                          4, offs*math.sqrt(2.0), math.pi/4+math.atan2(vb[1], vb[0]), fill=None)
    ax.add_patch(p)

    st = pi + alpha*vf
    ax.text(st[0]-0.1, st[1]-0.05, r"$\alpha_i\mathbf{v}^f_i/|\mathbf{v}^f_i|$", fontsize=15, verticalalignment='top')
    l0 = matplotlib.lines.Line2D((st[0], st[0]-rad*vf[1]), (st[1], st[1]+rad*vf[0]), color='black', linewidth=1.0)
    ax.add_line(l0)

    ax.text(st[0] - 0.5*rad*vf[1]+0.01,
            st[1] + 0.5*rad*vf[0],
            r"$r_i$", fontsize=15, verticalalignment='top')
    p = matplotlib.patches.RegularPolygon((pi[0] + (alpha - offs)*vf[0] - offs*vf[1],
                                           pi[1] + (alpha - offs)*vf[1] + offs*vf[0]),
                                          4, offs*math.sqrt(2.0), math.pi/4+math.atan2(vf[1], vf[0]), fill=None)
    ax.add_patch(p)

    # Draw circle center
    circle = matplotlib.patches.Circle((st[0]-rad*vf[1], st[1]+rad*vf[0]), 0.03, ec='none', facecolor='black', fill=True)
    ax.add_patch(circle)
    ax.text(st[0]-rad*vf[1]+0.04, st[1]+rad*vf[0], r"$c_i$", fontsize=15, verticalalignment='top')

    circle = matplotlib.patches.Circle(pi, 0.03, ec='none', facecolor='black', fill=True)
    ax.add_patch(circle)

    circle = matplotlib.patches.Arc((st[0]-rad*vf[1], st[1]+rad*vf[0]), 0.5, 0.5, 0.0, 90.0+angleb*180.0/math.pi, -90.0+anglef*180.0/math.pi, edgecolor='black', lw=1.0, fill=False)
    ax.add_patch(circle)

    ax.text(st[0]-rad*vf[1] - 0.205,
            st[1]+rad*vf[0] - 0.09,
            r"$\pi-\theta_i$", fontsize=15, verticalalignment='top')

    ax.text(st[0]-rad*vf[1] - 0.115,
            st[1]+rad*vf[0] - 0.17,
            r"$\pi-\theta_i$", fontsize=15, verticalalignment='top')

    ax.axis('tight')
    oldax = ax.axis()
    xdim = (oldax[1]-oldax[0])
    ydim = (oldax[3]-oldax[2])
    ax.axis([oldax[0]-xdim*0.10, oldax[1]+xdim*0.10,
             oldax[2]-ydim*0.10, oldax[3]+ydim*0.10])

    return ax

if __name__ == '__main__':
    pts = numpy.array([[0.1, 1.0], [0.4, 0.2], [3.0, 0.3],[2.0, 1.2], [3.0, 2.0]])

    # pylab.clf()
    # ax = pylab.axes([0,0,1,1])
    # ax = polyline_fig(ax, pts)
    # pylab.savefig("1.pdf")

    # pylab.clf()
    # ax = pylab.axes([0,0,1,1])
    # ax = tan_circ_fig(ax, pts)
    # pylab.savefig("2.pdf")

    # pylab.clf()
    # ax = pylab.axes([0,0,1,1])
    # ax = tan_circ_fig(ax, pts, True)
    # pylab.savefig("3.pdf")

    pylab.clf()
    ax = pylab.axes([0,0,1,1])
    ax = vec_fig(ax, pts[0:3])
    pylab.savefig("4.pdf", bbox_inches='tight', pad_inches=-1)
