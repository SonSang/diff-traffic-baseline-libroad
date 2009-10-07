#!/usr/bin/python

import numpy
import Blender
import math
import scipy.linalg
import itertools as it

def tan_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt(1 - dot*dot) / ( 1 + dot )

def cot_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return ( 1 + dot ) / math.sqrt(1 - dot*dot)

def axis_angle_matrix(theta, axis):
    c = math.cos(theta)
    s = math.sin(theta)
    ux, uy, uz = axis[0], axis[1], axis[2]
    return numpy.array(
        [[ux*ux + (1.0-ux*ux)*c, ux*uy*(1.0-c) -  uz*s,  ux*uz*(1.0-c) +  uy*s],
         [ux*uy*(1.0-c) +  uz*s, uy*uy + (1.0-uy*uy)*c,  uy*uz*(1.0-c) -  ux*s],
         [ux*uz*(1.0-c) +  uy*s, uy*uz*(1.0-c) -  ux*s,  uz*uz + (1.0-uz*uz)*c]])

def cross(nb, nf):
    """[ x  y  z]
       [bx by bz]
       [fx fy fz]"""
    return numpy.array([
        nb[1]*nf[2] - nf[1]*nb[2],
        nb[2]*nf[0] - nf[2]*nb[0],
        nb[0]*nf[1] - nf[0]*nb[1]])

def smooth_corner(pm, pi, pp, radius=None):
    vb = pm - pi
    lb = scipy.linalg.norm(vb)
    nb = vb/lb

    vf = pp - pi
    lf = scipy.linalg.norm(vf)
    nf = vf/lf

    axis = cross(nb, nf)
    laxis = scipy.linalg.norm(axis)
    if abs(laxis) < 1e-6:
        return None
    naxis = axis/laxis

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

    tb = nb*alpha

    rot_pi2  = axis_angle_matrix(math.pi*0.5, naxis)
    rot_npi2 = axis_angle_matrix(-math.pi*0.5, naxis)

    rb = numpy.dot(rot_pi2,  nb)
    rf = numpy.dot(rot_npi2, nf)

    ortho = cross(naxis, rb)

    center = tb + radius*rb + pi

    theta = math.acos(numpy.dot(nb, nf))
    ang_range = math.pi - theta

    circle_frame = numpy.array(
        [[ -rb[0], ortho[0], naxis[0], center[0]],
         [ -rb[1], ortho[1], naxis[1], center[1]],
         [ -rb[2], ortho[2], naxis[2], center[2]],
         [    0.0,      0.0,      0.0,       1.]])

    return [circle_frame, radius, ang_range]

def circle_len(matrix, radius, range, seg_len):
    nsteps = int(math.ceil(radius*range/seg_len))
    if nsteps < 2:
        nsteps = 2
    steps = ( range*x/float(nsteps-1) for x in xrange(nsteps) )
    return ( numpy.dot(matrix, numpy.array([radius*math.cos( theta ), radius*math.sin( theta ), 0.0, 1.0]))[:3] for theta in steps )

def poly_to_circ(p, radius=None):
    return it.ifilter(lambda x: x, (smooth_corner(p[ct-1], p[ct], p[ct+1], radius) for ct in xrange(1, len(p)-1)))

def smoothed_points(circles, res):
    for (frame, rad, ang_range) in circles:
        for i in circle_len(frame, rad, ang_range, res):
            yield i

def blender_curve(vrts, name):
    new_cu = Blender.Curve.New()
    nurb = new_cu.appendNurb(list(vrts[0])+[1.0])
    nurb.type = 0
    for v in vrts[1:]:
        nurb.append(list(v) + [1.0])
    return new_cu

def smooth_blender_curve(polyline_obj, name, radius, res):
    if polyline_obj.getType() != "Curve":
        raise Exception("type of object is %s, not Curve!" % polyline_obj.getType())
    if len(polyline_obj.getData()) != 1:
        raise Exception("Expected just 1 subcurve, not %d!" % len(polyline_obj.getType()))
    cu = polyline_obj.getData()[0]
    if cu.type != 0:
        raise Exception("type of object is %d, not poly (0)!" % cu.type)

    in_pts = [ numpy.array(pt[0:3]) for pt in cu ]

    circ = list(poly_to_circ(in_pts, radius))

    pts = it.chain( [in_pts[0]], smoothed_points(circ, res), [in_pts[-1]])

    return blender_curve(list(pts), "%s_curve" % name)

if __name__ == '__main__':
    scn = Blender.Scene.GetCurrent()
    if len(scn.objects.selected) != 1:
        raise Exception("Should have just 1 object selected!")

    scn.objects.new(smooth_blender_curve(scn.objects.selected[0], "test", 0.8, 0.05))

