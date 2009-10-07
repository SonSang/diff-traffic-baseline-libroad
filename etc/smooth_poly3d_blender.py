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
         [ux*uz*(1.0-c) -  uy*s, uy*uz*(1.0-c) +  ux*s,  uz*uz + (1.0-uz*uz)*c]])

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

    print "naxis", naxis
    print "rb", rb
    print "ortho", ortho

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
    for theta in steps:
        s = math.sin(theta)
        c = math.cos(theta)
        yield (numpy.dot(matrix, numpy.array([radius*c, radius*s, 0.0, 1.0]))[:3], numpy.dot(matrix[0:3,0:3], numpy.array([-s, c, 0.0])))

def poly_to_circ(p, radius=None):
    return it.ifilter(lambda x: x, (smooth_corner(p[ct-1], p[ct], p[ct+1], radius) for ct in xrange(1, len(p)-1)))

def smoothed_points(circles, res, offs=0.0, up=numpy.array([0.0, 0.0, 1.0])):
    for (frame, rad, ang_range) in circles:
        for (pos, tan) in circle_len(frame, rad, ang_range, res):
            left = cross(up, tan)
            llen = scipy.linalg.norm(left)
            left /= llen
            yield pos + left*offs

def triangle_angles(pt0, pt1, pt2):
    v01 = pt1 -pt0
    v01 /= scipy.linalg.norm(v01)

    v02 = pt2 -pt0
    v02 /= scipy.linalg.norm(v02)

    v12 = pt2 - pt1
    v12 /= scipy.linalg.norm(v12)

    a0 = math.acos(numpy.dot(v01, v02))
    a1 = math.acos(numpy.dot(-v01, v12))
    a2 = 2*math.pi - (a0 + a1)

    return (a0, a1, a2)

def smoothed_points_poly(pts, c, res, offs_range):
    up = numpy.array([0.0, 0.0, 1.0])
    n0 = pts[1]-pts[0]
    n0 /= scipy.linalg.norm(n0)
    left0 = cross(up, n0)
    l0len = scipy.linalg.norm(left0)
    left0 /= l0len

    nend = pts[-1]-pts[-2]
    nend /= scipy.linalg.norm(nend)
    leftend = cross(up, nend)
    leftendlen = scipy.linalg.norm(leftend)
    leftend /= leftendlen

    low_side  = it.chain( [pts[0] + offs_range[0]*left0], smoothed_points(c, res, offs_range[0], up), [pts[-1] + offs_range[0]*leftend])
    high_side = it.chain( [pts[0] + offs_range[1]*left0], smoothed_points(c, res, offs_range[1], up), [pts[-1] + offs_range[1]*leftend])

    vrts = []
    faces = []

    vrts.append(low_side.next())
    vrts.append(high_side.next())

    base_low_idx  = len(vrts)-2
    base_high_idx = len(vrts)-1

    base_low_vrt  = vrts[len(vrts)-2]
    base_high_vrt = vrts[len(vrts)-1]

    try:
        low_cand_vrt = low_side.next()
    except StopIteration:
        low_cand_vrt = None

    try:
        high_cand_vrt = high_side.next()
    except StopIteration:
        high_cand_vrt = None

    ## pick_vrt = True means high
    ## False means low
    while True:
        if low_cand_vrt == None:
            if high_cand_vrt == None:
                break
            else:
                pick_vrt = True
        elif high_cand_vrt == None:
            pick_vrt = False
        else:
            angs_low  = triangle_angles(base_low_vrt, base_high_vrt, low_cand_vrt)
            angs_high = triangle_angles(base_low_vrt, base_high_vrt, high_cand_vrt)
            lmax = max(angs_low)
            hmax = max(angs_high)
            if lmax > hmax:
                pick_vrt = False
            else:
                pick_vrt = True

        faces.append([base_low_idx, base_high_idx, len(vrts)])

        if pick_vrt:
            vrts.append(high_cand_vrt)
            base_high_vrt = high_cand_vrt
            base_high_idx = len(vrts)-1
            try:
                high_cand_vrt = high_side.next()
            except StopIteration:
                high_cand_vrt = None
        else:
            vrts.append(low_cand_vrt)
            base_low_vrt = low_cand_vrt
            base_low_idx = len(vrts)-1
            try:
                low_cand_vrt = low_side.next()
            except StopIteration:
                low_cand_vrt = None

    return (vrts, faces)

def blender_mesh(vrts, faces, name):
    me = Blender.Mesh.New(name)
    me.verts.extend( vrts )
    me.faces.extend([ [f[i] for i in xrange(3)] for f in faces])
    return me

def smooth_blender_mesh(polyline_obj, name, radius, res, offs_range):
    if polyline_obj.getType() != "Curve":
        raise Exception("type of object is %s, not Curve!" % polyline_obj.getType())
    if len(polyline_obj.getData()) != 1:
        raise Exception("Expected just 1 subcurve, not %d!" % len(polyline_obj.getType()))
    cu = polyline_obj.getData()[0]
    if cu.type != 0:
        raise Exception("type of object is %d, not poly (0)!" % cu.type)

    pts = [ numpy.array(pt[0:3]) for pt in cu ]
    c = list(poly_to_circ(pts, radius))
    (v, f) =  smoothed_points_poly(pts, c, res, offs_range)
    return blender_mesh(v, f, "%s_mesh" % name)

if __name__ == '__main__':
    scn = Blender.Scene.GetCurrent()
    if len(scn.objects.selected) != 1:
        raise Exception("Should have just 1 object selected!")

    scn.objects.new(smooth_blender_mesh(scn.objects.selected[0], "test", 0.9, 0.2, (-0.4, 0.4)))

