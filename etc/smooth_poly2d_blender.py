#!/usr/bin/python

import numpy
import Blender
import math
import scipy.linalg
import itertools as it

def tan_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt((1 - dot) / (1 + dot))

def cot_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt((1 + dot) / (1 - dot))

def orientation(nb, nf):
    return nb[0]*nf[1] - nf[0]*nb[1]

def rot_pi(vec):
    return numpy.array([-vec[1], vec[0], vec[2]])

def rot_n_pi(vec):
    return numpy.array([vec[1], -vec[0], vec[2]])

def circle_len(center, radius, angint, ccw, seg_len):
    angint2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in angint ]
    inorder = angint2[0] <= angint2[1]
    adist = abs(angint2[1] - angint2[0])
    if (not ccw and inorder) or (ccw and not inorder):
        adist = 2*math.pi - adist
    if not ccw:
        adist *= -1
    nsteps = int(math.ceil(radius*abs(adist)/seg_len))
    if nsteps < 2:
        nsteps = 2
    steps = ( math.fmod(angint2[0] + (adist)*x/float(nsteps-1), 2*math.pi) for x in xrange(nsteps) )
    return ( numpy.array([radius*math.cos( theta ) + center[0], radius*math.sin( theta ) + center[1], 0.0]) for theta in steps )

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
    else:
        o = math.copysign(1.0, o)

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

    rb = o*radius*rot_pi(nb)
    rf = o*radius*rot_n_pi(nf)

    center = tb + rb + pi

    angle_b = math.atan2(rb[1], rb[0]) + math.pi
    angle_f = math.atan2(rf[1], rf[0]) + math.pi

    return  [center, radius, (angle_b, angle_f), o < 0.0]

def poly_to_circ(p, radius=None):
    return it.ifilter(lambda x: x, (smooth_corner(p[ct-1], p[ct], p[ct+1], radius) for ct in xrange(1, len(p)-1)))

def smoothed_points(c, res, offs=0):
    for (center, rad, (angle_b, angle_f), d) in c:
        if d:
            newrad = rad-offs
        else:
            newrad = rad+offs
        for i in circle_len(center, newrad, (angle_b, angle_f), d, res):
            yield i

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
    n0 = pts[1]-pts[0]
    n0 /= scipy.linalg.norm(n0)
    n0 = rot_pi(n0)

    nend = pts[-1]-pts[-2]
    nend /= scipy.linalg.norm(nend)
    nend = rot_pi(nend)

    low_side  = it.chain( [pts[0] + offs_range[0]*n0], smoothed_points(c, res, offs_range[0]), [pts[-1] + offs_range[0]*nend])
    high_side = it.chain( [pts[0] + offs_range[1]*n0], smoothed_points(c, res, offs_range[1]), [pts[-1] + offs_range[1]*nend])

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
    me.verts.extend( [ [v[0], v[1], 0.0] for v in vrts ] )
    me.faces.extend([ [f[i] for i in xrange(3)] for f in faces])
    return me

def smooth_blender_curve(polyline_obj, name, radius, res, offs_range):
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

    scn.objects.new(smooth_blender_curve(scn.objects.selected[0], "test", 0.8, 0.05, (-0.5, 0.5)))
