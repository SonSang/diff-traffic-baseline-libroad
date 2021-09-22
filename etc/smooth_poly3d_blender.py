#!/usr/bin/python

import Blender
import numpy
import os,traceback,string,sys as Psys

Psys.path.append("/home/sewall/unc/libroad/etc/")
import smooth_polyline as sp

def blender_mesh(vrts, faces, name):
    me = Blender.Mesh.New(name)
    me.verts.extend( vrts )
    me.faces.extend([ [f[i] for i in xrange(3)] for f in faces])
    return me

def smooth_blender_mesh(polyline_obj, name, res, offs_range):
    if polyline_obj.getType() != "Curve":
        raise Exception("type of object is %s, not Curve!" % polyline_obj.getType())
    if len(polyline_obj.getData()) != 1:
        raise Exception("Expected just 1 subcurve, not %d!" % len(polyline_obj.getType()))
    cu = polyline_obj.getData()[0]
    if cu.type != 0:
        raise Exception("type of object is %d, not poly (0)!" % cu.type)

    p = sp.polyline([ numpy.array(pt[0:3]) for pt in cu ])
    ps = sp.smooth_polyline(p)
    (v, f) = sp.make_mesh((x for x in ps.extract_line(offs_range[0], res)),
                          (x for x in ps.extract_line(offs_range[1], res)))
    return blender_mesh(v, f, "%s_mesh" % name)

if __name__ == '__main__':
    scn = Blender.Scene.GetCurrent()
    if len(scn.objects.selected) != 1:
        raise Exception("Should have just 1 object selected!")

    scn.objects.new(smooth_blender_mesh(scn.objects.selected[0], "test", 0.55, (-0.1, 0.1)))

