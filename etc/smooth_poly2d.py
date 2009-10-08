#!/usr/bin/python

import numpy
import math
import pylab
import matplotlib
import scipy.linalg
import itertools as it

import OpenGL.GL as GL
import OpenGL.GLUT as GLUT
import OpenGL.GLU as GLU
import sys

def tan_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt((1 - dot) / ( 1 + dot ))

def cot_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt((1 + dot ) /(1 - dot*dot))

def orientation(nb, nf):
    return nb[0]*nf[1] - nf[0]*nb[1]

def rot_pi(vec):
    return numpy.array([-vec[1], vec[0]])

def rot_n_pi(vec):
    return numpy.array([vec[1], -vec[0]])

def circle_step(center, radius, angint, ccw, nsteps):
    angint2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in angint ]
    inorder = angint2[0] <= angint2[1]
    adist = abs(angint2[1] - angint2[0])
    if (not ccw and inorder) or (ccw and not inorder):
        adist = 2*math.pi - adist
    if not ccw:
        adist *= -1
    steps = ( math.fmod(angint2[0] + (adist)*x/float(nsteps-1), 2*math.pi) for x in xrange(nsteps) )
    return ( numpy.array([radius*math.cos( theta ) + center[0], radius*math.sin( theta ) + center[1]]) for theta in steps )

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
    return ( numpy.array([radius*math.cos( theta ) + center[0], radius*math.sin( theta ) + center[1]]) for theta in steps )

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

def pylab_plot_mesh(p, vrts, faces):
    pylab.clf()

    ax = pylab.axes([0,0,1,1], frame_on=False, xticks=[], yticks=[])
    ax.set_axis_off()

    for ct in xrange(len(p)-1):
        linedata = zip(p[ct], p[ct+1])
        ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='black'))

    for ct in xrange(len(faces)):
        for fno in xrange(3):
            linedata = zip(vrts[faces[ct][fno]], vrts[faces[ct][(fno+1)%3]])
            ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='red'))

    ax.axis('equal')
    oldax = ax.axis()
    xdim = (oldax[1]-oldax[0])
    ydim = (oldax[3]-oldax[2])
    ax.axis([oldax[0]-xdim*0.10, oldax[1]+xdim*0.10,
             oldax[2]-ydim*0.10, oldax[3]+ydim*0.10])
    ax.set_aspect('equal', 'box')
    pylab.show()

class MeshWindow(object):
    def __init__(self, w, h, vrts, faces, fullscreen=False):
        self.width = w
        self.height = h
        self.fullscreen = fullscreen
        self.vrts = vrts
        self.faces = faces
        self.window = None
    def InitGL(self):
        """
        A general OpenGL initialization function.  Sets all of the initial parameters.
        We call this right after our OpenGL window is created.
        """
        GL.glClearColor(0.0, 0.0, 0.0, 0.0)        ## This Will Clear The Background Color To Black
        GL.glShadeModel(GL.GL_SMOOTH)                 ## Enables Smooth Color Shading

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()                        ## Reset The Projection Matrix

    def ReSizeGLScene(self, Width, Height):
        """
        The function called when our window is resized (which shouldn't happen if you enable fullscreen, below)
        """
        self.width  = Width
        self.height = Height

        GL.glViewport(0, 0, self.width, self.height)               ## Reset The Current Viewport And Perspective Transformation
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        if self.width > self.height:
            asp = float(self.width)/float(self.height)
            GLU.gluOrtho2D(-asp, asp, -1.0, 1.0)
        else:
            asp = float(self.height)/float(self.width)
            GLU.gluOrtho2D(-1.0, 1.0, -asp, asp)
        GL.glMatrixMode(GL.GL_MODELVIEW)
    def DrawGLScene(self):
        """
        The main drawing function.
        """
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)
        GL.glLoadIdentity()
        center = (0.0, 0.0)
        extent = 5.0
        GL.glScalef(2.0/extent, 2.0/extent, 1.0)
        GL.glTranslatef(-center[0], -center[1], 0.5)

        GL.glColor3f(1.0, 1.0, 1.0)
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)

        GL.glBegin(GL.GL_TRIANGLES)
        for ct in xrange(len(self.faces)):
            for j in xrange(3):
                GL.glVertex2f(self.vrts[self.faces[ct][j]][0], self.vrts[self.faces[ct][j]][1])
        GL.glEnd()

        GLUT.glutSwapBuffers()

    def keyPressed(self, *args):
        """
        The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)
        """
        ## If escape is pressed, kill everything.
        if args[0] == '\033':
            sys.exit()
    def go(self, argv):
        GLUT.glutInit(argv)
        GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA | GLUT.GLUT_DOUBLE | GLUT.GLUT_DEPTH)
        GLUT.glutInitWindowSize(self.width, self.height)
        GLUT.glutInitWindowPosition(0, 0)
        self.window = GLUT.glutCreateWindow("Smoothed Polylines")
        GLUT.glutDisplayFunc(self.DrawGLScene)

        if self.fullscreen:
            GLUT.glutFullScreen()

        GLUT.glutIdleFunc(self.DrawGLScene)
        GLUT.glutReshapeFunc(self.ReSizeGLScene)
        GLUT.glutKeyboardFunc(self.keyPressed)

        GLUT.glutMainLoop()

if __name__ == '__main__':
    pts = numpy.array([[0.1, 4.0], [3.9, 4.2], [4.0, 0.0],[0.0, 0.0], [0.0, 2.0], [-2.0, 2.0], [-2.0, -2.0],[0.0, -2.0], [6.0, -2.0], [6.0, 0.0]])

    circles = list(poly_to_circ(pts, 0.9))

    (v, f) =  smoothed_points_poly(pts, circles, 0.1, (-0.05, 0.05))

    tr = MeshWindow(640, 480, v, f)
    tr.go(sys.argv)
