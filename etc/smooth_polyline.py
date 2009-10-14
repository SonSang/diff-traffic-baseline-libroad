import numpy
import scipy
import scipy.linalg
import math
import cvxopt
import cvxopt.solvers

def cot_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt((1 + dot ) /(1 - dot))

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

def circle_frame(theta, matrix, radius):
    s = math.sin(theta)
    c = math.cos(theta)
    numpy.dot(matrix, numpy.array([radius*c, radius*s, 0.0, 1.0]))
    return (numpy.dot(matrix, numpy.array([radius*c, radius*s, 0.0, 1.0]))[:3],
            numpy.dot(matrix[0:3,0:3], numpy.array([-s, c, 0.0])))

class polyline(object):
    __slots__ = ["points", "N", "vectors", "lengths"]
    def __init__(self, pts):
        self.points = numpy.array(pts)
        self.N = len(self.points)
        self.vectors = self.points[1:] - self.points[:-1]
        self.lengths = numpy.zeros(self.N-1)
        for i in xrange(self.N-1):
            self.lengths[i] = scipy.linalg.norm(self.vectors[i])
            self.vectors[i] /= self.lengths[i]

class smooth_polyline(object):
    __slots__ = ["p_start", "tan_start", "p_end", "tan_end", "frames", "radii", "arc", "N"]
    def __init__(self, pline, radii=None):
        self.N = pline.N-2
        self.p_start   = pline.points[0]
        self.tan_start = pline.vectors[0]
        self.p_end     = pline.points[pline.N-1]
        self.tan_end   = pline.vectors[pline.N-2]

        tan_thetas   = numpy.zeros(self.N)
        for i in xrange(self.N):
            tan_thetas[i] = cot_theta(pline.vectors[i], pline.vectors[i+1])
        if radii == None:
            self.radii = self.calc_radii(pline, tan_thetas).T[:,0]
        else:
            self.radii = radii

        self.frames = numpy.zeros((self.N, 4, 4))
        self.arc    = numpy.zeros((self.N,))

        for i in xrange(self.N):
            alpha  = self.radii[i]/tan_thetas[i]

            self.frames[i][:3,2] = -cross(pline.vectors[i], pline.vectors[i+1])
            laxis = scipy.linalg.norm(self.frames[i][:,2])
            assert abs(laxis) > 1e-6
            self.frames[i][:3,2] /= laxis

            rot_pi2  = axis_angle_matrix( math.pi*0.5, self.frames[i][:3,2])
            rot_npi2 = axis_angle_matrix(-math.pi*0.5, self.frames[i][:3,2])

            rm = numpy.dot(rot_pi2,  -pline.vectors[i])
            rp = numpy.dot(rot_npi2,  pline.vectors[i+1])

            self.frames[i][:3,0] = -rm
            self.frames[i][:3,1] = cross(self.frames[i][:3,2], rm)

            tf = alpha*pline.vectors[i+1]
            self.frames[i][:3,3] = tf + self.radii[i]*rp + pline.points[i+1]
            self.frames[i][3,3]  = 1.0

            self.arc[i] = math.pi - math.acos(numpy.dot(-pline.vectors[i], pline.vectors[i+1]))

    def calc_radii(self, pline, tan_thetas):
        b = cvxopt.matrix(0.0, ( 2*(self.N-1) + 3, 1 ))
        b[0:self.N+1] = pline.lengths

        # find which radius is the biggest minimum
        best_pick  = None
        best_alpha = None
        for pick in xrange(0, self.N):
            A = self.make_lp_A(tan_thetas, pick)

            c = cvxopt.matrix(0.0, (self.N, 1))
            # objective function
            c[pick, 0] = -tan_thetas[pick]

            sol = cvxopt.solvers.lp(c, A, b)
            if not best_alpha or best_alpha < sol['x'][pick]:
                best_pick = pick
                best_alpha = sol['x'][pick]

        # now optimize the others given the smallest
        pick = best_pick
        A = self.make_lp_A(tan_thetas, pick)
        b[self.N+1 + pick] = -best_alpha*tan_thetas[pick]
        c = cvxopt.matrix(0.0, (self.N, 1))
        # new objective function
        for i in xrange(self.N):
            c[i] = -tan_thetas[i]
        sol = cvxopt.solvers.lp(c, A, b)
        return tan_thetas * numpy.array(sol['x']).T
    def make_lp_A(self, tan_thetas, pick):
        A = cvxopt.matrix(0.0, (self.N+1 + self.N, self.N))
        # constraints [0..N+1)
        A[0, 0] = 1.0
        for ct in xrange(1, self.N):
            A[ct, ct-1] = 1.0
            A[ct, ct  ] = 1.0
        A[self.N, self.N-1] = 1.0

        # constraints [N+1, N+1 + N)
        cno = self.N+1
        for ct in xrange(0, self.N):
            if ct != pick:
                A[cno, pick] =  tan_thetas[pick]
            A[cno, ct]   = -tan_thetas[ct]
            cno += 1

        return A
    def extract_line(self, offset, resolution, up=numpy.array([0.0, 0.0, 1.0])):
        res = numpy.zeros((1, self.p_start.shape[0]))
        left0 = cross(up, self.tan_start)
        l0len = scipy.linalg.norm(left0)
        left0 /= l0len
        res[0] = self.p_start + offset*left0
        for i in xrange(self.N):
            frame  = self.frames[i]
            radius = self.radii[i]
            angle  = self.arc[i]

            (pos, tan) = circle_frame(0, frame, radius)
            left = cross(up, tan)
            llen = scipy.linalg.norm(left)
            left /= llen

            point0 = pos + left*offset
            dist = scipy.linalg.norm(point0 - res[-1])
            if dist < 1e-3:
                point0 = res[-1]
                new_start = 1
            else:
                new_start = 0

            (pos, tan) = circle_frame(angle, frame, radius)
            left = cross(up, tan)
            llen = scipy.linalg.norm(left)
            left /= llen

            pointend = pos + left*offset
            new_points = [(0, point0), (angle,pointend)]
            check = 0
            while check < len(new_points)-1:
                dist = scipy.linalg.norm(new_points[check+1][1]-new_points[check][1])
                if dist > resolution:
                    new_theta = (new_points[check+1][0]+new_points[check][0])*0.5
                    (pos, tan) = circle_frame(new_theta, frame, radius)
                    left = cross(up, tan)
                    llen = scipy.linalg.norm(left)
                    left /= llen
                    new_points = new_points[:check+1] + [(new_theta, pos + left*offset)] + new_points[check+1:]
                else:
                    check += 1

            start = res.shape[0]
            res = numpy.resize(res, (start + len(new_points)-new_start, res.shape[1]))
            for (ct, p) in enumerate(new_points[new_start:]):
                res[start+ct] = p[1]

        leftend = cross(up, self.tan_end)
        lendlen = scipy.linalg.norm(leftend)
        leftend /= lendlen
        pointend = self.p_end + offset*leftend

        dist = scipy.linalg.norm(pointend - res[-1])
        if dist > 1e-3:
            res = numpy.resize(res, (res.shape[0] + 1, res.shape[1]))
            res[-1] = pointend

        return res

def make_mesh(low_side, high_side):
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


if __name__ == '__main__':
    import pylab
    import matplotlib

    cvxopt.solvers.options['show_progress'] = False
    p = polyline([[-4.0, 0.0, 0.0], [-4.0, 4.0, 0.5], [4.0, 4.0, 1.5], [4.0, -4.0, 2.5], [-4.0, -4.0, 3.5], [-4.0, 0.0, 4.0]])

    ps = smooth_polyline(p)

    pylab.clf()

    li = p.points
    pylab.plot(li[:,0], li[:, 1])

    high = ps.extract_line(0.4, 0.5)
    low = ps.extract_line(0.5, 0.5)

    ax = pylab.gca()

    (vrts, faces) = make_mesh((x for x in low), (x for x in high))
    for ct in xrange(len(faces)):
        for fno in xrange(3):
            linedata = zip(vrts[faces[ct][fno]], vrts[faces[ct][(fno+1)%3]])
            ax.add_line(matplotlib.lines.Line2D(linedata[0], linedata[1], color='red'))

    pylab.gca().axis('equal')
    pylab.show()
