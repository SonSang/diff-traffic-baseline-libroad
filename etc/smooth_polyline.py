import numpy
import scipy
import scipy.linalg
import math
import cvxopt
import cvxopt.solvers

import pylab

def cot_theta(nb, nf):
    dot = numpy.dot(nb, nf)
    return math.sqrt((1 + dot ) /(1 - dot))

def orientation(nb, nf):
    return nb[0]*nf[1] - nf[0]*nb[1]

def rot_pi(vec):
    return numpy.array([-vec[1], vec[0]])

def rot_n_pi(vec):
    return numpy.array([vec[1], -vec[0]])

def circlelen(center, radius, angint, ccw, seg_len):
    angint2 = numpy.fmod(angint + 2*math.pi, 2*math.pi)
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
    res = numpy.zeros( (nsteps, len(center) ))
    for i in xrange(nsteps):
        theta = math.fmod(angint2[0] + (adist)*i/float(nsteps-1), 2*math.pi)
        res[i] = radius*numpy.array([math.cos( theta ), math.sin( theta )]) + center
    return res

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
    __slots__ = ["p_start", "p_end", "centers", "radii", "arcs", "orient", "N"]
    def __init__(self, pline, radii=None):
        self.N = pline.N-2
        self.p_start = pline.points[0]
        self.p_end   = pline.points[pline.N-1]

        tan_thetas   = numpy.zeros(self.N)
        for i in xrange(self.N):
            tan_thetas[i] = cot_theta(pline.vectors[i], pline.vectors[i+1])
        if radii == None:
            self.radii = self.calc_radii(pline, tan_thetas).T
        else:
            self.radii = radii

        self.centers = numpy.zeros((self.N, pline.points.shape[1]))
        self.arcs    = numpy.zeros((self.N, 2))
        self.orient  = numpy.zeros((self.N,))

        for i in xrange(self.N):
            alpha  = self.radii[i]/tan_thetas[i]
            self.orient[i] = math.copysign(1.0, -orientation(pline.vectors[i], pline.vectors[i+1]))
            rb = self.orient[i]*self.radii[i]*rot_pi(-pline.vectors[i])
            rf = self.orient[i]*self.radii[i]*rot_n_pi(pline.vectors[i+1])
            self.centers[i] = alpha*pline.vectors[i+1] + rf + pline.points[i+1]
            self.arcs[i][0] = math.atan2(rb[1], rb[0]) + math.pi
            self.arcs[i][1] = math.atan2(rf[1], rf[0]) + math.pi
    def calc_radii(self, pline, tan_thetas):
        b = cvxopt.matrix(0.0, ( 2*(self.N-1) + 3, 1 ))
        b[0:self.N-1] = pline.lengths[1:self.N]
        b[-2] = pline.lengths[0]
        b[-1] = pline.lengths[self.N]

        # find which radius is the biggest minimum
        best_pick   = None
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
        b[self.N-1 + pick] = -best_alpha*tan_thetas[pick]
        c = cvxopt.matrix(0.0, (self.N, 1))
        # new objective function
        for i in xrange(self.N):
            c[i] = -tan_thetas[i]
        sol = cvxopt.solvers.lp(c, A, b)
        return tan_thetas * numpy.array(sol['x']).T
    def make_lp_A(self, tan_thetas, pick):
        A = cvxopt.matrix(0.0, (2*(self.N-1) + 3, self.N))
        # constraints [0..N-1)
        for ct in xrange(0, self.N-1):
            A[ct, ct  ] = 1.0
            A[ct, ct+1] = 1.0

        # constraints [N-1, 2*(N-1))
        cno = self.N-1
        for ct in xrange(0, self.N):
            if ct != pick:
                A[cno, pick] =  tan_thetas[pick]
            A[cno, ct]   = -tan_thetas[ct]
            cno += 1

        # end constraints
        A[2 * (self.N - 1) + 1,            0] = 1.0
        A[2 * (self.N - 1) + 2, self.N-1] = 1.0
        return A
    def extract_line(self, resolution):
        res = numpy.zeros((1, self.p_start.shape[0]))
        res[0] = self.p_start
        for i in xrange(self.N):
            center = self.centers[i]
            radius = self.radii[i]
            angle  = self.arcs[i]
            orient = self.orient[i]
            circ = circlelen(center, radius, angle, orient < 0.0, resolution)
            s0 = res.shape[0]
            res.resize((s0 + circ.shape[0], res.shape[1]))
            res[s0:] = circ
        res.resize((res.shape[0] + 1, res.shape[1]))
        res[-1] = self.p_end
        return res

if __name__ == '__main__':
    cvxopt.solvers.options['show_progress'] = False
    p = polyline([[0.0, 4.0], [4.0, 3.0], [4.0, 0.0], [6.0, 0.0], [3.0, -2.0], [2.0, -1.0], [2.0, -4.0]])
    ps = smooth_polyline(p)
    li = ps.extract_line(0.1)
    pylab.clf()
    pylab.plot(li[:,0], li[:, 1])

    pylab.gca().axis('equal')
    pylab.show()
