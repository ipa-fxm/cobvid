import numpy as np
from scipy.misc import comb
from scipy import integrate

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=1000):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """



    nPoints = len(points)
    xPoints, yPoints = map(np.array, zip(*points))
    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals[::-1], yvals[::-1]


if __name__ == "__main__":
    from matplotlib import pyplot as plt

    nPoints = 3
    rpoints = np.random.rand(nPoints,2)*2


    points = np.array([[0,0], [0,1], [1,1], [1.5, 0.5], [2, 0.5], [2, 0], [1, 0.5], [1, 0],
                       [1, -1], [1, -1], [1, -1], [2, -1], [3, -1], [3, -1], [3, -1], [3, 0],
                       [2.5, 0.5], [2, 1], [1.5, 1], [0.5, 1], [-0.5, 1], [-0.5, 0], [-0.5, -1], [0.5, -1], ], np.float)

    points = np.array([[0,0], [4,0], [0,2]], np.float)

    xpoints, ypoints = zip(*points)

    #xrpoints, yrpoints = zip(*rpoints)
    #xpoints = np.append(xpoints, xrpoints)
    #ypoints = np.append(ypoints, yrpoints)
    #points = zip(xpoints, ypoints)


    duration = 18.0  # [s]
    rate = 100.0  # [Hz]
    sample_time = 1.0/rate
    nTimes=duration*rate

    xvals, yvals = bezier_curve(points, nTimes=nTimes)


    fig, (ax1) = plt.subplots(nrows=1, ncols=1)
    ax1.plot(xvals, yvals, 'x')
    ax1.plot(xpoints, ypoints, "ro")
    for nr in range(len(points)):
        ax1.text(points[nr][0], points[nr][1], nr)
    plt.axis('equal')

    tdist = np.linspace(0, nTimes, nTimes)
    tvel = np.linspace(0, nTimes-1, nTimes-1)

    xdiff = np.array([0])
    ydiff = np.array([0])
    xdiff = np.append(xdiff, np.diff(xvals))
    ydiff = np.append(ydiff, np.diff(yvals))
    rel_distance = np.sqrt(xdiff**2 + ydiff**2)
    abs_distance = integrate.cumtrapz(rel_distance, tdist, initial=0)
    abs_phi = np.arctan2(xdiff, ydiff)


    rel_phi = np.diff(abs_phi)
    corridx = [(idx+1, 1 if phi < 0 else -1) for idx, phi in enumerate(rel_phi) if abs(phi) > 6]
    for k, v in corridx:
        abs_phi[k:] += np.pi*2*v
    rel_phi = np.diff(abs_phi)

    velocity = rel_distance / sample_time
    theta_velocity = rel_phi / sample_time



    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
    ax1r = ax1.twinx()
    ax1.plot(tdist, abs_distance)
    ax1r.plot(tdist, abs_phi, 'r')

    ax2r = ax2.twinx()
    ax2.plot(tdist, velocity)
    ax2r.plot(tvel, theta_velocity, 'r')



    plt.show()