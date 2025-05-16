import scipy.interpolate
import scipy.integrate
import pandas as pd
import numpy as np
import math

def elaborate_output(x, y, phi, vx):
  """
  This function resamples the path of the global planner:
  - adds the curvature and s parameters
  - rotates the phi by 90 degrees

  Args:
    x (list): x coordinates of the path
    y (list): y coordinates of the path
    phi (list): angles of the path
    vx (list): velocities of the path

  Returns:
    dict: a dictionary containing the elaborated path with keys:
      - x: x coordinates of the path
      - y: y coordinates of the path
      - phi: angles of the path
      - r: radius of curvature of the path
      - vx: velocities of the path
      - s: arc length of the path
  """

  df = pd.DataFrame({
    "x": x,
    "y": y,
    "phi": phi,
    "vx": vx
  })
  tck, u = scipy.interpolate.splprep(df[['x', 'y']].values.T, per=1, s=0.0)

  def dlength(s, tck):
    # The derivative of the length is just the 2-norm of the derivative
    dx, dy = scipy.interpolate.splev(s, tck, 1)
    return np.sqrt(np.square(dx) + np.square(dy))

  def curvature(dxs, ddxs):
    return (dxs[0,:] * ddxs[1,:] - dxs[1,:] * ddxs[0,:]) / (dxs[0,:]**2 + dxs[1,:]**2) ** 1.5

  RESOLUTION = 0.1

  len_meters = scipy.integrate.quad(dlength, 0, 1, args=(tck,))[0]
  N = math.ceil(len_meters / RESOLUTION)

  spline_ss = np.linspace(0, 1, N)
  xs = np.vstack(scipy.interpolate.splev(spline_ss, tck, 0))
  print(xs[:,0])
  dxs = np.vstack(scipy.interpolate.splev(spline_ss, tck, 1))
  ddxs = np.vstack(scipy.interpolate.splev(spline_ss, tck, 2))
  ks = curvature(dxs, ddxs)
  ss = spline_ss * len_meters
  ts = dxs / np.linalg.norm(dxs, axis=0)

  # phi calcolata
  phis = np.arctan2(ts[1,:], ts[0,:])

  # phi dall'output ruotata
  #steering_angle = df['phi'].values
  #phi_output = (steering_angle + 1.57 + np.pi) % (2*np.pi) - np.pi

  return {
    "x": xs[0,:],
    "y": xs[1,:],
    "phi": phis,
    "r": 1/ks,
    "vx": vx,
    "s": spline_ss
  }

