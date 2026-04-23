"""
This code defines a Rover Dynamics+Kinematics Model, a Taylor Series linearization of the model through Jacobian computations around a reference point, 
and an exact Zero-Order Hold (ZOH) discretization.

Differential-drive rover, continuous-time and nonlinear model (2D plane, 5 states, 2 controls):
State: x = [px, py, theta, v, omega] (position, heading, linear & angular velocity)
Control: u = [TR, TL] (right & left wheel torques)

1. Dynamics:
    v_dot = c * omega**2 + (1/(R*M)) * (TR + TL)
    omega_dot = -alpha * omega * v + beta * (TR - TL)
    where alpha = (M*c) / (M*c**2 + J), beta = L / (R*(M*c**2 + J))

2. Kinematics:
    px_dot = v * cos(theta)
    py_dot = v * sin(theta)
    theta_dot = omega

3. Parameters:
    M: mass (kg)
    J: rotational inertia (kg*m^2)
    R: wheel radius (m)
    L: half-distance between wheels from both sides (m)
    c: longitudinal offset of center-of-mass from the mid-axle (m)
"""

import jax.numpy as np
from jax import jacfwd
from jax.scipy.linalg import expm

def params_dict(M=7.0, J=0.22, R=0.129, L=0.177, c=0.0): 
    """Definition of rover parameters with default values."""
    return dict(M=float(M), J=float(J), R=float(R), L=float(L), c=float(c)) #default values from Leo rover specs

def alpha_beta(p):
    """Definition of terms to be used in dynamics equations."""
    M, J, R, L, c = p['M'], p['J'], p['R'], p['L'], p['c']
    den = (M * c**2 + J)
    alpha = (M * c) / den
    beta  =  L / (R * den)
    return alpha, beta

def f_continuous(x, u, p):
    """Continuous-time nonlinear equations of motion."""
    x = np.asarray(x, dtype=float).reshape(-1)
    u = np.asarray(u, dtype=float).reshape(-1)
    M, R, c = p['M'], p['R'], p['c']
    alpha, beta = alpha_beta(p)
    _, _, th, v, om = x
    TR, TL = u
    xdot = np.zeros(5, dtype=float)
    xdot = xdot.at[0].set(v*np.cos(th)) #px_dot
    xdot = xdot.at[1].set(v*np.sin(th)) #py_dot
    xdot = xdot.at[2].set(om) #theta_dot
    xdot = xdot.at[3].set(c*om**2 + (TR + TL)/(R*M)) #v_dot
    xdot = xdot.at[4].set(-alpha*om*v + beta*(TR - TL)) #omega_dot
    return xdot

def TS_linearization(p, xref, uref):
    """Taylor Series linearization of f_continuous around reference point (xref, uref)."""
    xref = np.asarray(xref, dtype=float).reshape(-1)
    uref = np.asarray(uref, dtype=float).reshape(-1)
    A = jacfwd(f_continuous, argnums=0)(xref, uref, p) #df/dx at (xref,uref) (5x5)
    B = jacfwd(f_continuous, argnums=1)(xref, uref, p) #df/du at (xref,uref) (5x2)
    c = f_continuous(xref, uref, p) - A @ xref - B @ uref #error term
    return A, B, c
 
def ZOH_discretization(A, B, c, dt):
    """
    Zero-Order Hold discretization of linearized system:
    Ad = exp(A*dt) || Bd = integral_0^dt exp(A*tau) dtau * B || cd = integral_0^dt exp(A*tau) dtau * c
    """
    n = A.shape[0]
    integral_eAt = np.zeros((n, n))
    # Series -> integral(exp(A*dt)*dt) = I*dt + A*dt²/2 + A²*dt³/6 + A³*dt⁴/24:
    num_terms = 4 #number of terms in series expansion
    factorial_denoms = []
    factorial = 1
    for k in range(num_terms):
        factorial *= (k + 1) #(k+1)!
        factorial_denoms.append(float(factorial))
    A_power = np.eye(n)
    for k in range(num_terms):
        integral_eAt += A_power * (dt**(k+1)) / factorial_denoms[k] #A^k * dt^(k+1) / (k+1)!
        A_power = A_power @ A #A^(k+1)
    Ad = expm(A * dt)
    Bd = integral_eAt @ B #B and c inside integral usually, but in this case, they're constant w.r.t tau
    cd = integral_eAt @ c
    return Ad, Bd, cd

def linearize_and_discretize(xref, uref, dt, p):
    """TS linearization and ZOH discretization of system around (xref, uref) corresponding to one point along a trajectory."""
    A, B, c = TS_linearization(p, xref, uref)
    Ad, Bd, cd = ZOH_discretization(A, B, c, dt)
    return Ad, Bd, cd
