from abc import abstractmethod
import numpy as np
from scipy.integrate import solve_ivp, _ivp

class ComplexWaypoint:
    def __init__(self, vec_waypoint_local: np.ndarray, vec_roi_local: np.ndarray):
        self.vec_waypoint_local = vec_waypoint_local
        self.vec_roi_local = vec_roi_local

def simulateUasPhysics(tSpan: tuple[float,float], vec_uasStartState: np.ndarray, complexWaypoint: ComplexWaypoint) -> _ivp.ivp.OdeResult:
    # return solve_ivp(lambda t, vec_uasState: simulateUasPhysicsInner(t, vec_uasState, complexWaypointGenerator), tSpan, vec_uasStartState, atol=1e-6, rtol=1e-6)
    return solve_ivp(lambda t, vec_uasState: simulateUasPhysicsInner(t, vec_uasState, complexWaypoint), tSpan, vec_uasStartState)

def simulateUasPhysicsInner(t, vec_uasState, complexWaypoint: ComplexWaypoint) -> np.ndarray:
    vec_uasPos_local = vec_uasState[0:3]
    phi = vec_uasState[3]
    theta = vec_uasState[4]
    psi = vec_uasState[5]
    vec_uvw_uas = vec_uasState[6:9]
    u = vec_uasState[6]
    v = vec_uasState[7]
    w = vec_uasState[8]
    vec_pqr_uas = vec_uasState[9:12]
    p = vec_uasState[9]
    q = vec_uasState[10]
    r = vec_uasState[11]
    phi = vec_uasState[3]
    theta = vec_uasState[4]
    psi = vec_uasState[5]
    
    sinphi = np.sin(phi)
    cosphi = np.cos(phi)
    sintheta = np.sin(theta)
    costheta = np.cos(theta)
    sinpsi = np.sin(psi)
    cospsi = np.cos(psi)
    
    dcm_uas2local = np.array([
        [costheta*cospsi,sinphi*sintheta*cospsi-cosphi*sinpsi,cosphi*sintheta*cospsi+sinphi*sinpsi],
        [costheta*sinpsi,sinphi*sintheta*sinpsi+cosphi*cospsi,cosphi*sintheta*sinpsi-sinphi*cospsi],
        [-sintheta      ,sinphi*costheta                     ,cosphi*costheta                     ]
    ])
    
    # Determine control forces and moments
    dcm_local2uas = np.transpose(dcm_uas2local)
    
    vec_heading_uas = np.array([1,0,0])
    vec_heading_local = dcm_uas2local@vec_heading_uas
    
    vec_uas2roi_local = complexWaypoint.vec_roi_local - vec_uasPos_local
    
    vec_uasRight_uas = np.array([0,1,0])
    vec_uasRight_local = dcm_uas2local@vec_uasRight_uas
    
    pitchError = np.arcsin(vec_heading_local[2])
    yawError = np.arctan2(vec_heading_local[1]*vec_uas2roi_local[0]-vec_heading_local[0]*vec_uas2roi_local[1],vec_heading_local[0]*vec_uas2roi_local[0]+vec_heading_local[1]*vec_uas2roi_local[1])
    rollError = -np.arcsin(vec_uasRight_local[2])
    
    vec_uas2waypoint_local = complexWaypoint.vec_waypoint_local - vec_uasPos_local
    vec_uas2waypoint_uas = dcm_local2uas@vec_uas2waypoint_local
    
    m = 5.3063
    LMNmax = 22.63 * 0.3475
    Zmax = 73.54
    
    Lc = min(max(2000*rollError-250*v-500*p+140*min(max(vec_uas2waypoint_uas[1], -5), 5), -LMNmax), LMNmax)
    Mc = min(max(2000*pitchError+250*u-500*q-140*min(max(vec_uas2waypoint_uas[0], -5), 5), -LMNmax), LMNmax)
    Nc = min(max(-1200*yawError-400*r, -LMNmax), LMNmax)
    Zc = min(max(-9.81*m/np.cos(pitchError)/np.cos(rollError)+2*5*min(max(vec_uas2waypoint_local[2], -0.5), 0.5)-4*min(max(w, -1), 1), -Zmax), Zmax)
    
    # Determine rates of change using physics
    vec_dUasState = np.empty((12,))
    trix_rotDyn = np.array([
        [1,sinphi*np.tan(theta),cosphi*np.tan(theta)],
        [0,cosphi              ,-sinphi             ],
        [0,sinphi/costheta,cosphi/costheta]
    ])
    vec_dUasState[0:3] = dcm_uas2local@vec_uvw_uas
    vec_dUasState[3:6] = trix_rotDyn@vec_pqr_uas
    
    # TODO - Aerodynamic forces
    Ix = 5.3063 * 0.3475**2 / 4
    Iy = 5.3063 * 0.3475**2 / 4
    Iz = 5.3063 * 0.3475**2 / 2
    vec_dUasState[6] = vec_uvw_uas[1]*vec_pqr_uas[2]-vec_uvw_uas[2]*vec_pqr_uas[1] - 9.81*sintheta
    vec_dUasState[7] = vec_uvw_uas[2]*vec_pqr_uas[0]-vec_uvw_uas[0]*vec_pqr_uas[2] + 9.81*costheta*sinphi
    vec_dUasState[8] = vec_uvw_uas[0]*vec_pqr_uas[1]-vec_uvw_uas[1]*vec_pqr_uas[0] + 9.81*costheta*cosphi + Zc/m
    
    # TODO - Aerodynamic moments
    vec_dUasState[9:12] = np.array([
        (Iy-Iz)/Ix*q*r + Lc/Ix,
        (Iz-Ix)/Iy*p*r + Mc/Iy,
        (Ix-Iy)/Iz*p*q + Nc/Iz
    ])
    return vec_dUasState