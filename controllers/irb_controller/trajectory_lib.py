import numpy as np

class TrajectoryGenerator:
    def __init__(self):
        pass


    def cubic_trajectory(self, t, t0, tf, q0, qf):
        
        v0 = np.zeros_like(q0)
        vf = np.zeros_like(qf)
        
        T = tf - t0
        h = qf - q0
        
        a0 = q0
        a1 = v0
        a2 = (3 * h - (2 * v0 + vf) * T) / (T**2)
        a3 = (-2 * h + (v0 + vf) * T) / (T**3)
        
        dt = t - t0
        qt = a0 + a1 * dt + a2 * dt**2 + a3 * dt**3
        return qt


    def quintic_trajectory(self, t, t0, tf, q0, qf):
        v0 = np.zeros_like(q0)
        vf = np.zeros_like(qf)
        ac0 = np.zeros_like(q0)
        acf = np.zeros_like(qf)
        
        T = tf - t0
        h = qf - q0
        
        a0 = q0
        a1 = v0
        a2 = ac0 / 2.0
        
        a3 = (20*h - (8*vf + 12*v0)*T - (3*ac0 - acf)*T**2) / (2 * T**3)
        a4 = (-30*h + (14*vf + 16*v0)*T + (3*ac0 - 2*acf)*T**2) / (2 * T**4)
        a5 = (12*h - 6*(vf + v0)*T + (acf - ac0)*T**2) / (2 * T**5)
        
        dt = t - t0
        qt = a0 + a1*dt + a2*dt**2 + a3*dt**3 + a4*dt**4 + a5*dt**5
        return qt


    def trapezoidal_trajectory(self, t, t0, tf, q0, qf):
        T = tf - t0
        dt = t - t0
        qt = np.zeros_like(q0)
        
   
        for i in range(len(q0)):
            dist = qf[i] - q0[i]
            sign = np.sign(dist)
            L = abs(dist)
            
            if L < 1e-6: 
                qt[i] = q0[i]
                continue

           
            tb = T * 0.2 
            a_needed = L / (T * tb - tb**2) 
            v_cruise = a_needed * tb
            
            if dt < 0:
                qt[i] = q0[i]
            elif dt > T:
                qt[i] = qf[i]
            elif dt < tb:
                
                qt[i] = q0[i] + sign * 0.5 * a_needed * dt**2
            elif dt < (T - tb):
    
                qt[i] = q0[i] + sign * (0.5 * a_needed * tb**2 + v_cruise * (dt - tb))
            else:
                
                time_left = T - dt
                qt[i] = qf[i] - sign * 0.5 * a_needed * time_left**2
                
        return qt