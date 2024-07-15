

class actuator_simulator:
    def __init__(self,z,wn):
        self.delta = 0
        self.ddelta = 0
        self.z  = z #damping ratio
        self.wn = wn #natural frequency
    def update(self,delta_des,dt):
        #compute state derivatives
        dddelta = -2*self.z*self.wn*self.ddelta + (delta_des-self.delta)*self.wn*self.wn
        ddelta = self.ddelta
        #euler integration
        self.ddelta += dddelta*dt
        self.delta += ddelta*dt
