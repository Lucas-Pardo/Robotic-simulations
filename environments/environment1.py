import numpy as np
import cvxpy as cp

class Environment:
    def __init__(self, diameters, env_limit=20):
        self.env_limit = env_limit
        self.obstacles = []
        for diameter in diameters:
            x, y = self.generate_position(diameter)
            self.obstacles.append([diameter, x, y])

    def generate_position(self, diameter):
        min_distance = 2 + diameter / 2
        r = np.random.uniform(min_distance, min((1 + len(self.obstacles)/2) * min_distance, self.env_limit))
        angle = np.random.uniform(0, 2*np.pi)
        return r * np.cos(angle), r * np.sin(angle)
    
    def is_collision(self, x, y, diameter):
        for d, x_o, y_o in self.obstacles:
            if (x - x_o)**2 + (y - y_o)**2 <= (diameter/2 + d/2)**2:
                return True
        return False
    
    def apf(self, x, y, goal=[0, 0], rho=5, k=1, eta_0=1.2, gamma=2, k_rep=0.1, nu=1):
        # Common implementation of paraboloidal close, conical away potential field
        e = [goal[0] - x, goal[1] - y]
        d = np.sqrt(e[0]**2 + e[1]**2)
        F_x = 0
        F_y = 0
        if d > 1e-6:
            if d <= rho:
                F_x = k * e[0]
                F_y = k * e[1]
            else:
                F_x = k * rho * e[0] / d
                F_y = k * rho * e[1] / d
            
        # Common implementation of repulsive potential field
        for d, x_o, y_o in self.obstacles:
            r = np.sqrt((x - x_o)**2 + (y - y_o)**2)
            eta = r - d/2
            if r <= eta_0*d/2:
                fx = k_rep / r**2 * (1/eta - 1/(eta_0*d/2))**(gamma-1) * (x - x_o) / r
                fy = k_rep / r**2 * (1/eta - 1/(eta_0*d/2))**(gamma-1) * (y - y_o) / r
                aux = 1 #np.random.choice([-1, 1])
                # F_x += nu * fx + (1 - nu) * aux * fy
                # F_y += nu * fy - (1 - nu) * aux * fx
                F_x += fx + nu * aux * fy
                F_y += fy - nu * aux * fx
        return F_x, F_y
    
    def cbf(self, x, y, r=0, goal=[0, 0], alpha=1, beta=1):
        # Common implementation of control barrier function
        e = [goal[0] - x, goal[1] -y]
        u_des = [beta * e[0], beta * e[1]]
        u = cp.Variable(2)
        obj = cp.Minimize(cp.sum_squares(u - u_des))
        constraints = []
        for d, x_o, y_o in self.obstacles:
            x_i = np.array([x-x_o, y-y_o])
            h = np.linalg.norm(x_i)**2 - (d/2)**2 - (r/2)**2 # Safety function
            grad_h = 2 * x_i
            constraints.append(grad_h @ u >= -alpha * h)
        prob = cp.Problem(obj, constraints)
        prob.solve()
        return u.value

