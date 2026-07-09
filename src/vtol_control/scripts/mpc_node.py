#!/usr/bin/env python3
# MPC position controller: N=20 steps x 0.1 s horizon, RK4-discretized
# point-mass model, solved with CasADi/ipopt at 10 Hz.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import casadi as ca

class MpcNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.N = 20        # horizon steps
        self.dt = 0.1      # step size -> 2 s lookahead
        self.mass = 3.05
        self.g = 9.8
        self.state = None      # [x,y,z,vx,vy,vz]
        self.target = None     # [tx,ty,tz]
        self.mode = 0.0
        self.fails = 0

        self.create_subscription(Odometry, '/vtol_uav/odom_est', self.odom_cb, 10)
        self.create_subscription(Float64MultiArray, '/mpc_target', self.target_cb, 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/mpc_cmd', 10)
        self.build_solver()
        self.create_timer(0.1, self.solve)
        self.get_logger().info('MPC v1: CasADi/ipopt, N=20, dt=0.1, RK4 point-mass model')

    def odom_cb(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.state = [p.x, p.y, p.z, v.x, v.y, v.z]

    def target_cb(self, msg):
        self.target = [msg.data[0], msg.data[1], msg.data[2]]
        self.mode = msg.data[3]

    def dynamics(self, s, u):
        # s=[x,y,z,vx,vy,vz], u=[pitch, roll, thrust]
        ax = self.g * u[0] - 0.1 * s[3]          # small-angle tilt + drag
        ay = -self.g * u[1] - 0.1 * s[4]
        az = u[2] / self.mass - self.g - 0.2 * s[5]
        return ca.vertcat(s[3], s[4], s[5], ax, ay, az)

    def rk4(self, s, u):
        k1 = self.dynamics(s, u)
        k2 = self.dynamics(s + self.dt / 2 * k1, u)
        k3 = self.dynamics(s + self.dt / 2 * k2, u)
        k4 = self.dynamics(s + self.dt * k3, u)
        return s + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def build_solver(self):
        self.opti = ca.Opti()
        self.S = self.opti.variable(6, self.N + 1)   # states over horizon
        self.U = self.opti.variable(3, self.N)       # controls over horizon
        self.s0 = self.opti.parameter(6)
        self.ref = self.opti.parameter(3)

        cost = 0
        self.opti.subject_to(self.S[:, 0] == self.s0)
        for k in range(self.N):
            self.opti.subject_to(self.S[:, k + 1] == self.rk4(self.S[:, k], self.U[:, k]))
            perr = self.S[0:3, k + 1] - self.ref
            cost += 10.0 * ca.sumsqr(perr)                       # reach target
            cost += 0.1 * ca.sumsqr(self.S[3:6, k + 1])          # calm velocity
            cost += 1.0 * ca.sumsqr(self.U[0:2, k])              # small tilts
            cost += 0.001 * (self.U[2, k] - self.mass * self.g) ** 2

        self.opti.subject_to(self.opti.bounded(-0.15, self.U[0, :], 0.15))  # ~8.6 deg
        self.opti.subject_to(self.opti.bounded(-0.15, self.U[1, :], 0.15))
        self.opti.subject_to(self.opti.bounded(10.0, self.U[2, :], 45.0))
        self.opti.minimize(cost)
        self.opti.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0,
                                   'ipopt.max_iter': 60})

    def solve(self):
        if self.state is None or self.target is None:
            return
        if self.mode not in (0.0,1.0, 4.0, 5.0):   # only HOVER and BRAKE
            return
        self.opti.set_value(self.s0, self.state)
        self.opti.set_value(self.ref, self.target)
        try:
            sol = self.opti.solve()
            u0 = sol.value(self.U[:, 0])
            msg = Float64MultiArray()
            msg.data = [float(u0[0]), float(u0[1]), float(u0[2])]
            self.pub_cmd.publish(msg)
            # warm start next solve
            self.opti.set_initial(self.S, sol.value(self.S))
            self.opti.set_initial(self.U, sol.value(self.U))
        except RuntimeError:
            self.fails += 1
            self.get_logger().warn(f'MPC solve failed ({self.fails} total) - PD fallback active')

def main():
    rclpy.init()
    rclpy.spin(MpcNode())

if __name__ == '__main__':
    main()
