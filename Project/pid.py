import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# =====================================================
# PID CONTROL
# dh/dt = (1/A)*(Qin - Qout)
# Qout = k_out * h 
# =====================================================
A =  5 
H_MAX = 10 
H_REF = 5     


Kp = 183.0
Ki = 317.0
Kd = 10.0

# k_out*h(t)
k_out =1

# SIMULATION TIME
T0 = 0.0
TF = 100.0
DT = 0.05
T = np.arange(T0, TF + DT, DT)

# =====================================================
# PID CONTROLLER CLASS
# =====================================================
class PID:
    def __init__(self, kp, ki, kd, umin=0.0, umax=12.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.umin = umin
        self.umax = umax
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        u = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        # saturate output
        return np.clip(u, self.umin, self.umax)

# =====================================================
# FLOW FUNCTIONS
# =====================================================
def QOUT(h):
    return k_out * h

# =====================================================
# NUMERICAL SIMULATION WITH PID
# =====================================================
h = np.zeros_like(T)
Qin = np.zeros_like(T)
error = np.zeros_like(T)

pid = PID(Kp, Ki, Kd, umin=0, umax=20.0)
pid.reset()

for i in range(len(T)-1):
    hi = h[i]
    ti = T[i]

    # compute error
    e = H_REF - hi
    error[i] = e

    # compute PID output (Qin)
    q_in = pid.update(e, DT)

    # safety: do not exceed H_MAX
    if hi >= H_MAX:
        q_in = 0
        pid.integral *= 0.9  # anti-windup

    Qin[i] = q_in

    # RK4 integration for dh/dt
    def f(hh, qin_val):
        return (1.0/A)*(qin_val - QOUT(hh))

    k1 = f(hi, q_in)
    k2 = f(hi + DT*k1/2, q_in)
    k3 = f(hi + DT*k2/2, q_in)
    k4 = f(hi + DT*k3, q_in)
    h[i+1] = hi + (DT/6)*(k1 + 2*k2 + 2*k3 + k4)

    # ensure physical limit
    if h[i+1] < 0:
        h[i+1] = 0.0

Qin[-1] = Qin[-2]
error[-1] = error[-2]

# =====================================================
# ANIMATION
# =====================================================
plt.style.use("seaborn-v0_8-darkgrid")
fig = plt.figure(figsize=(14, 6))
gs = fig.add_gridspec(1, 3, width_ratios=[1.4, 1, 1])

ax_tank = fig.add_subplot(gs[0])
ax_h = fig.add_subplot(gs[1])
ax_q = fig.add_subplot(gs[2])

# Tank display
ax_tank.set_xlim(0, 1)
ax_tank.set_ylim(0, H_MAX + 1)
ax_tank.axis("off")

TANK_LEFT = 0.25
TANK_WIDTH = 0.5
TANK_BOTTOM = 0.1

tank = patches.Rectangle((TANK_LEFT, TANK_BOTTOM), TANK_WIDTH, H_MAX,
                         edgecolor="black", facecolor="none", lw=2)
water = patches.Rectangle((TANK_LEFT, TANK_BOTTOM), TANK_WIDTH, 0.0,
                          facecolor="#1f77b4", alpha=0.85)
ax_tank.add_patch(tank)
ax_tank.add_patch(water)

surface, = ax_tank.plot([TANK_LEFT, TANK_LEFT+TANK_WIDTH],
                        [TANK_BOTTOM, TANK_BOTTOM], color="white", lw=2)
text_h = ax_tank.text(0.5, H_MAX+0.5, "", ha="center", fontsize=12, weight="bold")
text_q = ax_tank.text(0.5, H_MAX+0.2, "", ha="center", fontsize=10)

# Water level plot
ax_h.set_title("Water Level h(t)")
ax_h.set_xlim(T0, TF)
ax_h.set_ylim(0, H_MAX+0.5)
line_h, = ax_h.plot([], [], lw=2)
ax_h.axhline(H_REF, ls="--", color="k")

# Qin plot
ax_q.set_title("PID Output")
ax_q.set_xlim(T0, TF)
ax_q.set_ylim(0, Qin.max()*1.2)
line_q, = ax_q.plot([], [], lw=2, color="orange")

hx, hy, qx, qy = [], [], [], []

def update(frame):
    hi = h[frame]
    ti = T[frame]
    q = Qin[frame]

    # Tank
    display_h = min(hi, H_MAX)
    water.set_height(display_h)
    surface.set_ydata([TANK_BOTTOM+display_h]*2)
    text_h.set_text(f"h = {hi:.3f}")
    text_q.set_text(f"Qin = {q:.2f}")

    # plot data
    hx.append(ti)
    hy.append(hi)
    qx.append(ti)
    qy.append(q)

    line_h.set_data(hx, hy)
    line_q.set_data(qx, qy)

    ax_h.set_xlim(max(0, ti-80), ti+1)
    ax_q.set_xlim(max(0, ti-80), ti+1)

    return water, surface, line_h, line_q, text_h, text_q

ani = FuncAnimation(fig, update, frames=len(T), interval=30, blit=False)
fig.suptitle("Tank PID Control (Qin Adjusted Automatically)", fontsize=14)
plt.show()
