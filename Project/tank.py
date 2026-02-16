import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# =====================================================
# SINGLE TANK – OPEN LOOP (NO CONTROLLER)
# MODEL BASED ON PHYSICAL DYNAMICS
#
# dh/dt = (1/A) * (Qin - Qout)
#
# A     : tank cross-sectional area
# Qin   : inlet flow (adjustable)
# Qout  : outlet flow (adjustable, valve)
# h(t)  : water level
# =====================================================

# ----------------------
# PHYSICAL PARAMETERS
A = 0.5         # tank area
H_MAX = 10       # maximum tank height (safety / visualization)

# ----------------------
# TIME SETTINGS
T0 = 0.0
TF = 200.0
DT = 0.05
T = np.arange(T0, TF + DT, DT)

# ----------------------
# USER-DEFINED FLOWS
# YOU CAN CHANGE THESE FREELY

def QIN(t):
        return 6.0

def QOUT(t, h):
    # example: gravity-driven outlet proportional to height
    # you can replace this with any function you want
    k_out = 2
    return k_out * h

# ----------------------
# TANK DYNAMICS
def dhdt(h, t):
    return (1.0 / A) * (QIN(t) - QOUT(t, h))

# ----------------------
# NUMERICAL INTEGRATION (RK4)
h = np.zeros_like(T)

for i in range(len(T) - 1):
    ti = T[i]
    hi = h[i]

    k1 = dhdt(hi, ti)
    k2 = dhdt(hi + DT * k1 / 2, ti + DT / 2)
    k3 = dhdt(hi + DT * k2 / 2, ti + DT / 2)
    k4 = dhdt(hi + DT * k3, ti + DT)

    h[i + 1] = hi + (DT / 6) * (k1 + 2*k2 + 2*k3 + k4)

    # physical constraint
    if h[i + 1] < 0:
        h[i + 1] = 0.0

QIN_VEC = np.array([QIN(ti) for ti in T])
QOUT_VEC = np.array([QOUT(ti, hi) for ti, hi in zip(T, h)])

# =====================================================
# ANIMATION
# =====================================================
plt.style.use("seaborn-v0_8-darkgrid")
fig = plt.figure(figsize=(14, 6))
gs = fig.add_gridspec(1, 3, width_ratios=[1.4, 1, 1])

ax_tank = fig.add_subplot(gs[0])
ax_h = fig.add_subplot(gs[1])
ax_q = fig.add_subplot(gs[2])

# ----------------------
# TANK VIEW
ax_tank.set_xlim(0, 1)
ax_tank.set_ylim(0, H_MAX + 1)
ax_tank.axis("off")

TANK_LEFT = 0.25
TANK_WIDTH = 0.5
TANK_BOTTOM = 0.1

tank = patches.Rectangle(
    (TANK_LEFT, TANK_BOTTOM),
    TANK_WIDTH,
    H_MAX,
    linewidth=2,
    edgecolor="black",
    facecolor="none"
)
water = patches.Rectangle(
    (TANK_LEFT, TANK_BOTTOM),
    TANK_WIDTH,
    0.0,
    facecolor="#1f77b4",
    alpha=0.85
)

ax_tank.add_patch(tank)
ax_tank.add_patch(water)

surface, = ax_tank.plot(
    [TANK_LEFT, TANK_LEFT + TANK_WIDTH],
    [TANK_BOTTOM, TANK_BOTTOM],
    color="white",
    lw=2
)

text_h = ax_tank.text(0.5, H_MAX + 0.5, "", ha="center", fontsize=12, weight="bold")
text_q = ax_tank.text(0.5, H_MAX + 0.2, "", ha="center", fontsize=10)

# ----------------------
# PLOTS
ax_h.set_title("Water Level h(t)")
ax_h.set_xlim(T0, TF)
ax_h.set_ylim(0, H_MAX + 0.5)
line_h, = ax_h.plot([], [], lw=2)

ax_q.set_title("Flows")
ax_q.set_xlim(T0, TF)
ax_q.set_ylim(0, max(QIN_VEC.max(), QOUT_VEC.max()) * 1.2)
line_qin, = ax_q.plot([], [], lw=2, label="Qin")
line_qout, = ax_q.plot([], [], lw=2, label="Qout")
ax_q.legend()

hx, hy = [], []
qx, qin_y, qout_y = [], [], []

# ----------------------
# UPDATE FUNCTION
def update(i):
    hi = h[i]
    ti = T[i]

    display_h = min(hi, H_MAX)
    water.set_height(display_h)
    surface.set_ydata([TANK_BOTTOM + display_h]*2)

    text_h.set_text(f"h = {hi:.3f}")
    text_q.set_text(f"Qin = {QIN_VEC[i]:.2f}   |   Qout = {QOUT_VEC[i]:.2f}")

    hx.append(ti)
    hy.append(hi)

    qx.append(ti)
    qin_y.append(QIN_VEC[i])
    qout_y.append(QOUT_VEC[i])

    line_h.set_data(hx, hy)
    line_qin.set_data(qx, qin_y)
    line_qout.set_data(qx, qout_y)

    ax_h.set_xlim(max(0, ti - 80), ti + 1)
    ax_q.set_xlim(max(0, ti - 80), ti + 1)

    return water, surface, line_h, line_qin, line_qout, text_h, text_q

# ----------------------
# RUN
ani = FuncAnimation(fig, update, frames=len(T), interval=5, blit=False)
fig.suptitle("Single Tank Open-Loop Dynamics (Qin / Qout Adjustable) & The Qout is closed during the valve time and opens as time passes.", fontsize=12)
plt.show()
