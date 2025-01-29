import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from planning import waypoint_selection, planning
from vessel_dynamics import vessel_dynamics
from integration import integration
from obstacle_sim import obstacle_sim
from cpa_calculations import cpa_calculations
from cpa_calculations_0speed import cpa_calculations_0speed
from controller import controller
from actuator_modeling import actuator_modeling
from risk_calculations import risk_calculations
#from decision_making import decision_making
from reactive_avoidance import reactive_avoidance
from animate import animate_step
import time


# Initialize parameters
t = 0.0  # initial time
Sim_time = 920  # simulation time
dt = 0.1  # time step
Ts = dt  # sampling time
N = round(Sim_time / dt) 
Animation = 1  # enable animation

# Initial conditions
x_v, y_v, psi_v = 0.0, 0.0, np.radians(45)  # initial position and heading
r_v, b_v, u_v = 0.0, 0.0, 0.0  # initial rates and velocity
ui_psi1 = 0.0  # initial integral of yaw error

X_0 = np.array([x_v, y_v, psi_v, r_v, b_v, u_v])
X = X_0.copy()

Sat_amp_s = 20
i = 0

# Waypoints
Xwpt = [0, 5000, 12000]
Ywpt = [0, 5000, 5000]
i_wpt = 1

LOA_own, BOL_own = 30, 16
CPA_own = LOA_own * 2

Xob = [6000.0, 8000.0]
Yob = [2000.0, 5000.0]
Vob = [9.5, 0.0]
psiob = np.radians([150, 0])

LOA_ob = [80, 0]
BOL_ob = [30, 0]
CPA_ob = [LOA_ob[0] * 1, 200]

# Prepare for animation if enabled
if Animation == 1:
    fig, ax = plt.subplots()
    plt.plot(Xwpt, Ywpt, 'ob', Xwpt, Ywpt, ':b', linewidth=1.0)
    plt.grid(True)
    plt.xlabel(r'$X (m)$')
    plt.ylabel(r'$Y (m)$')
    writer = animation.PillowWriter(fps=5)

# Main simulation loop
time = []
x, y, psi = np.zeros(N), np.zeros(N), np.zeros(N)
r, b, u = np.zeros(N), np.zeros(N), np.zeros(N)
v_c = np.zeros(N)
u_p, tau_c, tau_ac = np.zeros(N), np.zeros(N), np.zeros(N)
psi_p, psi_wp, psi_oa = np.zeros(N), np.zeros(N), np.zeros(N)

V_x, V_y = np.zeros(N), np.zeros(N)

Xobs, Yobs, Vxobs, Vyobs = (np.zeros((N, len(Xob))) for _ in range(4))

DCPA, TCPA, Vrel, alpha, psi_Vrel = (np.zeros((N, len(Xob))) for i in range(5))
DCPA[:1], TCPA[:1] = 1000, 1000

DCPA2, TCPA2, Vrel2, alpha2, psi_Vrel2 = (np.zeros((N, len(Xob))) for _ in range(5))

Distance_ob, Bearing_ob, Risk = (np.zeros((N, len(Xob))) for _ in range(3))

ui_psi1 = 0.0


with writer.saving(fig, "scenario_animation.gif", dpi=200):

    while t <= Sim_time:
        # Fetching data
        time.append(t)
        x[i] = X[0]
        y[i]= X[1]
        psi[i] = X[2]
        r[i] = X[3]
        b[i] = X[4]
        u[i] = X[5]
        X_0 = X.copy()

        # Speed command
        u_p[i] = 16.0

        # Path planning based on waypoints
        #psi_w[i], i_wpt = planning(Xwpt, Ywpt, x[-1], y[-1], t, i_wpt)
        i_wpt = waypoint_selection(Xwpt, Ywpt, x[i], y[i], i_wpt)
        #i_wpt = 1
        psi_wp[i] = planning(Xwpt, Ywpt, x[i], y[i], i_wpt)
        #print(i_wpt)

        # Collision avoidance
        psi_oa[i], w_B, w_R, Distance_ob[i, :], Bearing_ob[i, :] = reactive_avoidance(Xob, Yob, x[i], y[i], psi[i], t)

        # Overall yaw command
        psi_p[i] = psi_wp[i] + psi_oa[i]

        # Controller section
        #psi_p[i] = np.deg2rad(45)  # (Test) Desired heading
        tau_c[i], v_c[i], ui_psi1 = controller(
            psi_p[i], psi[i], r[i], u_p[i], b, ui_psi1, Ts)

        # Actuator modeling
        tau_ac[i] = actuator_modeling(tau_c[i], Sat_amp_s)
        
        inputs = [tau_ac[i], v_c[i]]

        # System dynamics simulation
        X_dot = vessel_dynamics(X_0, inputs)
        X = integration(X_0, X_dot, dt)
        
        V_x[i] = X_dot[0]
        V_y[i] = X_dot[1] 

        # Obstacles simulation
        Xob, Yob, Vxob, Vyob = obstacle_sim(Xob, Yob, Vob, psiob, dt)
        Xobs[i, :] = Xob  # Record the updated Xob values
        Yobs[i, :] = Yob  # Record the updated Yob values
        Vxobs[i, :] = Vxob  # Record the updated Vxob values
        Vyobs[i, :] = Vyob  # Record the updated Vyob values

        # Risk analysis and collision prevention
        for j in range(len(Xob)):
            if i >= 1:  # Note: Indices adjusted to match Python convention (i starts from 0)
            # CPA calculations with updated indexing and parameters
                DCPA[i, j], TCPA[i, j], Vrel[i, j], alpha[i, j], psi_Vrel[i, j] = cpa_calculations(
                x[i], y[i], x[i-1], y[i-1], Xobs[i, j], Yobs[i, j], Xobs[i-1, j], Yobs[i-1, j], Ts
            )

                #DCPA2[i, j], TCPA2[i, j], Vrel2[i, j], alpha2[i, j], psi_Vrel2[i, j] = cpa_calculations_0speed(
                #x[i], y[i], Xobs[i, j], Yobs[i, j], V_x[i], V_y[i], Vxobs[i, j], Vyobs[i, j], Distance_ob[i, j]
            #)

            # Calculate risk using the correct DCPA and TCPA values
            Risk[i, j] = risk_calculations(DCPA[i, j], TCPA[i, j], Distance_ob[i, j], Vrel[i, j])

        # Decision making- to be developed - All actions move to starboard
        # #COLREG_no[i], Action, Heading_dir, Speed_Level, Relative_Bearing_ob = decision_making(
        #    x[-1], y[-1], psi[-1], Xob, Yob, psiob, Vrel, u[-1], Risk[i - 1])

        # Animation

        animate_step(x[i], y[i], psi[i], LOA_own, BOL_own, CPA_own, Xob, Yob, psiob, LOA_ob, BOL_ob, CPA_ob, Risk[i, :], Vob, i)
        
        if i % 101 == 0 and i != 0:
            writer.grab_frame()
                

        # Time accumulation
        t += dt
        i += 1



# Final animation saving or static plot
#if Animation == 1:
plt.savefig('simulation_result.png')
plt.show(block=True)

#else:
plt.figure()
plt.plot(x, y, 'b', linewidth=2)
plt.plot(Xwpt, Ywpt, 'or')
plt.plot(Xwpt, Ywpt, '-.r', linewidth=1.0)
plt.plot(Xobs, Yobs, ':g', linewidth=2)
plt.scatter(Xob, Yob, color='black')
plt.xlabel(r'$X (m)$')
plt.ylabel(r'$Y (m)$')
plt.show()

plt.subplot(211)
plt.plot(time, np.degrees(psi_p), ':r', time, np.degrees(psi), 'b', linewidth=1.5)
plt.legend([r'Desired $\psi$', r'Actual $\psi$'])
plt.xlabel('Time (s)')
plt.ylabel(r'Heading ($^\circ$)')

plt.subplot(212)
plt.plot(time, tau_ac, 'b', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel(r'Control Input $\tau_c$ (Nm)')
plt.show()  

plt.figure()
fig, axs = plt.subplots(2, 2)

axs[0, 0].plot(time, DCPA[:, 0], 'b', linewidth=1.0)
axs[0, 0].set_xlim([0, Sim_time])
axs[0, 0].set_ylabel(r'$DCPA$ (m)')

axs[0, 1].plot(time, Distance_ob[:, 0], 'b', linewidth=1.0)
axs[0, 1].set_xlim([0, Sim_time])
axs[0, 1].set_ylim([0, 2000])
axs[0, 1].set_ylabel(r'$R$ (m)')

axs[1, 0].plot(time, TCPA[:, 0], 'b', linewidth=1.0)
axs[1, 0].set_xlim([0, Sim_time])
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel(r'$TCPA$ (s)')

axs[1, 1].plot(time, Risk[:, 0], 'b', linewidth=1.0)
axs[1, 1].set_xlim([0, Sim_time])
axs[1, 1].set_ylim([0, 1])
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel(r'$Risk$')

plt.tight_layout()
plt.show()

'''
plt.figure()
plt.plot(time,np.rad2deg(psi), 'b', linewidth=1.5)

plt.show()
'''