import time
import matplotlib.pyplot as plt
from rendering import animate_ship, animate_static_obstacle

# Function to animate ship and obstacles

def animate_step(x, y, psi, LOA_own, BOL_own, CPA_own, Xob, Yob, psiob, LOA_ob, BOL_ob, CPA_ob, Risk, Vob, step):
    if step % 200 == 0:
        animate_ship(x, y, psi, LOA_own * 5, BOL_own * 5, CPA_own, [0.2, 0.2, 0.2])

        plt.xlabel(r'$X$ (m)')  # Set x-axis label
        plt.ylabel(r'$Y$ (m)')  # Set y-axis label

        plt.draw()  # Update the plot
        plt.pause(0.1)  


    # Uncomment this section if you want to handle obstacles
    if step % 400 == 0:
        for j in range(len(Xob)):
            obs_col = [0.0, 0.7, 0.0]
            if Risk[j] > 0.75:
                obs_col = [1.0, 0.0, 0.0]
            elif Risk[j] > 0.6:
                obs_col = [1.0, 0.6, 0.0]
            elif Risk[j] > 0.35:
                obs_col = [1.0, 0.9, 0.0]

            if Vob[j] > 0.5:
                animate_ship(Xob[j], Yob[j], psiob[j], LOA_ob[j] * 3, BOL_ob[j] * 3, CPA_ob[j], obs_col)
            else:
                animate_static_obstacle(Xob[j], Yob[j], CPA_ob[j], obs_col)