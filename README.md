# Integrated_Risk_Assessment_and_Collision_Avoidance_a_Maritime_Autonomous_Surface_Ship_MASS_Python

<p align="center">
  <img src="https://github.com/user-attachments/assets/db2aea61-0ac4-4794-ac0a-68249c334e21" width="50%">
</p>


This repo includes the Python implementation of practical waypoint tracking, online risk assessment, collision avoidance, and control algorithms, all implemented in a simulation of a MASS. The mission planning incorporates Line of Sight (LOS), Cross-Track Error (CTE), and Collision Avoidance (COLAV) terms [1]. The controller is a PD, and the vehicle model is a simple yet realistic representation of a vessel with stochastic disturbances. A fuzzy online collision risk assessment is also provided. Users can modify the initial conditions to simulate different scenarios.
<p align="center">
  <img src="https://github.com/user-attachments/assets/a42017b8-a164-4370-b33d-2d50edbc1925" width="60%">
</p>

An LLM based decision-making algorithm is also proposed and augmented to this system that its descripts can be found in [2]. 


### 📝 Citation

[1] P. Sarhadi, W. Naeem and N. Athanasopoulos, "A survey of recent machine learning solutions for ship collision avoidance and mission planning", 14th IFAC Conference on Control Applications in Marine Systems, Robotics and Vehicles, Kongens Lyngby, Denmark, Sept. 2022, Elsevier, 2022, Vol: 55(31), pp: 257-268.

[2] K. Agyei, P. Sarhadi, W. Naeem, "Large Language Model-based Decision-making for COLREGs and the Control of Autonomous Surface Vehicles", arXiv preprint arXiv:2411.16587, 2025.

