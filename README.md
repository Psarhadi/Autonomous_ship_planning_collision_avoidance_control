# Integrated_Risk_Assessment_and_Collision_Avoidance_a_Maritime_Autonomous_Surface_Ship_MASS_Python

<p align="center">
  <img src="https://github.com/user-attachments/assets/db2aea61-0ac4-4794-ac0a-68249c334e21" width="50%">
</p>


This repo includes the Python implementation of practical waypoint tracking, online risk assessment, collision avoidance, and control algorithms, all implemented in a simulation of a MASS. The mission planning incorporates Line of Sight (LOS), Cross-Track Error (CTE), and Collision Avoidance (COLAV) terms [1]. The controller is a PD, and the vehicle model is a simple yet realistic representation of a vessel with stochastic disturbances. A fuzzy online collision risk assessment is also provided. Finally, it visualises the motion and saves the animation outcome. Users can modify the initial conditions to simulate different scenarios.

An LLM-based decision-making algorithm is also proposed and integrated into this system, with its description available in [2]. Feel free to amend and use the code with citations.

### 📝 Citation

[1] P. Sarhadi, W. Naeem and N. Athanasopoulos, "A survey of recent machine learning solutions for ship collision avoidance and mission planning", 14th IFAC Conference on Control Applications in Marine Systems, Robotics and Vehicles, Kongens Lyngby, Denmark, Sept. 2022, Elsevier, 2022, Vol: 55(31), pp: 257-268.

[2] K. Agyei, P. Sarhadi, W. Naeem, "Large Language Model-based Decision-making for COLREGs and the Control of Autonomous Surface Vehicles", arXiv preprint arXiv:2411.16587, 2025.

```bibtex
@inproceedings{sarhadi2022integrated,
  title={An Integrated Risk Assessment and Collision Avoidance Methodology for an Autonomous Catamaran with Fuzzy Weighting Functions},
  author={Sarhadi, Pouria and Naeem, Wasif and Athanasopoulos, Nikolaos},
  booktitle={2022 UKACC 13th International Conference on Control (CONTROL)},
  pages={228--234},
  year={2022},
  organization={IEEE}
}
```

```bibtex
@article{agyei2025large,
  title={Large Language Model-based Decision-making for COLREGs and the Control of Autonomous Surface Vehicles},
  author={Agyei, Klinsmann and Sarhadi, Pouria and Naeem, Wasif},
  journal={arXiv preprint arXiv:2411.16587},
  year={2024}
}
```

### Prerequisites
This is a simple implementation and basic tolls are used.
- An IDE to run Python, VScose is suggested;
- Python 3.x
- Numpy
- Matplotlib
- Time


### 🏄 More examples

<p align="center">
  <img src="https://github.com/user-attachments/assets/a42017b8-a164-4370-b33d-2d50edbc1925" width="60%">
</p>

Extra plots for the first figure:

<p align="center">
  <img src="https://github.com/user-attachments/assets/fc4f59fb-0573-4653-853e-5decdd08dbd4" width="60%">
</p>

