# Randomised MPC Controller for F1Tenth

## Acknowledgments & Citations

This project integrates the GPU-accelerated Randomized Model Predictive Control (RMPC) solver developed by **Arun Muraleedharan** with the **F1TENTH** autonomous racing simulation framework.

### Core Controller
The core controller in `helper/randomized-mpc` is governed by the [PolyForm Noncommercial License 1.0.0](/rmpc_f1tenth/helper/randomized-mpc/LICENSE.md) (Copyright © 2026 Arun Muraleedharan). If you use this package in your research, please cite the foundational RMPC papers:

- A. Muraleedharan, H. Okuda, and T. Suzuki, "Real-Time Implementation of Randomized Model Predictive Control for Autonomous Driving," *IEEE Transactions on Intelligent Vehicles*, vol. 7, no. 1, pp. 86–97, 2022. doi: [10.1109/TIV.2021.3062730](https://doi.org/10.1109/TIV.2021.3062730)
- A. Muraleedharan, I. Simões, and G. Pau, "Randomized Model Predictive Control for Autonomous Racing," in *2025 IEEE Intelligent Vehicles Symposium (IV)*, 2025.

### Simulation Platform
Simulation validation and testing utilise [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros). If you use this platform, please cite the original F1TENTH simulator:

- M. O'Kelly, H. Zheng, D. Karthik, and R. Mangharam, "F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning," in *Proceedings of the NeurIPS 2019 Competition and Demonstration Track*, PMLR, 2020, pp. 77–89.

---

### BibTeX

```bibtex
@article{Muraleedharan_Real-Time_Implementation_of_2022,
author = {Muraleedharan, Arun and Okuda, Hiroyuki and Suzuki, Tatsuya},
doi = {10.1109/TIV.2021.3062730},
journal = {IEEE Transactions on Intelligent Vehicles},
number = {1},
pages = {11},
title = {{Real-Time Implementation of Randomized Model Predictive Control for Autonomous Driving}},
volume = {7},
year = {2022}
}

@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```

## License

This project is licensed under [Apache License 2.0](LICENSE)
