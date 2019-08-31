

## <center>Impact-aware multi contact motion generation with a QP controller</center>


[![Impact-aware QP controller](https://img.youtube.com/vi/K9ar8tsPN8s/0.jpg)](https://www.youtube.com/watch?v=K9ar8tsPN8s)
 
Impact-aware tasks are generally ignored in multi-objective whole body controllers of humanoid robots, which leads to the fact that a humanoid robot typically operates at
near-zero velocity to interact with the external environment. 

We explicitly investigate the propagation of the impact-induced velocity and torque jumps along the structure linkage, see [mc_impact_predictor](https://github.com/wyqsnddd/mc_impact_predictor) and propose [a set of constraints](https://github.com/wyqsnddd/mc_impact_constraints) that always keep the hardware limits,
contact maintenance conditions and the stability measure, i.e. the [zero moment point condition](doc/zmp.md). 
