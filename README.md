# Coevolved Sumobots


## About
This project applies concepts from within the field of coevolution to evolve complex robot behavior in the context of a "sumo" fighting ring. Specifically, this project uses the NeuroEvolution of Augmenting Topologies (NEAT) algorithm to evolve the structure of neural networks used to control our sumobots.

## Dependencies
- Python 2.7
- ROS Melodic
- Gazebo
- NEAT
  - ```$ pip install neat-python```
- Graphviz
  - ```$ pip install graphviz```

## Table of Contents
1. [Introduction](#introduction)
2. 


## Introduction <a name="introduction"></a>
#### Problem Statement
The original objective of this project was to design an autonomous robot in the style of a sumo combat robot. These are small, agile, autonomous robots designed to force their adversary (in our case, a physically identical copy of the robot) out of the circular sumo arena. This project encompassed the creation of the custom simulated robot models, the ROS Gazebo world with the sumo arena, and the primary task of programming robot behavior. The goal for coding behavior was to find a way to map observations from sensor data onto linear and angular velocity values in a sufficiently intelligent manner so as to produce interesting, sophisticated, and entertaining behavior in the sumo wrestling match. We chose at the outset of the project to make use of coevolutionary methods to accomplish this goal. Specifically, we proposed each robot be controlled by a neural network. During training, robots are drawn from a population, paired up within the sumo arena, and assigned a fitness payoff in accordance with the outcome of the fight. Robots then have a likelihood of "reproducing" based on their assigned fitness payoff.

#### Relevant Literature
We make use of the [NEAT algorithm](http://nn.cs.utexas.edu/downloads/papers/stanley.cec02.pdf) (Stanley and Miikkulainen, 2002) to evolve the neural network topology. Other papers that informed our process includes [(Ficici and Pollack, 1998)](https://pdfs.semanticscholar.org/9979/ababa4100cf35afc1c8be8777326134d14fd.pdf), specifically their work on the problem of "Mediocre Stable States" within competitive coevolution.

## References
1. K. O. Stanley and R. Miikkulainen, "Efficient evolution of neural network topologies," Proceedings of the 2002 Congress on Evolutionary Computation. CEC'02 (Cat. No.02TH8600), Honolulu, HI, USA, 2002, pp. 1757-1762 vol.2, doi: 10.1109/CEC.2002.1004508.
2. Ficici, Sevan & Pollack, Jordan. (1998). Challenges in Coevolutionary Learning: Arms-Race Dynamics, Open-Endedness, and Mediocre Stable States. 

@INPROCEEDINGS{1004508,  author={K. O. {Stanley} and R. {Miikkulainen}},  booktitle={Proceedings of the 2002 Congress on Evolutionary Computation. CEC'02 (Cat. No.02TH8600)},   title={Efficient evolution of neural network topologies},   year={2002},  volume={2},  number={},  pages={1757-1762 vol.2},  doi={10.1109/CEC.2002.1004508}}

@INPROCEEDINGS{Ficici98challengesin,
    author = {Sevan Ficici and Jordan B. Pollack},
    title = {Challenges in Coevolutionary Learning: Arms-Race Dynamics, Open-Endedness, and Mediocre Stable States},
    booktitle = {Proceedings of the Sixth International Conference on Artificial Life},
    year = {1998},
    pages = {238--247},
    publisher = {MIT Press}
}



