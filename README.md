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
2. [What Was Created](#whatwascreated)


## Introduction <a name="introduction"></a>
### Problem Statement
The objective of this project was to design an autonomous robot in the style of a sumo combat robot. These are small, agile, autonomous robots designed to force their adversary (in our case, a physically identical copy of the robot) out of the circular sumo arena. The scope of our project involved the design of the custom simulated robot models as well as the Gazebo world with the sumo arena, and the primary task of programming robot behavior. Our goal was to use coevolutionary methods to evolve the topology of neural networks that would be used to control our robots. Specifically, during training, robots are drawn from a population, paired up within the sumo arena, and assigned a fitness payoff in accordance with the outcome of the fight. Robots then have a likelihood of "reproducing" based on their assigned fitness payoff. With enough time and a sufficiently large population, the hypothesis was that we'd see robot behavior that was interesting, sophisticated, and entertaining in the sumo wrestling match. 

### Relevant Literature
We make use of the [NEAT algorithm](http://nn.cs.utexas.edu/downloads/papers/stanley.cec02.pdf) (Stanley and Miikkulainen, 2002) to evolve the neural network topology. Other papers that informed our process includes [(Ficici and Pollack, 1998)](https://pdfs.semanticscholar.org/9979/ababa4100cf35afc1c8be8777326134d14fd.pdf), specifically their work on the problem of "Mediocre Stable States" within competitive coevolution.

## What Was Created <a name="whatwascreated"></a>
### Technical Descriptions and Illustrations
![An arena with two sumobots](https://i.imgur.com/OFNZExb.jpg)
The first model we created was an arena in Gazebo on which the robots could fight, seen above. The arena is a 1.5m radius by 0.3m tall cylinder with a blue ring surrounding it which is 0.1m thick; the blue ring was created for the possibility of adapting this setup to use data from the camera on the sumobot. This surface of the arena is similar to any generic hard surface with an average amount of friction.
![A closeup of one sumobot](https://i.imgur.com/tMIMyfb.jpeg)
The second task we had was to create the sumobot itself. The sumobot was cannibalized from a waffle model Turtlebot3 for ease of development. The sumobot is a differential drive rover with two wheels, two casters, and a cylindrical body as seen above. The bot also features a LiDAR, camera, and IMU. Most of the mass of the bot is held in the wheels and casters to balance the center of gravity equally. The wheels have a very high friction coefficient in the forwards and backwards direction meaning the bot can start and stop quickly, however, the friction coefficient is lower in the horizontal direction causing the bot to slide when hit from the side by another robot. The differential drive control has been updated to have very high acceleration and torque values allowing for agile movement when combined with the high friction wheels. The cylindrical body was specified in such a way to collide perfectly inelastically to produce more interesting combat behavior as discussed in detail in the "Problems Encountered" section. Additionally, the radius of the body was intentionally made relatively small to encourage hits which flipped robots up to again bring about the possibility of unique combat techniques. Finally, the sumobot models were simplified as much as possible (such as wheel models being changed to simple cylinders) to increase performance of the simulation; this change doesn't affect robot behavior however because the collision boxes of these parts were already simple objects.
### Algorithms, Modules, Techniques

### Problems Encountered
A problem we encountered during the 
One of the problems we encountered during coevolutionary training was stagnation caused by the existence of Mediocre Stable States (MSS), a phenomenon that is well-documented within the coevolutionary literature [(Ficici and Pollack, 1998)](https://pdfs.semanticscholar.org/9979/ababa4100cf35afc1c8be8777326134d14fd.pdf). Specifically, our sumobots consistently got stuck with a strategy of simply remaining stationary throughout the entire fight. This was evidently due to the fact that any movement at all turned out to be too high a risk, that is, would too often result in the robot falling off the sumo arena platform. The solution we came up with was to modify our payoff assignments in two ways. Firstly, we explicitly incentivized movement toward one's opponent by assigning a negative payoff to each robot proportional with the distance between the robots. And secondly, we withheld any positive payoff from the winning robot if the winner was more than 1 meter away from the opponent at the moment when the opponent fell off the platform. These two changes worked together to incentivize active movement of the robots toward each other, and thereby make for a more interesting and exciting fight.

## Reflections


## References
1. K. O. Stanley and R. Miikkulainen, "Efficient evolution of neural network topologies," Proceedings of the 2002 Congress on Evolutionary Computation. CEC'02 (Cat. No.02TH8600), Honolulu, HI, USA, 2002, pp. 1757-1762 vol.2, doi: 10.1109/CEC.2002.1004508.
2. S. Ficici and J. B. Pollack, "Challenges in Coevolutionary Learning: Arms-Race Dynamics, Open-Endedness, and Mediocre Stable States," Proceedings of the Sixth International Conference on Artificial Life. MIT Press.

@INPROCEEDINGS{1004508,  author={K. O. {Stanley} and R. {Miikkulainen}},  booktitle={Proceedings of the 2002 Congress on Evolutionary Computation. CEC'02 (Cat. No.02TH8600)},   title={Efficient evolution of neural network topologies},   year={2002},  volume={2},  number={},  pages={1757-1762 vol.2},  doi={10.1109/CEC.2002.1004508}}

@INPROCEEDINGS{Ficici98challengesin,
    author = {Sevan Ficici and Jordan B. Pollack},
    title = {Challenges in Coevolutionary Learning: Arms-Race Dynamics, Open-Endedness, and Mediocre Stable States},
    booktitle = {Proceedings of the Sixth International Conference on Artificial Life},
    year = {1998},
    pages = {238--247},
    publisher = {MIT Press}
}



