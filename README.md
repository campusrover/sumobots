# Coevolved Sumobots


## About
This project is an attempt to apply concepts from the field of coevolution to evolve complex robot behavior in the context of a "sumo" fighting ring. Specifically, this project uses the NeuroEvolution of Augmenting Topologies (NEAT) algorithm to evolve the structure of neural networks used to control our sumobots. Click <a href="https://youtu.be/DfJWQu9lygw" target="_blank">here</a> to watch a short video summary of our project.

## Dependencies
- Python 2.7
- ROS Melodic
- Gazebo
- NEAT
  - ```$ pip install neat-python```
- Graphviz
  - ```$ pip install graphviz```

## How To Run
### Training the Sumobots
There are a few settings to consider when beginning a new training session.

First you will need to select the size of your population you want to train; go to ```src/config``` and change the ```pop_size``` parameter to be the size of the population you want.

Next, go to ```src/main.py``` and at the bottom make sure to comment out the ```play_winners(...)``` line and uncomment the ```train(...)``` line. As an argument for the ```train(...)``` function, pass in the number of generations you would like your training session to run.

Next, go to the ```src/sumo_trainer.py``` file and notice the first line of the ```fitness_function(self, genomes, config):``` method calls a method of pairing starting with the word ```pair```, this describes how robots will be matched up to fight each generation; feel free to pick another one of the ```pair_...``` methods from the ```sumo_trainer.py``` file and insert it there if you would like.

Once all of that is set up, go ahead and navigate in your Unix shell to the ```sumobots``` directory.

Use the command ```$ roslaunch sumobots arena.launch``` to begin the simulation.

Once it is all up and running, click the play button at the bottom of Gazebo and minimize the Gazebo simulator, and the shell will notify you as each generation goes by.

### Playing the Winning Sumobots
Once you have evolved a robot, you can watch the two best genomes from your learning battle each other. Go back to ```src/main.py``` and comment out the ```train(...)``` line at the bottom. In the results folder you will find a new subdirectory specified by the date, time, and number of generations of your run. Copy that subdirectory name and paste it in the now uncommented ```play_winners(...)``` line as follows: ```play_winners('../catkin_ws/src/sumobots/results/[YOUR_RESULT_SUBDIRECTORY]')```

Now you can launch again using the command ```$ roslaunch sumobots arena.launch``` to watch the best robots fight.

## Table of Contents
1. [Introduction](#introduction)
    1. [Problem Statement](#problemstatement)
    2. [Relevant Literature](#literature)
2. [What Was Created](#whatwascreated)
    1. [Technical Descriptions and Illustrations](#technical)
        1. [Robot and Arena Models](#models)
    2. [Algorithms, Modules, Techniques](#algorithms)
        1. [Neural Network](#neuralnetwork)
        2. [The NEAT Algorithm](#neatalgorithm)
            1. [Species in NEAT](#speciesneat)
        3. [Evolutionary Training in Gazebo](#gazebotraining)
        4. [Our Training Scheme](#trainingscheme)
            1. [Genome Pairing Techniques](#genomepairing)
            2. [Assigning Payoffs](#payoffs)
    3. [Problems Encountered](#problemsencountered)
3. [Reflections](#reflections)
4. [References](#references)

<a name="introduction"></a>
## Introduction <a name="problemstatement"></a>
### Problem Statement
The objective of this project was to design an autonomous robot in the style of a sumo combat robot. These are small, agile, autonomous robots designed to force their adversary (in our case, a physically identical copy of the robot) out of the circular sumo arena. The scope of our project involved the design of the custom simulated robot models as well as the Gazebo world with the sumo arena, and the primary task of programming robot behavior. Our goal was to use coevolutionary methods to evolve the topology of the neural networks that would be used to control our robots. Specifically, during training, robots are drawn from a population, paired up within the sumo arena, and assigned a fitness payoff in accordance with the outcome of the fight. Robots then have a likelihood of "reproducing" based on their assigned fitness payoff. With enough time and a sufficiently large population, the hypothesis was that we'd see robot behavior that was interesting, sophisticated, and entertaining in the sumo wrestling match. 
<a name="literature"></a>
### Relevant Literature 
We make use of the NeuroEvolution of Augmenting Topologies (NEAT) algorithm [(Stanley and Miikkulainen, 2002)](#refneat) to evolve our neural network topologies. Other papers that informed our process include [(Ficici and Pollack, 1998)](#refmss), specifically their work on the problem of "Mediocre Stable States" within competitive coevolution. 
<a name="whatwascreated"></a>
## What Was Created <a name="technical"></a>
### Technical Descriptions and Illustrations <a name="models"></a>
#### Robot and Arena Models
<img src="https://i.imgur.com/OFNZExb.jpg" width="600" />

The first model we created was an arena in Gazebo on which the robots could fight, seen above. The arena is a 1.5m radius by 0.3m tall cylinder with a blue ring surrounding it which is 0.1m thick; the blue ring was created for the possibility of adapting this setup to use data from the camera on the sumobot. This surface of the arena is similar to any generic hard surface with an average amount of friction.

<img src="https://i.imgur.com/tMIMyfb.jpeg" width="600" />

The second task we had was to create the sumobot itself. The sumobot was cannibalized from a waffle model Turtlebot3 for ease of development. The sumobot is a differential drive rover with two wheels, two casters, and a cylindrical body as seen above. The bot also features a LiDAR, camera, and IMU. Most of the mass of the bot is held in the wheels and casters to balance the center of gravity equally. The wheels have a very high friction coefficient in the forwards and backwards direction meaning the bot can start and stop quickly, however, the friction coefficient is lower in the horizontal direction causing the bot to slide when hit from the side by another robot. The differential drive control has been updated to have very high acceleration and torque values allowing for agile movement when combined with the high friction wheels. The cylindrical body was specified in such a way to collide elastically to produce more interesting combat behavior as discussed in detail in the [Problems Encountered](#problemsencountered) section. Additionally, the radius of the body was intentionally made relatively small to encourage hits which flipped robots up to again bring about the possibility of unique combat techniques. Finally, the sumobot models were simplified as much as possible (such as wheel models being changed to simple cylinders) to increase performance of the simulation; this change doesn't affect robot behavior however because the collision boxes of these parts were already simple objects.
<a name="algorithms"></a>
### Algorithms, Modules, Techniques <a name="neuralnetwork"></a>
#### Neural Network 
Each of our robots is controlled by a neural network. Our networks map from a continuous observation space to a discrete action space. Our observation space consists of the following four observations:
1. The distance in meters between the robot and the center of the sumo arena.
2. The direction in radians of the center of the sumo arena relative to the robot's orientation.
3. The distance in meters between the robot and its opponent.
4. The direction in radians of the robot's opponent relative to the robot's orientation.

These relative coordinate values are obtained using the ROS transform. This technique is notably limited and likely unworkable in a real-world environment, but nevertheless works well within a Gazebo simulated environment.

Our action space consists of the following four discrete actions:
1. Move forward at 2.0 m/s
2. Move backward at 2.0 m/s
3. Rotate left at 2.0 rad/s
4. Rotate right at 2.0 rad/s

Importantly, our actions are not one-hot vectors, but rather allow for any combination of movement using the above four actions.

We experimented with both feed-forward as well as recurrent neural networks. Our networks used sigmoid activation functions, but other activation functions are readily available within the NEAT package.
<a name="neatalgorithm"></a>
#### The NEAT Algorithm
We make use of the NeuroEvolution of Augmenting Toplogies (NEAT) algorithm [(Stanley and Miikkulainen, 2002)](https://ieeexplore.ieee.org/document/1004508) to evolve the structure of our neural networks. NEAT draws inspiration from biolgoy in that it encodes each neural network as a "genome" object. Each genome contains node and connection "genes" which encode for the neural network phenotype. We initialize a population of such genomes, each encoding for some number of random connections between input and output nodes according to the NEAT configuration parameters. In our sumobot application, genomes are then paired up according to a chosen pairing method and evaluated in the sumo wrestling ring. Payoffs are then assigned to each genome as fitness metrics according to the outcome of the fight. After the evaluation period is over and fitness values have been assigned to each genome, NEAT selects a fraction of the population of genomes to reproduce using genetic crossover. The size of this fraction is specified by the user in the NEAT configuration file. We used 0.2 for our trials. In addition to the diversity introduced to the population via genetic crossover, mutations are also introduced into the population after the reproduction phase. This new generation of genomes then returns to the evaluation period, and the cycle repeats for the specified number of generations.
<a name="speciesneat"></a>
##### Species in NEAT
The NEAT algorithm divides the population into multiple "species" during training according to genome similarity. Each genome is then only evaluated against other genomes within their own species, that is, similar genomes. This allows for new genetic innovations that might not be initially adaptive to nonetheless have a chance at "proving" themselves, so to speak, later on down the evolutionary road.
<a name="gazebotraining"></a>
#### Evolutionary Training in Gazebo
Both reinforcement learning and evolutionary learning require assessing the state of one or more agents at each time step and assigning payoffs to the agent based on that state. To do this in the context of a Gazebo simulation, it is necessary to pause the simulation while this code is executing at each time step. Additionally, it is necessary to be able to reset the simulated world upon completion of the training task in order to begin the next episode of training. To accomplish this for our project, we adapted the GazeboConnection class from the [ROS OpenAI package](#refopenairos). This class provides functions for pausing, unpausing, and resetting the Gazebo simulation that can be called at each time step of training. To this class we added a function for setting the state of each robot model, such that we could easily assign a randomized pose to each robot at the start of each fight.

We adapted and simplified the MultiAgentEnv class from OpenAI's [multiagent particle environment](#refopenaimape) package for a strictly discrete action space and for use in Gazebo via integration with the aforementioned GazeboConnection class.
<a name="trainingscheme"></a>
#### Our Training Scheme
<a name="genomepairing"></a>
##### Genome Pairing Techniques
One of the variables we experimented with in our training scheme was the manner in which we went about pairing genomes during evaluation in the sumo ring. The following methods were tried:
1. Randomized one-vs-one: The population of genomes is shuffled, split in half, and the two sub-populations paired off accordingly.
2. All-vs-all: All possible unique combinations of genome pairs are assessed, and the payoffs across all fights averaged for determining fitness.
3. All-vs-some: each genome plays against n other unique genomes.
4. Intra-species pairing: This method pairs genomes from the same species using one of the above three methods, and thus only pairs genomes that are relatively similar to one another.

The first technique is the fastest, since each genome is only paired once in each generation. However, it also gives the poorest sence of each genome's true adaptive capacity, because each is only given one chance to fight. Any genome that happens to be paired up with a particularly self-destructive genome is going to be assigned a high fitness value that does not reflect its true skill level.

The second technique affords a much more representative evaluation of each genome, since each genome is paired up with every other genome in the population and the payoffs averaged across all runs. However, such a pairing scheme takes considerable time to run, and is thus often impractical for any considerably sized population.

The third pairing technique attempts to get the best of both the first and second methods, by pairing each genome with only a few other genomes. That way, fitness values are more representative of a genome's true adaptivity since they will be the averaged result across several fights, but will nevertheless keep the training time down by limiting each genome to only a small number of opponent match-ups.

The fourth and final pairing technique makes use of NEAT's speciation feature. By only pairing genomes from the same species together, this ensures a more fair fight, since genomes within the same species are more similar than those outside the species. By ensuring each pairing is of a relatively similar skill level, we are more likely to maintain a meaningful learning gradient for each genome to evolve along. The alternative is a problem in which one genome so far outperforms its opponent, that there is no chance for the opponent to find whatever learning gradient might otherwise be there to find.

Based on the explanation we've presented here, we recommend using the intra-species all-vs-some pairing method in future training runs.
<a name="payoffs"></a>
##### Assigning Payoffs
The design of payoffs is of utmost importance to successful evolutionary training. Our original payoff scheme was as follows:
1. Each robot is assigned a small negative payoff at each time step, so as to incentivize acting quickly in the fight.
2. If a robot falls off the arena platform, they receive a large negative payoff, and the fight ends.
3. If a robot's opponent falls off the arena platform, they receive a large positive payoff, and the fight ends.

After this payoff scheme yielded unimpressive results, we modified it as follows:
1. Each robot is assigned a small negative payoff at each time step proportional to their distance in space from their opponent, so as to incentivize movement toward one's opponent.
2. Same as before.
3. If a robot's opponent falls off the arena platform, they receive a large positive payoff as before, but only if within 1 meter of their opponent at the final time step, so as to withhold reward from robots that simply happened to be paired with a self-destructive opponent.
<a name="spawning"></a>
##### Spawning
Our initial approach was to spawn each robot in the same pose for each fight: on opposite sides of the arena facing one another. However, toward the end of the project we realized this was likely leading to behavior that depended on this initial pose in order to be adaptive. As an alternative, we introduced random spawning for each fight, assigning a random pose to each robot. However, due to the time constraints of the project, we were unable to run a robust set of experiments using this method. This is discussed more in the following section.
<a name="problemsencountered"></a>
### Problems Encountered
The single biggest challenge of this project was the time constraint, simply due to how much time is required for training. Even with a virtual cloud desktop with 8 CPU cores, the Gazebo real time factor lingered around a value of 2. Attempts to improve this by increasing the maximum step size of our simulation altered the physics of the simulation too heavily. Thus, training often took multiple days to perform, making experimentation and broad-scale debugging extremely time consuming.

A problem encountered during the creation of the robot models was the issue of surface type on each robot. We ideally wanted the robots to collide into each other and bounce off as they would in real life, however this was incredibly difficult to create in Gazebo. As an aside, please refer to our [lab notebook contribution](https://campus-rover.gitbook.io/lab-notebook/miscellaneous/bouncy-objects) on creating bouncy surfaces in Gazebo to see how we made sure the robots always collide  elastically. This however doesn't solve our issue because Gazebo doesn't simulate kinetic friction any different from static friction, so what happens is if the robots have enough traction to drive forward at one another, they have enough friction to resist that motion in the opposite direction. The solution we created was to have the robots have a much weaker coefficient of friction in the lateral direction so they are able to easily push each other side to side, even if not forwards and backwards.

A problem we encountered during coevolutionary training was stagnation caused by the existence of Mediocre Stable States (MSS), a phenomenon that is well-documented within the coevolutionary literature [(Ficici and Pollack, 1998)](https://pdfs.semanticscholar.org/9979/ababa4100cf35afc1c8be8777326134d14fd.pdf). Specifically, our sumobots often appeared to oscillate between two mediocre states: that of remaining entirely stationary, and that of constant forward motion. The former we attribute to the high risk involved in any movement whatsoever. In the event that no viable movement in response to the robot's environment can be found, the genomes selected for are those that play it most safe (i.e. those that don't move at all). As for the latter, this behavior only arises when our spawning method was consistent for each fight, that is, when we spawn each robot on opposite sides of the arena and facing one another. From here, direct forward motion often leads to forcing one's opponent out of the ring. In fact, we found that when both robots employ this simple strategy of constant forward motion, the result is actually quite an entertaining fight. That being said, there is obviously nothing remotely intelligent about such behavior.

Toward the end of the project, we modified our spawning method to randomize the initial pose of each robot for each fight. We hoped this would lead to more robust behavior, rather than behavior that relies on a given starting pose in order to be successful. However, more experimentation is needed to test this hypothesis.

One avenue we explored briefly was the idea of simulating multiple fights simultaneously using parallel processing. However, the time constraints on our project did not allow us to realize this improvement, which would have sped up training considerably. We recommend looking into this possibility for anyone who wishes to build upon our work.

<a name="reflections"></a>
## Reflections
In the end, we are very proud of the work we put together during this project. Not only did we learn an incredible amount about a complex field, we created the base for vastly more complex projects to come. We solved countless problems we couldn't have even imagined we would encounter, we dove headfirst into research on fascinating topics, and we came away feeling like we created something we can be proud of.

We worked really well together, divided work between us well, and tackled our own challenges with the level of detail required while being able to support the other when they became truly stuck on something. We entered this project with two very different ideas about what we wanted to research and were able to combine our ideas into a single, much more interesting, project.

While we are obviously disappointed that we never saw an absolutely dominant sumobot, we hope that we are able to build off of all the work we have done to eventually see a robot which could best any manually programmed sumobot created. The framework is already built to continue researching this topic and we hope the results from this project only continue to expand.
<a name="references"></a>
## References
<a name="refneat"></a>
1. K. O. Stanley and R. Miikkulainen, "Efficient evolution of neural network topologies," Proceedings of the 2002 Congress on Evolutionary Computation. CEC'02 (Cat. No.02TH8600), Honolulu, HI, USA, 2002, pp. 1757-1762 vol.2, doi: 10.1109/CEC.2002.1004508. https://ieeexplore.ieee.org/document/1004508
<a name="refmss"></a>
2. S. Ficici and J. B. Pollack, "Challenges in Coevolutionary Learning: Arms-Race Dynamics, Open-Endedness, and Mediocre Stable States," Proceedings of the Sixth International Conference on Artificial Life. MIT Press. https://www.researchgate.net/publication/2765711_Challenges_in_Coevolutionary_Learning_Arms-Race_Dynamics_Open-Endedness_and_Mediocre_Stable_States
<a name="refopenairos"></a>
3. OpenAI ROS Package. https://bitbucket.org/theconstructcore/openai_ros/src/kinetic-devel/
<a name="refopenaimape"></a>
4. OpenAI Multi Agent Particle Environment. https://github.com/openai/multiagent-particle-envs
