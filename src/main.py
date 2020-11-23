#!/usr/bin/env python
import os
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))
sys.path.insert(2, os.path.join(sys.path[1], 'gym-gazebo'))
sys.path.insert(3, os.path.join(sys.path[1], 'neat-python'))
import rospy
from multiagent_env import MultiAgentGazeboEnv
import pickle
from sumo_scenario import SumoScenario
from ecosystem import Ecosystem
import neat
import visualize
import numpy as np
from itertools import izip_longest as zip_longest
import random
from datetime import datetime


def fitness_function(envs, pops):
    for i, env_pops in enumerate(pops):
        populations = []
        configs = []
        for j, pop in enumerate(env_pops):
            configs.append(pop.config)
            temp = list(pop.population.values())
            random.shuffle(temp)
            populations.append(temp)
        # Line up populations to form genome sets to evaluate on the given environment
        # TODO: This needs a more general solution
        smaller_pop = np.argmin([len(p) for p in populations])
        genome_sets = zip_longest(*populations, fillvalue=populations[smaller_pop][0])
        for gs in genome_sets:
            eval_genomes(gs, envs[i], configs)


def eval_genomes(genomes, env, configs):
    nets = []
    for i, genome in enumerate(genomes):
        nets.append(neat.nn.FeedForwardNetwork.create(genome, configs[i]))
    # execution loop
    obs_n = env.reset()
    total_reward_n = [0] * len(env.num_agents)
    fitnesses = []
    num_runs = 5
    steps_per_run = 60
    for _ in range(num_runs):
        for _ in range(steps_per_run):
            # query for action from each agent's policy
            act_n = []
            for i, net in enumerate(nets):
                actions = net.activate(obs_n[i])
                actions = [1.0 if a >= 0.5 else 0 for a in actions]
                act_n.append([actions])
            # step environment
            obs_n, reward_n, done_n, _ = env.step(act_n)
            total_reward_n = [a + b for a, b in zip(total_reward_n, reward_n)]
        obs_n = env.reset()
        fitnesses.append(total_reward_n)
        total_reward_n = [0] * len(env.world.agents)
    for i, genome in enumerate(genomes):
        genome.fitness = min([f[i] for f in fitnesses])


def run():
    # Define ecosystem structure as a list of lists, where each inner list represents an
    # environment indicating number of pursuers and evaders for that environment
    eco_structure = [[0, 1]]

    # create environments and populations
    environments = []
    populations = []
    stats = []
    configs = []
    for e in eco_structure:
        scenario = SumoScenario(num_robots=sum(e))
        env = MultiAgentGazeboEnv('/launch/single_agent.launch',
                            reward_callback=scenario.reward,
                            observation_callback=scenario.observation,
                            done_callback=scenario.done)
        environments.append(env)
        env_pops = []
        for _ in range(sum(e)):
            # Load the config_1_agents file, which is assumed to live in
            # the same directory as this script.
            local_dir = os.path.dirname(__file__)
            config_path = os.path.join(local_dir, 'config_%d_agents' % sum(e))
            config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                 neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                 config_path)
            configs.append(config)
            pop = neat.Population(config)
            s = neat.StatisticsReporter()
            pop.add_reporter(s)
            pop.add_reporter(neat.StdOutReporter(True))
            env_pops.append(pop)
            stats.append(s)
        populations.append(env_pops)
    num_gen = 20
    ecosystem = Ecosystem(environments, populations)
    best_genomes = ecosystem.run(fitness_function, n=num_gen)

    time = datetime.now().strftime("%m%d%Y_%H%M%S")  # current date and time in string format

    # Create new directory in which to save results
    path = '../results/%s_%d' % (time, num_gen)
    os.mkdir(path)

    # Save results
    for i, winner in enumerate(best_genomes):
        with open(path + '/genome_%d' % (i + 1), 'wb') as f:
            pickle.dump(winner, f)
        configs[i].save(path + '/config_%d' % (i + 1))
        node_names = {0: 'stay',
                      1: 'right',
                      2: 'left',
                      3: 'forward',
                      4: 'backward',
                      -1: 'x_self',
                      -2: 'y_self',
                      -3: 'x_landmark_1',
                      -4: 'y_landmark_1',
                      -5: 'x_landmark_2',
                      -6: 'y_landmark_2',
                      -7: 'x_other',
                      -8: 'y_other'
                      }
        visualize.draw_net(configs[i], winner, True, filename=path + "/nn_%d.svg" % (i + 1), node_names=node_names)
        visualize.plot_stats(stats[i], ylog=True, view=True, filename=path + "/fitness_%d.svg" % (i + 1))
        visualize.plot_species(stats[i], view=True, filename=path + "/speciation_%d.svg" % (i + 1))


def play_winners():
    # Watch the winners play pursuer-evader game.
    eco_structure = [[0, 1]]
    # create scenarios
    environments = []
    for e in eco_structure:
        scenario = SumoScenario(num_robots=sum(e))
        env = MultiAgentGazeboEnv('/launch/single_agent.launch',
                            reward_callback=scenario.reward,
                            observation_callback=scenario.observation,
                            done_callback=scenario.done)
        environments.append(env)
    path = 'results/11172020_175028_500'
    configs = []
    winners = []
    for i in range(sum(eco_structure[0])):
        local_dir = os.path.dirname(__file__)
        config_path = os.path.join(local_dir, path + '/config_' + str(i + 1))
        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                             neat.DefaultSpeciesSet, neat.DefaultStagnation,
                             config_path)
        configs.append(config)
        winner = pickle.load(open(path + '/genome_' + str(i + 1), 'rb'))
        winners.append(winner)
    nets = []
    for i, genome in enumerate(winners):
        nets.append(neat.nn.FeedForwardNetwork.create(genome, configs[i]))
    env = environments[0]
    # execution loop
    obs_n = env.reset()
    steps_per_run = 200
    while True:
        for _ in range(steps_per_run):
            # query for action from each agent's policy
            act_n = []
            for i, net in enumerate(nets):
                actions = net.activate(obs_n[i])
                actions = [1.0 if a >= 0.5 else 0 for a in actions]
                act_n.append(np.concatenate([actions, np.zeros(env.world.dim_c)]))
            # step environment
            obs_n, reward_n, done_n, _ = env.step(act_n)
        obs_n = env.reset()


def test_action():
    scenario = SumoScenario(num_robots=1)
    env = MultiAgentGazeboEnv(reward_callback=scenario.reward,
                              observation_callback=scenario.observation,
                              done_callback=scenario.done)
    act_n = [[[0, 0, 0, 1.0]]]
    k = 0
    while k < 50:
        obs_n, reward_n, done_n, _ = env.step(act_n)
        k += 1


if __name__ == '__main__':
    rospy.init_node('sumo')
    #run()
    #play_winners()
    test_action()


