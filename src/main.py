#!/usr/bin/env python
import os
import rospy
from multiagent_env import MultiAgentGazeboEnv
import pickle
from sumo_scenario import SumoScenario
from ecosystem import Ecosystem
import neat
import visualize
import numpy as np
from datetime import datetime
from operator import attrgetter
from itertools import izip_longest
import random


def fitness_function(env, pops):
    configs = []
    best_genomes = []
    for pop in pops:
        configs.append(pop.config)
        genomes = list(pop.population.values())
        random.shuffle(genomes)
        genome_set = izip_longest(genomes[0:(len(genomes)//2)], genomes[(len(genomes)//2):], fillvalue=genomes[0])
    for gs in genome_set:
        eval_genomes(gs, env, pop.config)


def eval_genomes(genomes, env, config):
    nets = []
    for i, genome in enumerate(genomes):
        nets.append(neat.nn.FeedForwardNetwork.create(genome, config))
    # execution loop
    obs_n = env.reset()
    total_reward_n = [0] * env.num_agents
    fitnesses = []
    num_runs = 1
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
            if any(done_n):
                obs_n = env.reset()
                break
        obs_n = env.reset()
        fitnesses.append(total_reward_n)
        total_reward_n = [0] * env.num_agents
    for i, genome in enumerate(genomes):
        genome.fitness = min([f[i] for f in fitnesses])


def run():
    num_populations = 1

    # create environment and population
    scenario = SumoScenario(num_robots=2)
    env = MultiAgentGazeboEnv(reward_callback=scenario.reward,
                              observation_callback=scenario.observation,
                              done_callback=scenario.done)
    populations = []
    configs = []
    stats = []
    for _ in range(num_populations):
        # Load the config file, which is assumed to live in the same directory as this script.
        local_dir = os.path.dirname(__file__)
        config_path = os.path.join(local_dir, 'config')
        config = neat.Config(neat.DefaultGenome,
                            neat.DefaultReproduction,
                            neat.DefaultSpeciesSet,
                            neat.DefaultStagnation,
                            config_path)
        pop = neat.Population(config)
        s = neat.StatisticsReporter()
        pop.add_reporter(s)
        pop.add_reporter(neat.StdOutReporter(True))
        populations.append(pop)
        configs.append(config)
        stats.append(s)

    num_gen = 100
    ecosystem = Ecosystem(env, populations)
    best_genomes = ecosystem.run(fitness_function, n=num_gen)

    time = datetime.now().strftime("%m%d%Y_%H%M%S")  # current date and time in string format

    # Create new directory in which to save results
    path = '../catkin_ws/src/sumobots/results/%s_%d' % (time, num_gen)
    os.mkdir(path)

    # Save results
    for i, winner in enumerate(best_genomes):
        with open(path + '/genome_%d' % (i + 1), 'wb') as f:
            pickle.dump(winner, f)
        configs[i].save(path + '/config_%d' % (i + 1))
        node_names = {0: 'forward',
                      1: 'backward',
                      2: 'left',
                      3: 'right',
                      -1: 'center_dist',
                      -2: 'center_dir',
                      -3: 'other_dist',
                      -4: 'other_dir'
                      }
        visualize.draw_net(configs[i], winner, True, filename=path + "/nn_%d.svg" % (i + 1), node_names=node_names)
        visualize.plot_stats(stats[i], ylog=True, view=True, filename=path + "/fitness_%d.svg" % (i + 1))
        visualize.plot_species(stats[i], view=True, filename=path + "/speciation_%d.svg" % (i + 1))


def play_winners():
    # Watch the winners play pursuer-evader game.
    num_each_pop = [[0, 1]]
    # create scenarios
    environments = []
    for e in num_each_pop:
        scenario = SumoScenario(num_robots=sum(e))
        env = MultiAgentGazeboEnv(reward_callback=scenario.reward,
                                  observation_callback=scenario.observation,
                                  done_callback=scenario.done)
        environments.append(env)
    
    node_names = {0: 'forward',
                  1: 'backward',
                  2: 'left',
                  3: 'right',
                  -1: 'center_dist',
                  -2: 'center_dir',
                  -3: 'other_dist',
                  -4: 'other_dir'
                  }
    #path = '../catkin_ws/src/sumobots/results/11302020_230953_100'
    path = '../catkin_ws/src/sumobots/results/12012020_112811_50'
    configs = []
    winners = []
    for i in range(sum(num_each_pop[0])):
        config_path = path + '/config_%d' % (i + 1)
        print(config_path)
        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                             neat.DefaultSpeciesSet, neat.DefaultStagnation,
                             config_path)
        configs.append(config)
        winner = pickle.load(open(path + '/genome_%d' % (i + 1), 'rb'))
        winners.append(winner)
        visualize.draw_net(config, winner, True, filename=path + "/nn_%d.svg" % (i + 1), node_names=node_names)
    nets = []
    for i, genome in enumerate(winners):
        nets.append(neat.nn.FeedForwardNetwork.create(genome, configs[i]))
    env = environments[0]
    # execution loop
    total_reward_n = [0] * env.num_agents
    obs_n = env.reset()
    steps_per_run = 60
    while not rospy.is_shutdown():
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
            if any(done_n):
                obs_n = env.reset()
                break
        obs_n = env.reset()


def test_action():
    scenario = SumoScenario(num_robots=1)
    env = MultiAgentGazeboEnv(reward_callback=scenario.reward,
                              observation_callback=scenario.observation,
                              done_callback=scenario.done)
    act_n = [[[0, 0, 1.0, 0]]]
    while not rospy.is_shutdown():
        obs_n, reward_n, done_n, _ = env.step(act_n)
        if any(done_n):
            obs_n = env.reset()


if __name__ == '__main__':
    rospy.init_node('sumo')
    run()
    #play_winners()
    #test_action()


