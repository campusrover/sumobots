import os
import multiprocessing
import pickle
import random
import numpy as np
from operator import attrgetter
from datetime import datetime
from itertools import izip_longest, combinations
from collections import defaultdict
from multiagent_env import MultiAgentGazeboEnv
from sumo_scenario import SumoScenario
import neat
import visualize

'''
---------------------------------------------------------------------------------------------------
Performs coevolutionary training of neural networks on the sumobots game, using the NEAT package to
evolve neural network topologies. Creates a population of genomes according to specifications in
the NEAT config file, runs the population for n generations, evaluating genomes using the trainer's
evaluation function, and saves the results.

Author: Joseph Pickens, August Soderberg
---------------------------------------------------------------------------------------------------
'''
class SumoTrainer:
    def __init__(self):
        # create sumo environment
        scenario = SumoScenario()
        self.env = MultiAgentGazeboEnv(scenario.num_robots,
                                       reward_callback=scenario.reward,
                                       observation_callback=scenario.observation,
                                       done_callback=scenario.done)
        # load the config file, which is assumed to live in the same directory as this class.
        local_dir = os.path.dirname(__file__)
        config_path = os.path.join(local_dir, 'config')
        self.config = neat.Config(neat.DefaultGenome,
                            neat.DefaultReproduction,
                            neat.DefaultSpeciesSet,
                            neat.DefaultStagnation,
                            config_path)
        # create population of genomes and corresponding statistics reporters
        self.population = neat.Population(self.config)
        self.stats = neat.StatisticsReporter()
        self.population.add_reporter(self.stats)
        self.population.add_reporter(neat.StdOutReporter(True))
        # track the best two genomes
        self.best_genomes = [None] * self.env.num_agents

    def run(self, num_gen):
        self.population.run(self.fitness_function, n=num_gen)
        self.save_results(self.best_genomes, num_gen)

    def fitness_function(self, genomes, config):
        genome_pairs = self.pair_species_all_vs_all(genomes)
        genome_dict = defaultdict(int)
        for genome_pair in genome_pairs:
            latest_fitnesses = self.eval_genome_pair(genome_pair, config)
            num_matches = [genome_dict[g] for g in genome_pair]
            for i, n in enumerate(num_matches):
                g = genome_pair[i]
                if n == 0:
                    g.fitness = latest_fitnesses[i]
                else:
                    # average fitnesses from all sumo matches during this generation
                    g.fitness = (g.fitness * n + latest_fitnesses[i]) / (n + 1)
                genome_dict[g] += 1
        genomes = [g[1] for g in genomes]
        if not all([g.fitness is None for g in genomes]):
            self.update_best_genomes(genomes)

    def eval_genome_pair(self, genome_pair, config):
        nets = []
        for genome in genome_pair:
            nets.append(neat.nn.RecurrentNetwork.create(genome, config))
        # execution loop
        obs_n = self.env.reset()
        total_reward_n = [0] * self.env.num_agents
        steps_per_run = 300
        for _ in range(steps_per_run):
            # query for action from each agent's policy
            act_n = []
            for i, net in enumerate(nets):
                actions = net.activate(obs_n[i])
                actions = [1.0 if a >= 0.5 else 0 for a in actions]
                act_n.append(actions)
            # step environment
            obs_n, reward_n, done_n, _ = self.env.step(act_n)
            total_reward_n = [a + b for a, b in zip(total_reward_n, reward_n)]
            if any(done_n):
                break
        return total_reward_n

    def pair_all_vs_all(self, genomes):
        genomes = [g[1] for g in genomes]
        return list(combinations(genomes, self.env.num_agents))

    def pair_one_vs_one(self, genomes):
        genomes = [g[1] for g in genomes]
        random.shuffle(genomes)
        return izip_longest(genomes[0:(len(genomes) // 2)], genomes[(len(genomes) // 2):],
                            fillvalue=genomes[0])

    def pair_species_all_vs_all(self, genomes):
        genomes_grouped = self.group_genomes_by_species(genomes)
        genome_pairs = []
        for gs in genomes_grouped:
            genome_pairs.extend(self.pair_all_vs_all(gs))
        return genome_pairs

    def pair_species_one_vs_one(self, genomes):
        genomes_grouped = self.group_genomes_by_species(genomes)
        genome_pairs = []
        for gs in genomes_grouped:
            genome_pairs.extend(self.pair_one_vs_one(gs))
        return genome_pairs

    def group_genomes_by_species(self, genomes):
        genome_ids = [g[0] for g in genomes]
        species_ids = [self.population.species.get_species_id(i) for i in genome_ids]
        genome_species_map = zip(genomes, species_ids)
        species_unique_ids = set(species_ids)
        genomes_grouped = [[y[0] for y in genome_species_map if y[1]==x] for x in species_unique_ids]
        return genomes_grouped

    def update_best_genomes(self, genomes):
        best = self.population.best_genome
        genomes.extend(self.best_genomes)
        second_best = max([g for g in genomes if g is not None
                           and g.fitness is not None and g is not best],
                           key=attrgetter('fitness'))
        self.best_genomes = [best, second_best]

    def save_results(self, best_genomes, num_gen):
        # current date and time in string format
        time = datetime.now().strftime("%m%d%Y_%H%M%S")

        # Create new directory in which to save results
        path = '../catkin_ws/src/sumobots/results/%s_%d' % (time, num_gen)
        os.mkdir(path)

        # Save results
        node_names = {0: 'forward',
                      1: 'backward',
                      2: 'left',
                      3: 'right',
                     -1: 'center_dist',
                     -2: 'center_dir',
                     -3: 'other_dist',
                     -4: 'other_dir'
                     }
        for i, genome in enumerate(best_genomes):
            with open(path + '/genome_%d' % (i+1), 'wb') as f:
                pickle.dump(genome, f)
            visualize.draw_net(self.config, genome, True, filename=path + "/nn_%d.svg" % (i+1), node_names=node_names)
        self.config.save(path + '/config')
        visualize.plot_stats(self.stats, ylog=True, view=True, filename=path + "/fitness.svg")
        visualize.plot_species(self.stats, view=True, filename=path + "/speciation.svg")