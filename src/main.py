#!/usr/bin/env python
import rospy
import pickle
from sumo_trainer import SumoTrainer
import neat

'''
---------------------------------------------------------------------------------------------------
Main script for running the sumobots game, including coevolutionary training for the specified
number of generations.

Author: Joseph Pickens, August Soderberg
---------------------------------------------------------------------------------------------------
'''

# train for the given number of generations
def train(num_generations):
    trainer = SumoTrainer()
    trainer.run(num_generations)

# play the winning genomes from the given saved results filepath
def play_winners(results_path):
    winners = []
    config_path = results_path + '/config'
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)
    for i in range(2):
        winner = pickle.load(open(results_path + '/genome_%d' % (i + 1), 'rb'))
        winners.append(winner)

    trainer = SumoTrainer()
    while not rospy.is_shutdown():
        trainer.eval_genome_pair(tuple(winners), config)


if __name__ == '__main__':
    rospy.init_node('sumo')
    #train(350)
    play_winners('../catkin_ws/src/sumobots/results/12092020_054242_300')


