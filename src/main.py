#!/usr/bin/env python
import rospy
import pickle
from sumo_trainer import SumoTrainer
import neat


def train(num_generations):
    trainer = SumoTrainer()
    trainer.run(num_generations)

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
    train(1000)
    #play_winners('../catkin_ws/src/sumobots/results/12042020_140426_500')
    #test_action()


