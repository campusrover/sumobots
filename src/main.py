#!/usr/bin/env python
import rospy
from sumo_trainer import SumoTrainer


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
        trainer.eval_genome_set(tuple(winners))


if __name__ == '__main__':
    rospy.init_node('sumo')
    trainer = SumoTrainer()
    num_generations = 500
    trainer.run(num_generations)
    #play_winners('../catkin_ws/src/sumobots/results/12012020_112811_50')
    #test_action()


