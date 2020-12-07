"""
The ParallelEvaluator class in the neat-python module (github link below) modified to accommodate
evaluating genomes in multiagent tasks.

-------------https://github.com/CodeReclaimers/neat-python/blob/master/neat/parallel.py------------

Runs evaluation functions in parallel subprocesses
in order to evaluate multiple sets of genomes at once.
"""
from multiprocessing import Pool
from collections import defaultdict
import time
import copy_reg
import types

def _pickle_method(method):
    """
    Author: Steven Bethard 
    http://bytes.com/topic/python/answers/552476-why-cant-you-pickle-instancemethods
    """
    func_name = method.im_func.__name__
    obj = method.im_self
    cls = method.im_class
    cls_name = ''
    if func_name.startswith('__') and not func_name.endswith('__'):
        cls_name = cls.__name__.lstrip('_')
    if cls_name:
        func_name = '_' + cls_name + func_name
    return _unpickle_method, (func_name, obj, cls)


def _unpickle_method(func_name, obj, cls):
    """
    Author: Steven Bethard
    http://bytes.com/topic/python/answers/552476-why-cant-you-pickle-instancemethods
    """
    for cls in cls.mro():
        try:
            func = cls.__dict__[func_name]
        except KeyError:
            pass
        else:
            break
    return func.__get__(obj, cls)

# This call to copy_reg.pickle allows you to pass methods as the first arg to
# mp.Pool methods. If you comment out this line, `pool.map(self.foo, ...)` results in
# PicklingError: Can't pickle <type 'instancemethod'>: attribute lookup
# __builtin__.instancemethod failed

copy_reg.pickle(types.MethodType, _pickle_method, _unpickle_method)



class ParallelEvaluator(object):
    def __init__(self, num_workers, eval_function, genome_grouping_function, timeout=None):
        """
        eval_function should take one argument, a tuple of
        (genome object, config object), and return
        a single float (the genome's fitness).
        """
        self.num_workers = num_workers
        self.eval_function = eval_function
        self.genome_grouping_function = genome_grouping_function
        self.timeout = timeout
        self.pool = Pool(num_workers)

    def __del__(self):
        self.pool.close() # should this be terminate?
        self.pool.join()

    def evaluate(self, genomes, config):
        genome_sets = self.genome_grouping_function(genomes)
        jobs = []
        for genome_set in genome_sets:
            jobs.append(self.pool.apply_async(self.eval_function, (genome_set, config)))

        # average fitness across all runs for each genome and assign back to each genome
        self.assign_fitnesses(jobs, genome_sets)
        
    def assign_fitnesses(self, jobs, genome_sets):
        genome_dict = defaultdict(int)
        for job, genome_set in zip(jobs, genome_sets):
            for i, genome in enumerate(genome_set):
                num_runs = genome_dict[genome]
                if num_runs == 0:
                    genome.fitness = job.get(timeout=self.timeout)[i]
                else:
                    genome.fitness = (genome.fitness * num_runs + job.get(timeout=self.timeout)[i]) / (num_runs + 1)
                genome_dict[genome] += 1