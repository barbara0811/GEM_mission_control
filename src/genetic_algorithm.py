#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'barbanas'

import time
import random
from copy import deepcopy
from hashlib import md5
from math import factorial, floor

import simulator
from ummc_agent_lib.taems.taems import TaemsTree
from ummc_agent_lib.taems.interrelationships import Interrelationship
from utilities import my_logger

logger = my_logger.CustomLogger('GENETIC')


class GeneticAlgorithm(object):
    """
    Genetic algorithm for scheduling unordered list of tasks.

    Attributes:
        populationSize (int): Size of the population used in algorithm.
        population (list[Chromosome]): Current population.
        populationID (list[str]): IDs of every chromosome in a population.
        rating (float): Rating based on percentage of temporal and precedence constraints met by every chromosome.
        fitness (float): Quality of every chromosome that unites rating and total schedule length.
        bestSolution (Chromosome): Best solution of a scheduling problem (with max fitness).
        elitePercentage (int): Percentage of population that is automatically promoted to next generation.
        worstPercentage (int): Percentage of population that is removed from population before creating new population.
        initialDurationEV (dict[str,float]): Initial expected value for every method's duration.
        start (float): Start time of the optimization procedure.
        """

    def __init__(self, popSize, elite, worst):
        """
        Initialize instance variables.

        Args:
            popSize (int): Size of the population used in algorithm.
            elite (float): Percentage of population that is automatically promoted to next generation.
            worst (float): Percentage of population that is removed from population before creating new population.
        """
        self.populationSize = popSize
        self.population = []
        self.populationID = []
        self.rating = []
        self.fitness = []
        self.bestSolution = None
        self.elitePercentage = elite
        self.worstPercentage = worst
        self.initialDurationEV = {}
        self.start = None

    def optimize(self, alternative, nonLocalTasks, tree, noIter, scheduleStartTime):
        """
        Start genetic algorithm for generating schedule from unordered list of methods.

        Args:
            alternative (list[str]): Labels of unordered methods (from taems task structure).
            nonLocalTasks (list[str]): Labels of non local tasks (from taems task structure).
            tree (TaemsTree): Taems tree that defines the task structure.
            noIter (int): Maximum number of iterations of algorithm (generations created).
            scheduleStartTime (float): Mission schedule start time.

        Returns:
            Best schedule.
        """
        sim = simulator.LightSimulator(tree)
        tasks = deepcopy(alternative)
        tasks.extend(nonLocalTasks)
        sim.execute_alternative(tasks, tasks)
        tasksToComplete = sim.completedTasks
        self.start = time.time()

        logger.debugs('Optimization start.\nAlternatives: %s\nNon local: %s\nTasks to complete: %s\n', alternative, nonLocalTasks, tasksToComplete)

        [chromosome, done] = self.init_population(alternative, tasksToComplete, nonLocalTasks, tree)
        if done:
            if chromosome is None:
                return None
            else:
                chromosome.evaluate(tree, sim, scheduleStartTime)
                logger.debug('Best solution =1:\n{}'.format(chromosome))
                return chromosome.schedule

        self.evaluate_population(tree, tasksToComplete, sim, scheduleStartTime)

        for unit in self.population:
            logger.debug('Initial population:\n%s', unit)

        if self.populationSize < 20:
            self.bestSolution = self.population[self.fitness.index(max(self.fitness))]
            logger.debug('Best solution <20:\n{}'.format(self.bestSolution))
            return self.bestSolution.schedule

        iteration = 0
        iterWithoutChange = 0
        previousMaxFitness = -1

        while iteration < noIter:
            logger.debug('Iteration: %s', iteration)

            if self.bestSolution is not None:
                previousMaxFitness = self.bestSolution.fitness

            self.bestSolution = self.population[self.fitness.index(max(self.fitness))]

            if previousMaxFitness == self.bestSolution.fitness:
                iterWithoutChange += 1
            else:
                iterWithoutChange = 0

            if iterWithoutChange == 5:
                logger.debug('Best solution =5:\n{}'.format(self.bestSolution))
                return self.bestSolution.schedule

            nextGen = []
            nextGenID = []
            nextGenRating = []

            self.promote_elite(nextGen, nextGenID, nextGenRating)
            self.eliminate_the_worst()

            # logger.debugs('\tCreate new generation')
            self.create_new_generation(nextGen, nextGenID, nextGenRating, tree, tasksToComplete, sim, scheduleStartTime)

            self.population = [x for x in nextGen]
            self.populationID = [x for x in nextGenID]
            self.rating = [x for x in nextGenRating]

            self.calc_fitness()

            iteration += 1



    def init_population(self, alternative, tasksToComplete, nonLocalTasks, tree):
        """
        Create initial population of feasible solutions to the scheduling problem.

        Args:
            alternative (list[str]): Labels of unordered methods (from taems task structure).
            tasksToComplete
            nonLocalTasks (list[str]): Labels of non local tasks (from taems task structure).
            tree (TaemsTree): Taems tree that defines the task structure.
        """
        logger.debugs('Create initial population.')

        self.population = []
        self.initialDurationEV = {}
        for method in alternative:
            self.initialDurationEV[method] = tree.tasks[method].DurationEV

        sim = simulator.LightSimulator(tree)

        feasibleBasePermutations = []
        numberOfPermutations = 5  # WUT? Zasto 5?
        i = 0
        onlyBase = False

        scheduleDone = False
        scheduleAll = True  # TODO: make it parametric or find a better way
        maxIteration = 100
        j = 0
        while i < numberOfPermutations:
            result = self.create_feasible_base_order(alternative, nonLocalTasks, tasksToComplete, sim)
            [feasibleOrder, noOtherPermutations, scheduleDone] = result

            # TODO: find a better solution and remove this
            if scheduleAll:
                if j >= maxIteration:
                    if not feasibleBasePermutations:
                        logger.error('No feasible base order found!')
                    break
                if set(alternative) != set(feasibleOrder):
                    logger.debugs('Elements missing from feasible order: %s', set(alternative) - set(feasibleOrder))
                    pass
                else:
                    feasibleBasePermutations.append(feasibleOrder)
                    onlyBase = True
                    i += 1
                j += 1
                sim.setup()
                continue

            feasibleBasePermutations.append(feasibleOrder)
            # logger.debugs('Permutation: %s, Base order:\n%s', i, feasibleOrder)

            if len(feasibleBasePermutations[-1]) == 0:
                feasibleBasePermutations.pop()
                break

            if scheduleDone or noOtherPermutations:
                break

            if i == 0:
                if len(feasibleBasePermutations[-1]) == len(alternative):
                    logger.debugs('Len of base is equal to the len of alternative.')
                    # WUT? Zasto 20? Zasto ne vise?
                    #       OK. Znaci ako je duljina permutacije baze jednaka duljini alternative, onda su svi taskovi
                    #       dio baze. Ali zasto ogranicit na 20? Sto ako imamo puno metoda i zbog toga puno kombinacija?
                    #       Onda se najbolja kombinacija mozda ni ne pojavi, a nece se pojaviti ni u algoritmu jer se
                    #       uopce ne pokrece ako je populacija manja od 20. Nema nikakvog smisla.
                    numberOfPermutations = 20
                    self.populationSize = 20
                    onlyBase = True
            # Method CreateFeasibleBaseOrder changes simulators attributes so it is necessary to set it up again.
            sim.setup()
            i += 1

        if scheduleDone:
            if len(feasibleBasePermutations[-1]) == 0:
                return [None, True]
            chromosome = Chromosome(feasibleBasePermutations[-1], range(len(alternative)), self.initialDurationEV)
            chromosome.calc_id()
            return [chromosome, True]

        if len(feasibleBasePermutations) == 0:
            maxPopSize = factorial(len(alternative))
        else:
            maxPopSize = factorial(len(alternative)) / factorial(len(feasibleBasePermutations[-1]))

        for base in feasibleBasePermutations:
            logger.debugs('Base permutation: %s', base)

        logger.debug('Max pop size: %s, Len(alternative): %s, Len(base): %s',
                     maxPopSize, len(alternative), len(feasibleBasePermutations[-1]))

        if self.populationSize > maxPopSize:
            self.populationSize = maxPopSize

        while len(self.population) < self.populationSize:
            if onlyBase:
                if len(feasibleBasePermutations) == 0:
                    self.populationSize = len(self.population)
                    return [None, False]
                methods = feasibleBasePermutations.pop()
                baseIndex = range(len(methods))
            else:
                if len(feasibleBasePermutations) == 0:
                    base = []
                else:
                    base = random.choice(feasibleBasePermutations)
                [methods, baseIndex] = self.create_solution_from_base(base, alternative)

            chromosome = Chromosome(methods, baseIndex, self.initialDurationEV)
            chromosome.calc_id()

            if chromosome.ID in self.populationID:
                continue

            self.population.append(chromosome)
            self.populationID.append(chromosome.ID)

        return [None, False]

    def create_feasible_base_order(self, alternative, nonLocalTasks, tasksToComplete, sim):
        """
        Create a feasible order of hard-constrained methods in alternative (base of schedule).

        Args:
            alternative (list[str]): Labels of unordered methods (from taems task structure).
            nonLocalTasks (list[str]): Labels of non local tasks (from taems task structure).
            tasksToComplete
            sim (simulator.LightSimulator): Simulator used to simulate effects of activation of hard constraints.

        Returns:
            feasibleOrder, noOtherPermutations, scheduleDone
        """
        logger.debugs('\nCreate feasible base order.')

        toComplete = list(set(alternative) | set(nonLocalTasks))
        sim.init_disablements(toComplete, tasksToComplete)
        sim.execute_alternative(nonLocalTasks, tasksToComplete)  # WUT? zasto je ovo zakomentirano?
                                                                #    Ne moze se napraviti raspored jer nelokalne metode o kojima ovise lokalne metode nisu izvrsene
        logger.debugs('Disabled tasks: %s', sim.disabledMethods)

        hardConstrained = list(sim.hardConstrainedTasks)

        logger.debugs('\nHard constrained tasks: %s', hardConstrained)

        # No tasks are hard constrained so base is not needed.
        if len(hardConstrained) == 0:
            return [[], False, False]

        temp = deepcopy(hardConstrained)
        feasibleBaseOrder = []

        numberOfOptions = 0
        while True:
            possibleNext = sim.get_enabled_tasks(temp)
            logger.debugs('Possible next: %s', possibleNext)
            if len(set(possibleNext) & set(toComplete)) == 0:
                break
            nonLocalNext = set(nonLocalTasks) & set(possibleNext)
            possibleNext = set(alternative) & set(possibleNext)
            logger.debugs('Possible next: %s', possibleNext)
            logger.debugs('Nonlocal next: %s', nonLocalNext)

            sim.execute_alternative(nonLocalNext, tasksToComplete)
            temp = list(set(temp) - set(nonLocalNext))
            numberOfOptions += len(possibleNext)
            logger.debugs('Number of options: %s', numberOfOptions)

            # temp holds all hard constrained tasks except the non-local ones.
            # Take one random task from possible ones and add it to the base order.
            # Remove it from temp, when temp is empty, we're done.
            if len(possibleNext) > 0:
                feasibleBaseOrder.append(random.sample(possibleNext, 1)[0])
                logger.debugs('Added: %s\n', feasibleBaseOrder[-1])
                sim.execute_task(feasibleBaseOrder[-1], tasksToComplete)
                temp.remove(feasibleBaseOrder[-1])
                if len(temp) == 0:
                    break

        logger.debugs('Number of options: %s', numberOfOptions)
        logger.debugs('Feasible base order: %s\n', feasibleBaseOrder)
        if numberOfOptions == len(alternative):
            return [feasibleBaseOrder, True, True]
        # WUT? numberOfOptions se povecava samo s mogucim taskovima koji su u alternativama, a hardconstrained ima i
        #   nelokalne taskove. Nije li to onda usporedivanje kruska i jabuka?
        if numberOfOptions == len(hardConstrained):
            return [feasibleBaseOrder, True, False]
        elif numberOfOptions == 0:
            return [[], True, False]
        else:
            return [feasibleBaseOrder, False, False]

    def create_solution_from_base(self, base, alternative):
        """
        Create solution from ordered list of base methods.

        First, select indices for base methods and put them into selected positions.
        Then all other methods are permuted in random order and put in available places in the schedule.

        Args:
            base (list[str]): Labels of ordered base methods.
            alternative (list[str]): Labels of unordered methods (from taems task structure).
        """
        schedule = [''] * len(alternative)
        baseIndex = []
        x = -1

        # 1. position base
        temp = deepcopy(base)
        for i in range(len(base)):
            if x == -1:
                x = random.randint(0, len(alternative) - len(temp))
            else:
                x = random.randint(x + 1, len(alternative) - len(temp))

            schedule[x] = base[i]
            baseIndex.append(x)
            temp.remove(base[i])

            if len(alternative) - x - 1 == len(temp):
                schedule[x + 1:] = temp[:]
                baseIndex.extend(range(x + 1, len(schedule)))
                break

        if len(schedule) == len(base):
            return [schedule, baseIndex]

        # 2. put other tasks into schedule
        permutation = GeneticAlgorithm.permutation(list(set(alternative) - set(base)))

        for i in range(len(schedule)):
            if schedule[i] == "":
                schedule[i] = permutation.pop(0)
                if len(permutation) == 0:
                    break

        return [schedule, baseIndex]

    @staticmethod
    def permutation(array):
        """
        Create random permutation from a array.

        Args:
            array (list): Elements to permute.
        """
        perm = []
        temp = deepcopy(array)

        while len(temp) > 0:
            perm.append(temp.pop(random.randint(0, len(temp)-1)))

        return perm

    def evaluate_population(self, tree, tasksToComplete, sim, startTime):
        """
        Evaluate population by calculating its rating and fitness.

        Args:
            tree (TaemsTree): Taems tree that defines the task structure.
            tasksToComplete
            sim (simulator.LightSimulator): Simulator used to simulate effects of activation of hard constraints.
            startTime (float): Mission schedule start time.
        """
        self.populationID = []
        self.rating = []

        for chromosome in self.population:
            self.rating.append(chromosome.evaluate(tree, sim, startTime))
            self.populationID.append(chromosome.ID)

        self.calc_fitness()

    def calc_fitness(self):
        """
        Calculate fitness of every chromosome in population.

        Combine previously calculated chromosome rating with total duration of every chromosomes schedule.
        """
        self.fitness = []
        maxScheduleDuration = -1

        for chromosome in self.population:
            if chromosome.scheduleDuration > maxScheduleDuration:
                maxScheduleDuration = chromosome.scheduleDuration

        for chromosome in self.population:
            string = str(int(maxScheduleDuration - chromosome.scheduleDuration))
            string = string.zfill(8)
            chromosome.fitness = float(str(int(chromosome.rating * 10000))+ "." + string)
            self.fitness.append(chromosome.fitness)

    def promote_elite(self, nextGen, nextGenID, nextGenRating):
        """
        Put elitePercentage of best chromosomes in population into next generation.

        Passed arguments are empty list which get partially populated in this method.

        Args:
            nextGen (list[Chromosome]): Chromosomes in next generation.
            nextGenID (list[str]): Chromosome IDs in next generation.
            nextGenRating (list[float]): Ratings of chromosomes in next generation.
        """
        eliteNumber = int(floor(self.populationSize * self.elitePercentage))
        temp = deepcopy(self.fitness)

        while eliteNumber > 0:
            x = temp.index(max(temp))
            temp[x] = -1

            nextGen.append(self.population[x])
            nextGenID.append(self.populationID[x])
            nextGenRating.append(self.rating[x])

            eliteNumber -= 1

    def eliminate_the_worst(self):
        """Remove worstPercentage of chromosomes with lowest fitness from current population."""
        worstNumber = int(floor(self.populationSize * self.worstPercentage))

        removed = 0
        while removed < worstNumber:
            x = self.fitness.index(min(self.fitness))

            self.fitness.pop(x)
            self.population.pop(x)
            self.populationID.pop(x)

            removed += 1

    def create_new_generation(self, nextGen, nextGenID, nextGenRating, tree, tasksToComplete, sim, startTime):
        """
        Populate next generation with new chromosomes.

        Chromosomes are created by applying genetic operators to parents from current population.

        Args:
            nextGen (list[Chromosome]): Chromosomes in next generation.
            nextGenID (list[str]): Chromosome IDs in next generation.
            nextGenRating (list[float]): Ratings of chromosomes in next generation.
            tree (TaemsTree): Taems tree that defines the task structure.
            tasksToComplete
            sim (simulator.LightSimulator): Simulator used to simulate effects of activation of hard constraints.
            startTime (float): Mission schedule start time.
        """
        while len(nextGen) < self.populationSize:
            # logger.debugs('\t\tCurrent pop size: %s', len(nextGen))
            # Select a parent.
            parents = [random.randint(0, len(self.population) - 1)]

            if random.random() < 0.5:
                # Mutation.
                # logger.debugs('\t\t\tMutation parents: %s', parents)
                if random.random() < 0.5:
                    children = self.mutation_order_based(parents[0], tree)
                else:
                    children = self.mutation_position_based(parents[0], tree)
                # logger.debugs('\t\t\tMutation children %s', children)
            else:
                # Crossover.
                # Select another parent.
                parents.append(random.randint(0, len(self.population) - 1))
                while parents[1] == parents[0]:
                    parents[1] = random.randint(0, len(self.population) - 1)

                # logger.debugs('\t\t\tMutation parents: %s', parents)
                if random.random() < 0.5:
                    children = self.crossover_position_based(parents, tree)
                else:
                    children = self.crossover_order_based(parents, tree)
                # logger.debugs('\t\t\tCrossover children %s', children)

            for child in children:
                child.evaluate(tree, sim, startTime)
                nextGen.append(child)
                nextGenID.append(child.ID)
                nextGenRating.append(child.rating)

    def crossover_position_based(self, parents, tree):
        """
        Perform position based crossover.

        Children get order of (all) methods from one parent and base method positions from other parent.
        It only changes the position of base methods.

        Args:
            parents (list[int]): Indices of two chromosomes from population.
            tree (TaemsTree): Taems tree that defines the task structure.
        """
        p1 = self.population[parents[0]]
        p2 = self.population[parents[1]]

        child1 = Chromosome([''] * len(p1.genes), p2.base, self.initialDurationEV)
        child2 = Chromosome([''] * len(p2.genes), p1.base, self.initialDurationEV)

        # Set up base.
        for i in range(len(child1.base)):
            child2.genes[child2.base[i]] = p2.genes[p2.base[i]]
            child1.genes[child1.base[i]] = p1.genes[p1.base[i]]

        i1 = 0
        i2 = 0
        for i in range(len(p1.genes)):
            if len(child2.genes[i]) == 0:
                while i2 in p2.base:
                    i2 += 1
                child2.genes[i] = p2.genes[i2]
                i2 += 1

            if len(child1.genes[i]) == 0:
                while i1 in p1.base:
                    i1 += 1
                child1.genes[i] = p1.genes[i1]
                i1 += 1

        child1.calc_id()
        child2.calc_id()

        result = []
        if child1.ID not in self.populationID:
            result.append(child1)

        if child2.ID not in self.populationID:
            result.append(child2)

        return result

    def crossover_order_based(self, parents, tree):
        """
        Perform position based crossover.

        Children get base method positions from one parent and base method order form other.
        It doesn't change the position of base methods, the only change is order of base.

        Args:
            parents (list[int]): Indices of two chromosomes from population.
            tree (TaemsTree): Taems tree that defines the task structure.
        """
        p1 = self.population[parents[0]]
        p2 = self.population[parents[1]]

        child1 = Chromosome([''] * len(p1.genes), p1.base, self.initialDurationEV)
        child2 = Chromosome([''] * len(p2.genes), p2.base, self.initialDurationEV)

        # set up base
        for i in range(len(child1.base)):
            child2.genes[child2.base[i]] = p1.genes[p1.base[i]]
            child1.genes[child1.base[i]] = p2.genes[p2.base[i]]

        i1 = 0
        i2 = 0
        for i in range(len(p1.genes)):
            if len(child2.genes[i]) == 0:
                while i1 in p1.base:
                    i1 += 1
                child2.genes[i] = p1.genes[i1]
                i1 += 1

            if len(child1.genes[i]) == 0:
                while i2 in p2.base:
                    i2 += 1
                child1.genes[i] = p2.genes[i2]
                i2 += 1

        child1.calc_id()
        child2.calc_id()

        result = []
        if child1.ID not in self.populationID:
            result.append(child1)

        if child2.ID not in self.populationID:
            result.append(child2)

        return result

    def mutation_order_based(self, parent, tree):
        """
        Perform order based mutation.

        Leave base positions as they are in a parent and randomly permute non-base methods.

        Args:
            parent (int): Index of parent chromosome from population.
            tree (TaemsTree): Taems tree that defines the task structure.
        """
        p = self.population[parent]
        nonBase = list(set(p.genes) - set(p.get_base()))

        child = Chromosome([''] * len(p.genes), p.base, self.initialDurationEV)

        for i in range(len(child.base)):
            child.genes[child.base[i]] = p.genes[p.base[i]]

        nonBase = GeneticAlgorithm.permutation(nonBase)
        i1 = 0
        for i in range(len(p.genes)):
            if len(child.genes[i]) == 0:
                child.genes[i] = nonBase[i1]
                i1 += 1

        child.calc_id()

        if child.ID not in self.populationID:
            return [child]

        return []

    def mutation_position_based(self, parent, tree):
        """
        Perform position based mutation.

        Order of methods stays as it is in a parent, new positions of base elements are randomly chosen.

        Args:
            parent (int): Index of parent chromosome from population.
            tree (TaemsTree): Taems tree that defines the task structure.
        """
        p = self.population[parent]

        base = random.sample(range(len(p.genes)), len(p.base))
        base.sort()
        child = Chromosome([''] * len(p.genes), base, self.initialDurationEV)

        for i in range(len(child.base)):
            child.genes[child.base[i]] = p.genes[p.base[i]]

        i1 = 0
        for i in range(len(p.genes)):
            if len(child.genes[i]) == 0:
                while i1 in p.base:
                    i1 += 1
                child.genes[i] = p.genes[i1]
                i1 += 1

        child.calc_id()

        if child.ID not in self.populationID:
            return [child]

        return []


class Chromosome(object):
    """

    Attributes:
        genes (list[str]): An ordered list of methods.
        base (list): Positions of base methods.
        rating (float): Chromosome's rating (for explanation see GeneticAlgorithm description).
        fitness (float): Chromosome's fitness (for explanation see GeneticAlgorithm description).
        durationEV (dict[str,float]): {Key = method label, Value = expected value of duration}.
        schedule (list[list]): Schedule in form of [[method1 label, method1 start time, method1 end time], ...  ]
        ID (str): Chromosome's ID - unique identifier that is calculated from order of methods in a genome.
        scheduleDuration (float): Expected duration of schedule.
    """

    def __init__(self, genes, base, initialDurationEV):
        """
        Initialize instance variables.

        Args:
             genes (list[str]): An ordered list of methods.
             base (list): Positions of base methods.
             initialDurationEV (dict[str,float]): Initial expected value for every method's duration.
        """
        self.genes = deepcopy(genes)
        self.base = deepcopy(base)
        self.rating = 0
        self.fitness = 0
        self.durationEV = deepcopy(initialDurationEV)
        self.schedule = []
        self.ID = None
        self.scheduleDuration = 0

    def __str__(self):
        if self.schedule:
            self.schedule = [[x[0], round(x[1], 3), round(x[2], 3)] for x in self.schedule]
            for x in self.schedule:
                x[1:3] = [round(x[1], 3), round(x[2], 3)]
            return 'Chromosome: {s.ID:.5}...  Rating: {s.rating:.2f}  Fitness: {s.fitness:.8f}\n' \
                   'Schedule: {s.schedule}'.format(s=self)
        else:
            return 'Chromosome: {s.ID:.5}...  Rating: {s.rating:.2f}  Fitness: {s.fitness:.8f}\n' \
                   'Genes: {s.genes}'.format(s=self)

    def __repr__(self):
        return 'Chromosome {s.ID:.5}... R: {s.rating:.2f}, F: {s.fitness:.8f}'.format(s=self)

    def evaluate(self, tree, sim, time):
        """
        Create a schedule from ordered list of methods and calculate its rating.

        Ordered list of methods is located in self.genes.

        Args:
            tree (TaemsTree): Task structure in form of taems tree.
            sim (simulator.LightSimulator):
            time (float): Mission schedule start time.
        """
        self.calc_id()
        self.completedTasks = []

        startTime = [time]
        noSoftIRActivated = [0, 0]  # [facilitates, hinders]  # TODO: named tuple?
        noSoftIR = [0, 0]
        self.schedule = []
        deadlines = [0, 0]  # [total number of deadlines, number of broken deadlines]
        earliestStartTimes = [0, 0]  # [total number of earliest start times, number of broken est]

        sim.setup()
        sim.execute_alternative(self.genes, self.genes)

        for ir in tree.IRs.values():
            if ir.From in sim.completedTasks and ir.To in sim.completedTasks:
                if ir.type not in [Interrelationship.IR_types['facilitates'], Interrelationship.IR_types['hinders']]:
                    continue
                noSoftIR[ir.type - 2] += 1
                # TESTME: activating soft IRs
                if sim.completedTasks.index(ir.From) < sim.completedTasks.index(ir.To):
                    noSoftIRActivated[ir.type - 2] += 1
                    if ir.type == Interrelationship.IR_types['facilitates']:
                        self.durationEV[ir.To] /= ir.d_powerEV
                    if ir.type == Interrelationship.IR_types['hinders']:
                        self.durationEV[ir.To] *= ir.d_powerEV

        for method in self.genes:
            if tree.tasks[method].earliestStartTime is not None:
                diff = tree.tasks[method].earliestStartTime - startTime[-1]
                # insert slack
                if diff > 0:
                    startTime.append(startTime[-1] + diff)
                    self.schedule.append(["slack", startTime[-2], startTime[-1]])

            if tree.tasks[method].deadline is not None:
                deadlines[0] += 1
                if startTime[-1] + tree.tasks[method].DurationEV - tree.tasks[method].deadline > 0:
                    deadlines[1] += 1

            if tree.tasks[method].earliestStartTime is not None:
                earliestStartTimes[0] += 1
                if tree.tasks[method].earliestStartTime - startTime[-1] > 0:
                    earliestStartTimes[1] += 1

            startTime.append(startTime[-1] + self.durationEV[method])
            self.schedule.append([method, startTime[-2], startTime[-1]])

        self.scheduleDuration = self.schedule[-1][2]

        # Calculate chromosome's fitness.
        # Punish broken deadlines.
        rating2 = 1
        if deadlines[0] > 0:
            rating2 = 1.01 - deadlines[1] / float(deadlines[0])

        # Punish broken earliest start times.
        rating3 = 1
        if earliestStartTimes[0] > 0:
            rating3 = 1.01 - earliestStartTimes[1] / float(earliestStartTimes[0])

        # Reward activated positive soft IRs.
        rating4 = 0
        if noSoftIR[0] > 0:
            rating4 = noSoftIRActivated[0] / float(noSoftIR[0])

        # Punish activated negative soft IRs.
        rating5 = 0
        if noSoftIR[0] > 0:
            rating5 = - noSoftIRActivated[1] / float(noSoftIR[1])

        totalRating = 100 * rating2 * rating3 * (1 + rating4 + rating5)
        self.rating = totalRating
        # logger.debugs(str(self))
        return totalRating

    def calc_id(self):
        """Calculate chromosome's ID."""
        string = ''.join(self.genes)
        self.ID = md5(string).hexdigest()

    def get_base(self):
        """Return chromosome's base methods - elements on self.base positions."""
        baseMethods = []
        for x in self.base:
            baseMethods.append(self.genes[x])

        return baseMethods
