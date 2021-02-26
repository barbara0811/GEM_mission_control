#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'barbanas'

import numpy as np
from copy import deepcopy

class BranchAndBoundOptimizer(object):
    '''
    A class that performs branch and bound algorithm
    for task allocation of n tasks among m agents (m >= n).
    '''
    @staticmethod
    def optimize(matrix):
        # adjust matrix to dimension (m,m) by filling missing columns with zeroes
        if matrix.shape[0] > matrix.shape[1]:
            x = np.array([[0]*(matrix.shape[0] - matrix.shape[1])]*matrix.shape[0])
            matrix = np.append(matrix, x, 1)
        elif matrix.shape[1] > matrix.shape[0]:
            x = np.array([[0]* matrix.shape[1]]*(matrix.shape[1] - matrix.shape[0]))
            matrix = np.append(matrix, x, 0)

        root = Node(matrix, np.array([-1]*matrix.shape[0]))
        # calculate upper and lower bound
        root.calc_upper_bound()
        UB = root.upperBound
        diagonalSum = [matrix.trace().item(0), matrix[::-1].trace().item(0)]
        LB = max(diagonalSum)
        # set best feasible solution
        diagonal = diagonalSum.index(LB)
        if diagonal == 0:
            bestSolution = Node(matrix, range(matrix.shape[0]))
        else:
            bestSolution = Node(matrix, range(matrix.shape[0] - 1, -1, -1))

        # check if optimum is on one of matrix diagonals
        if LB == UB:
            return bestSolution.taskAssignment

        queue = root.expand()
        while len(queue) > 0:
            node = queue.pop(0)
            node.calc_upper_bound()
            if node.isFeasible:
                # optimal solution is found
                if node.upperBound == UB:
                    return node.upperBoundAssignment
                elif node.upperBound > LB:
                    # update best feasible solution and LB
                    bestSolution = node
                    LB = node.upperBound
                    # queue is LIFO -> depth first search (new nodes are added to the beginning)
                    expansion = node.expand()
                    expansion.extend(queue)
                    queue = expansion
            else:
                # possible better solution
                if node.upperBound > LB:
                    # queue is LIFO -> depth first search (new nodes are added to the beginning)
                    expansion = node.expand()
                    expansion.extend(queue)
                    queue = expansion

        return bestSolution.upperBoundAssignment


class Node(object):
    '''
    A class that represents a node of branch and bound algorithm.

    Attributes:
        matrix - (agent,task) square matrix, representation of maximization problem
        taskAssignment - a list of tasks assigned to every agent - list indices represent agents
        dimension - dimension n of (n,n) matrix
        assignedTasks - a list of indices representing tasks already assigned to agents
        assignedAgents - a list of indices representing agents that have tasks assigned to them
        upperBound - best possible solution that can be produced from this node (doesn't have to be feasible)
        upperBoundAssignment - task assignment that produces solution with upperBound quality
        isFeasible - a flag that represents task assignment feasibility (feasible if each task is assigned to exactly one agent)
    '''
    def __init__(self, matrix, taskAssignment):
        self.matrix = matrix
        self.taskAssignment = deepcopy(taskAssignment)
        self.dimension = len(taskAssignment)
        self.assignedTasks = self.taskAssignment[self.taskAssignment>=0]
        self.assignedAgents = np.where(self.taskAssignment>=0)[0]
        self.upperBound = 0
        self.upperBoundAssignment = deepcopy(self.taskAssignment)
        self.isFeasible = True

    def expand(self):

        expansion = []
        # a list of agent indices that have no tasks assigned to them
        assignTo = list(set(range(self.dimension)) - set(self.assignedAgents))
        # a list of task indices that have to be assigned
        tasksToAssign = list(set(range(self.dimension)) - set(self.assignedTasks))

        for agent in assignTo:
            temp = deepcopy(self.taskAssignment)
            for item in tasksToAssign:
                temp[agent] = item
                newNode = Node(self.matrix, temp)
                expansion.append(newNode)

        return expansion

    def calc_upper_bound(self):

        if self.upperBound > 0:
            print("[B&B] ERROR: trying to calculate upper bound of a node multiple times!")
            return

        # a list of agent indices that have no tasks assigned to them
        assign_to = list(set(range(self.dimension)) - set(self.assignedAgents))

        for agent in self.assignedAgents:
            self.upperBound += self.matrix[agent, self.taskAssignment[agent]]

        temp = deepcopy(self.matrix)
        x = np.array([-1]*self.dimension)
        x = np.reshape(x, [self.dimension, 1])
        # set rating of all assigned tasks to -1 for all agents
        for column in self.assignedTasks:
            temp[:, column] = x
        # get a task with max rating for every agent
        max_columns = np.argmax(temp, axis=1)

        for agent in assign_to:
            self.upperBound += temp.item((agent, max_columns.item(agent)))
            if max_columns[agent] in self.upperBoundAssignment:
                self.isFeasible = False
            self.upperBoundAssignment[agent] = max_columns[agent]
