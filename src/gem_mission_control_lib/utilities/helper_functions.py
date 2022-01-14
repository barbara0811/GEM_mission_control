__author__ = 'barbanas'

import itertools
from sys import maxint
import numpy as np


def cartesianProductOfDistributions(listOfDistributions, function):
    '''
        listOfDistributions = [D1, D2, ...]
        Di = {}  -> dictionary key = variable value, value = probability
        function -> sum, max or min
        '''
    product = {}

    KeyList = []
    for i in listOfDistributions:
        KeyList.append(i.keys())

    keys = cartesianProduct(KeyList, False, False)

    for key in keys:
        prob = 1
        if function == 'sum':
            value = 0
        elif function == 'max':
            value = -maxint
        elif function == 'min':
            value = maxint

        for i in range(len(key)):
            x = key[i]
            prob *= listOfDistributions[i][x]

            if function == 'sum':
                value += x
            elif function == 'max':
                if x > value:
                    value = x
            elif function == 'min':
                if x < value:
                    value = x

        if value not in product.keys():
            product[value] = prob
        else:
            product[value] += prob

    return product


def mutiplyDistribution(distribution, value):
    for key in distribution.keys():
        new_key = key * value
        distribution[new_key] = distribution.pop(key)


def cartesianProduct(setList, removeDuplicates, sort):
    temp = []

    i = 0
    for element in itertools.product(*setList):
        temp.append([])

        for e in element:
            '''
                if type(e) is list:
                    temp[i].extend(e)
                else:
                '''
            temp[i].append(e)

        if removeDuplicates:
            temp[i] = list(set(temp[i]))

        if sort:
            temp[i].sort()

        i += 1

    return temp


def calcExpectedValue(distribution):

    i = np.array(distribution.keys())
    ev = np.array(distribution.values())

    return sum(i*ev)


def probXSmallerThanVal(distribution, val):
    prob = 0

    for x in distribution.keys():
        if float(x) < val:
            prob += distribution[x]

    return prob


def probXGreaterThanVal(distribution, val):
    prob = 0

    for x in distribution.keys():
        if float(x) > val:
            prob += distribution[x]

    return prob


def OR(setList):
    temp = []

    for i in range(len(setList)):
        for item in setList[i]:
            temp.append(item)

    return temp


def powerSet(setList):
    for subset in itertools.chain.from_iterable(itertools.combinations(setList, r) for r in range(len(setList) + 1)):
        if len(subset) >= 1:
            yield list(subset)


def removeDuplicates(setList):
    retValue = []

    for item in setList:
        if item not in retValue:
            retValue.append(item)

    return retValue
