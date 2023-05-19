import random
import math
from collections import deque
import heapq
from collections import defaultdict
import networkx as nx
from queue import PriorityQueue



def setValue(list_of_lists):


    temperature = 90
    decay = 0.5
    maxIter = 100

    JobList = [sublist[0] for sublist in list_of_lists]

    print("GIVEN LIST:", JobList)
    bestRoute, bestCost = SimulatedAnnealing(JobList, list_of_lists, temperature, decay, maxIter)

    print("GENERATED PATH :",bestRoute)
    print("OPTIMAL COST",bestCost)

    # @staticmethod
    # def print_path(traced_path, goal):
    #     print(traced_path)
    #     print(f'Goal node: {goal}')

def CostFinderHelper(tempJobs,tempNest,Duration,Cost):
    first_element = tempJobs[0]
    tempJobs.pop(0)


    for items in tempNest:
        if items[0]==first_element:
            Cost=Duration-items[2]

    for lst in tempNest:
        if first_element in lst:
            tempNest.remove(lst)

    return Cost


# Calculate Total Duration
def Cal_Duration(totalJobs):
    sumVar = 0
    for item in totalJobs:
        sumVar += item[1]
    return sumVar



def CostFinder(toCheckList,tempNestesList):

    sampleTempLists=tempNestesList.copy()
    sampleCheckList=toCheckList.copy()
    Cost=0
    tempCost=0
    while(sampleCheckList):
        TotalDuration = Cal_Duration(sampleTempLists)
        tempCost=CostFinderHelper(sampleCheckList,sampleTempLists,TotalDuration,Cost)
        if tempCost>0:
            Cost+=tempCost

    return Cost


def SimulatedAnnealing(jobsList,NestedList,temp,decay,maxIter):

    random.shuffle(jobsList)
    CurrentCost=CostFinder(jobsList,NestedList)

    bestRoute=[]
    bestCost=CurrentCost


    while(temp>0.001 or maxIter>0):

        new_Jobs_list=jobsList.copy()

        random.shuffle(new_Jobs_list)

        newCost=CostFinder(new_Jobs_list,NestedList)
        if newCost<CurrentCost:
            CurrentCost=newCost
            bestCost=CurrentCost
            bestRoute=new_Jobs_list
        else:
            prob_accept = math.exp((CurrentCost - newCost) / temp)
            if random.random() < prob_accept:
                jobsList = new_Jobs_list.copy()
                CurrentCost = newCost


        maxIter-=1
        temp *= decay
        # print(temp)

    return bestRoute, bestCost





if __name__ == "__main__":
    list_of_lists = [
        ['A', 3, 5],
        ['B', 5, 6],
        ['C', 9, 16],
        ['D', 7, 14]
    ]
    setValue(list_of_lists)




