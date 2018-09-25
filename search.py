# -*- coding: utf-8 -*-
#
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST

    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    
    estat = problem.getStartState()
    
    stateStack = util.Stack()
    
    explored = []
    
    stateStack.push([estat,[]])
    
    while not stateStack.isEmpty():
    
        actual = stateStack.pop()
        
        if(problem.isGoalState(actual[0])):
            return actual[1]
        
        if actual[0] not in explored:
            for i in problem.getSuccessors(actual[0]):
                    stateStack.push([i[0],actual[1]+[i[1]]])
        
        explored.append(actual[0])
        

def breadthFirstSearch(problem):
    
    estat = problem.getStartState()
    print estat
    
    stateQueue = util.Queue()
    
    explored = []
    
    stateQueue.push([estat,[]])
    
    while not stateQueue.isEmpty():
    
        actual = stateQueue.pop()
        
        if(problem.isGoalState(actual[0])):
            return actual[1]
        
        if actual[0] not in explored:
            for i in problem.getSuccessors(actual[0]):
                stateQueue.push([i[0],actual[1]+[i[1]]])
        
        explored.append(actual[0])

def uniformCostSearch(problem):
    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    
    estat = problem.getStartState()
    
    statePQueue = util.PriorityQueue()
    
    explored = []
    
    h = heuristic(estat,problem)
    
    statePQueue.push([estat,[],0],h)
    
    goal = False
    
    while not statePQueue.isEmpty():
    
        actual = statePQueue.pop()
                                      
        if(problem.isGoalState(actual[0]) or goal):
            """
            goal = True
            sol_parcial = actual
            if(not statePQueue.isEmpty()):
                prova = statePQueue.pop()
                if(sol_parcial[2]<prova[2]+heuristic(prova[0],problem)):
                    return sol_parcial[1]
                else:
                    prio = prova[2]+heuristic(prova[0],problem)
                    statePQueue(prova,prio)
            else:
                return sol_parcial[1]
            """
            return actual[1]
        
        if actual[0] not in explored:
            
            for i in problem.getSuccessors(actual[0]):
                
                hact = heuristic(i[0],problem)
                g = actual[2]+i[2]
                statePQueue.push([i[0],actual[1]+[i[1]],g],hact+g)
        
        explored.append(actual[0])
        

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
