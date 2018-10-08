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
    #Guardem la coordenada inicial del nostre pacman
    estat = problem.getStartState()
    #creem l'Stack que utilitzarem pel nostre dfs
    stateStack = util.Stack()
    #creem una llista buida on guardarem les coordenades ja explorades
    explored = []
    #afegim al nostre Stack la primera posició juntament amb una llista que contindrà
    #els moviments fets fins aquella posició.
    #Així, a partir d'ara tractarem els estats com una llista de dos elements formada
    #per [(coordenada),[llista de moviments fins la coordenada]]
    stateStack.push([estat,[]])
    #Ara creem un bucle per anar explorant els estats. Degut a que volem fer un dfs,
    #seguirem el model LIFO (last in, first out) per aconseguir-ho
    while not stateStack.isEmpty():
    	#Extraiem l'últim estat afegit a l'Stack
        current = stateStack.pop()
        #Si aquest és l'objectiu, retornem la llista de moviments que ens han dut a ell.
        if(problem.isGoalState(current[0])):
            return current[1]
        #Comprovem que aquesta coordenada no l'hem explorat ja
        if current[0] not in explored:
        	#Si no ha estat explorat, obtenim (si en té) els successors d'aquest i els
        	#afegim al nostre Stack juntament amb el nou moviment
            for i in problem.getSuccessors(current[0]):
                    stateStack.push([i[0],current[1]+[i[1]]])
        #Afegim la coordenada a explorats
        explored.append(current[0])
        

def breadthFirstSearch(problem):
    #Guardem la coordenada inicial del nostre pacman
    estat = problem.getStartState()
    #creem la cua que utilitzarem pel nostre bfs
    stateQueue = util.Queue()
    #creem una llista buida on guardarem les coordenades ja explorades
    visited = []
    #afegim a la nostre cua la primera posició juntament amb una llista que contindrà
    #els moviments fets fins aquella posició.
    #Així, a partir d'ara tractarem els estats com una llista de dos elements formada
    #per [(coordenada),[llista de moviments fins la coordenada]]
    stateQueue.push([estat,[]])
    #Ara creem un bucle per anar explorant els estats. Degut a que volem fer un bfs,
    #seguirem el model FIFO (first in, first out) per aconseguir-ho
    while stateQueue:
        #Extraiem el primer estat afegit a la cua. Aquest cop, guardarem per separat
        #la coordenada (current) i els moviments (actions)
        current, actions = stateQueue.pop()
        #Si aquest és l'objectiu, retornem la llista de moviments que ens han dut a ell.
        if problem.isGoalState(current):
            return actions
        #Comprovem que aquesta coordenada no l'hem explorat ja
        if current not in visited:
        	#Si no ha estat explorat, obtenim (si en té) els successors d'aquest i els
        	#afegim al nostre Stack juntament amb el nou moviment
            for successor in problem.getSuccessors(current):
                stateQueue.push([successor[0],actions+[successor[1]]])
            #Afegim la coordenada a explorats        
            visited.append(current)

def uniformCostSearch(problem):
    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    #Guardem la coordenada inicial del nostre pacman
    estat = problem.getStartState()
    #Creem la cua prioritària que utilitzarem
    statePQueue = util.PriorityQueue()
    #Creem una llista buida on afegirem els estats que ja haguem explorat
    explored = []
    #Guardem la heurística de l'estat actual
    h = heuristic(estat,problem)
    #En aquest cas, els estats seran una llista de la forma [coordenada,moviments,cost acumulat]
    #i la prioritat serà h(heurística)+g(cost acumulat).
    #En el primer push, el cost 'g' és 0.
    statePQueue.push([estat,[],0],h)
    
    goal = False
    #Ara creem un bucle per anar explorant els estats de manera que el següent estat a explorar
    #serà el que tingui prioritat més baixa. (per prioritat entenem g+h = cost acumulat+heurística)
    while not statePQueue.isEmpty():
    	#Extraiem el primer estat
        current = statePQueue.pop()
        #Si la coordenada es goal (en el cas dels corners dins de current[0] tambe hi haura una llista
        #de corners visitats) retorna el camí.
        if(problem.isGoalState(current[0]) or goal):
            return current[1]
        #Comprovem no haver explorat l'estat
        if current[0] not in explored:
        	#Afegim cada successor que hagi amb la seva heuristica i cost acumulat
            for i in problem.getSuccessors(current[0]):
                
                hact = heuristic(i[0],problem)
                g = current[2]+i[2]
                statePQueue.push([i[0],current[1]+[i[1]],g],hact+g)
        #Afegim l'estat a la llista d'explorats
        explored.append(current[0])
        

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
