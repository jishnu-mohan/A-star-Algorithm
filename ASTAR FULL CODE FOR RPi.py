import RPi.GPIO as GPIO
GPIO.setwarnings(FALSE)
import time
L1 = 35
L2 = 37
R1 = 36
R2 = 38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(L1, GPIO.OUT)
GPIO.setup(L2, GPIO.OUT)
GPIO.setup(R1, GPIO.OUT)
GPIO.setup(R2, GPIO.OUT)

turndelay = 1
halftdelay = turndelay / 2
revdelay = turndelay * 2
fordelay = 1
dfordelay = 1.4

class AStarGraph(object):
    # Define a class board like grid with two barriers

    def __init__(self):
        self.barriers = []
        self.barriers.append \
            ([(2 ,4) ,(2 ,5) ,(2 ,6) ,(3 ,6) ,(4 ,6) ,(5 ,6) ,(5 ,5) ,(5 ,4) ,(5 ,3) ,(5 ,2) ,(4 ,2) ,(3 ,2)])

    def heuristic(self, start, goal):
        # Use Chebyshev distance heuristic if we can move one square either
        # adjacent or diagonal
        D = 1
        D2 = 1
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def get_vertex_neighbours(self, pos):
        n = []
        # Moves allow link a chess king
        for dx, dy in [(1 ,0) ,(-1 ,0) ,(0 ,1) ,(0 ,-1) ,(1 ,1) ,(-1 ,1) ,(1 ,-1) ,(-1 ,-1)]:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
                continue
            n.append((x2, y2))
        return n

    def move_cost(self, a, b):
        for barrier in self.barriers:
            if b in barrier:
                return 100  # Extremely high cost to enter barrier squares
        return 1  # Normal movement cost

def AStarSearch(start, end, graph):

    G = {}  # Actual movement cost to each position from the start position
    F = {}  # Estimated movement cost of start to end going via this position

    # Initialize starting values
    G[start] = 0
    F[start] = graph.heuristic(start, end)

    closedVertices = set()
    openVertices = set([start])
    cameFrom = {}

    while len(openVertices) > 0:
        # Get the vertex in the open list with the lowest F score
        current = None
        currentFscore = None
        for pos in openVertices:
            if current is None or F[pos] < currentFscore:
                currentFscore = F[pos]
                current = pos

        # Check if we have reached the goal
        if current == end:
            # Retrace our route backward
            path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(current)
            path.reverse()
            return path, F[end]  # Done!

        # Mark the current vertex as closed
        openVertices.remove(current)
        closedVertices.add(current)

        # Update scores for vertices near the current position
        for neighbour in graph.get_vertex_neighbours(current):
            if neighbour in closedVertices:
                continue  # We have already processed this node exhaustively
            candidateG = G[current] + graph.move_cost(current, neighbour)

            if neighbour not in openVertices:
                openVertices.add(neighbour)  # Discovered a new vertex
            elif candidateG >= G[neighbour]:
                continue  # This G score is worse than previously found

            # Adopt this G score
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = graph.heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H

    raise RuntimeError("A* failed to find a solution")

def TraversePath(result,rd):
    global turndelay,fordelay,revdelay
    x,y = result[0]
    for i in range(len(result)-1):
        nx ,ny = result[i+1]

        if rd == 1:        #East
            if nx == x+1 and ny == y:    # East
                Forward(fordelay)
                rd = 1
            elif nx == x-1and ny == y:     #West
                TurnRight(revdelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnLeft(turndelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnLeft(turndelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnLeft(halftdelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnRight(halftdelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1 :      #SW
                TurnRight(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnLeft(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 2:        #West
            if nx == x+1 and ny == y:    # East
                TurnRight(revdelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnRight(turndelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnLeft(turndelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnRight(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnLeft(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                TurnLeft(halftdelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnRight(halftdelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 3:        #North
            if nx == x+1 and ny == y:    # East
                TurnRight(turndelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                TurnLeft(turndelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnRight(revdelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnRight(halftdelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnRight(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                TurnLeft(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnLeft(halftdelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 4:        #South
            if nx == x+1 and ny == y:    # East
                TurnLeft(turndelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                TurnRight(turndelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnRight(revdelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnLeft(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnLeft(halftdelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                TurnRight(halftdelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnRight(turndelay+halftdelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 5:        #NE
            if nx == x+1 and ny == y:    # East
                TurnRight(halftdelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                TurnLeft(turndelay+halftdelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnLeft(halftdelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnRight(turndelay+halftdelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnRight(halftdelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                TurnRight(revdelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnLeft(turndelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 6:        #SE
            if nx == x+1 and ny == y:    # East
                TurnLeft(halftdelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                TurnRight(turndelay+halftdelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnLeft(turndelay+halftdelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnRight(halftdelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnLeft(turndelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                TurnRight(turndelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnRight(revdelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 7:        #SW
            if nx == x+1 and ny == y:    # East
                TurnLeft(turndelay+halftdelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                TurnRight(halftdelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnRight(turndelay+halftdelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnLeft(halftdelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnRight(revdelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnLeft(turndelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                TurnRight(turndelay)
                Forward(dfordelay)
                rd = 8

        elif rd == 8:        #NW
            if nx == x+1 and ny == y:    # East
                TurnRight(turndelay+halftdelay)
                Forward(fordelay)
                rd = 1
            elif nx == x-1 and ny == y:     #West
                TurnLeft(halftdelay)
                Forward(fordelay)
                rd = 2
            elif nx ==x and ny == y+1:    #North
                TurnRight(halftdelay)
                Forward(fordelay)
                rd = 3
            elif nx == x and ny == y-1:     #South
                TurnLeft(turndelay+halftdelay)
                Forward(fordelay)
                rd = 4
            elif nx == x+1 and ny == y+1:    #NE
                TurnRight(turndelay)
                Forward(dfordelay)
                rd = 5
            elif nx == x+1 and ny == y-1:      #SE
                TurnRight(revdelay)
                Forward(dfordelay)
                rd = 6
            elif nx == x-1 and ny == y-1:      #SW
                TurnLeft(turndelay)
                Forward(dfordelay)
                rd = 7
            elif nx == x-1 and ny == y+1:    #NW
                Forward(dfordelay)
                rd = 8



def TurnRight(delay):
    GPIO.output(L1, GPIO.HIGH)
    GPIO.output(L2, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(L1, GPIO.LOW)
    GPIO.output(L2, GPIO.LOW)
    #make pins high and stop after delay
def TurnLeft(delay):
    GPIO.output(R1, GPIO.HIGH)
    GPIO.output(R2, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(R1, GPIO.LOW)
    GPIO.output(R2, GPIO.LOW)
def Forward(delay):
    GPIO.output(L1, GPIO.HIGH)
    GPIO.output(L2, GPIO.HIGH)
    GPIO.output(R1, GPIO.HIGH)
    GPIO.output(R2, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(L1, GPIO.LOW)
    GPIO.output(L2, GPIO.LOW)
    GPIO.output(R1, GPIO.LOW)
    GPIO.output(R2, GPIO.LOW) 


if __name__== "__main__":
    rd = 3          #1=East,2=West,3=North,4=South  5=NE, 6=SE, 7=SW, 8=NW
    graph = AStarGraph()
    result, cost = AStarSearch((0, 0), (7, 7), graph)
   # print("route", result)
   # print("cost", cost)

    # To traverse the path
    TraversePath(result,rd)







