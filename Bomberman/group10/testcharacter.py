# This is necessary to find the main code
import operator
import sys

from Bomberman.bomberman.entity import MonsterEntity
from Bomberman.bomberman.sensed_world import SensedWorld

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from queue import PriorityQueue

import math
from enum import Enum

class TestCharacter(CharacterEntity):

    destination = (0, 0)
    expectiDepth = 3
    minimaxDepth = 4
    bound = 4

    def do(self, wrld):
        # Your code here
        loc = (self.x, self.y)
        wrldState = self.evaluateState(wrld)
        characterState = wrldState[0]
        # exit is first destination
        self.destination = wrld.exitcell

        # If the exit is right next to us, just pick it
        if wrld.exitcell in self.getNeighbors(loc, wrld, [obstacles.EXIT, obstacles.PLAYER]):
            move = self.calculateD(loc, wrld.exitcell)
            self.move(move[0], move[1])
            return

        # There is a monster close to us
        if characterState == state.UNSAFE:
            self.place_bomb()

            # running away from stupid
            if wrldState[1][0] == 'stupid':
                v, action = self.maxvalue(wrld, loc, 0, 'stupid')
                next_move = self.calculateD(loc, action)
                self.move(next_move[0], next_move[1])

            # Running away from aggressive
            if wrldState[1][0] == 'aggressive':
                v, action = self.miniMaxvalue(wrld, -math.inf, math.inf, loc, 0, 'aggressive')
                next_move = self.calculateD(loc, action)
                self.move(next_move[0], next_move[1])

            # Running away from selfpreserving
            if wrldState[1][0] == 'selfpreserving':
                v, action = self.miniMaxvalue(wrld, -math.inf, math.inf, loc, 0, 'selfpreserving')
                next_move = self.calculateD(loc, action)
                self.move(next_move[0], next_move[1])

        # What to do when there is a bomb near us
        if characterState == state.NEAR_BOMB:
            next_move = (0, 0)
            max = 0
            name = ''
            flag = True
            if wrldState[1]:
                name = wrldState[1][0]
                flag = False
            if self.bomb_check(loc, wrld):
                for cell in self.getNeighbors(loc, wrld, [obstacles.EXIT]):
                    if not self.bomb_check(cell, wrld):
                        # predict one step ahead
                        next_move = self.calculateD(loc, cell)
                        newWrld = SensedWorld.from_world(wrld)
                        character = next(iter(newWrld.characters.values()))[0]
                        new_move = self.calculateD((character.x, character.y), (cell[0], cell[1]))
                        character.move(new_move[0], new_move[1])
                        if name != '':
                            monster = self.getMonster(newWrld, name)
                            monster.move(0, 0)
                        newerWrld = newWrld.next()[0]
                        if flag:
                            test = self.exit_utility(newerWrld)
                        else:
                            test = self.utility(newerWrld, name)
                        if test > max:
                            max = test
                            next_move = new_move
                self.move(next_move[0], next_move[1])
            else:
                self.move(0, 0)

        # What to do if we cannot currently reach the exit
        if characterState == state.BLOCKED:
            walls = []
            route = []
            reachable = False
            # Map a direct course to the exit, ignoring walls
            came_from, cost_so_far = self.AStar(wrld, loc, wrld.exitcell, [obstacles.EXIT, obstacles.WALL])
            path = wrld.exitcell
            while path != loc:
                path = came_from[path]
                route.append(path)

            # Find all the walls you have to go through
            for stepnum, step in enumerate(route):
                self.set_cell_color(step[0], step[1], Fore.RED + Back.GREEN)
                if wrld.wall_at(step[0], step[1]):
                    walls.append(route[stepnum+1])
            # Choose the closest reachable wall to the exit
            closest_wall = (0,0)
            for wall in (walls):
                new_goal = wall
                came_from, cost_so_far = self.AStar(wrld, loc, new_goal, [obstacles.EXIT])
                for path in came_from:
                    if path == new_goal:
                        closest_wall = new_goal
                        reachable = True
                        break
                if reachable: break

            self.destination = closest_wall
            # Navigate to that location
            came_from, cost_so_far = self.AStar(wrld, loc, closest_wall, [obstacles.EXIT])
            path = closest_wall
            next_m = (0, 0)
            while path != loc:
                temp = path
                path = came_from[path]
                if path == loc:
                    next_m = temp
                    break
            next_move = self.calculateD(loc, next_m)

            # Place bomb at wall -- deal with diagonal!?!
            if loc == closest_wall:
                self.place_bomb()
            else:
                self.move(next_move[0], next_move[1])

        # What to do if there are no monsters near us and we can reach the exit
        if characterState == state.SAFE:
            # Just do A star
            came_from, cost_so_far = self.AStar(wrld, loc, self.destination, [obstacles.EXIT])
            path = self.destination
            next_m = (0, 0)
            while path != loc:
                temp = path
                path = came_from[path]
                if path == loc:
                    next_m = temp
                    break
            next_move = self.calculateD(loc, next_m)

            self.move(next_move[0], next_move[1])

    # Max Value function of expecitmax
    def maxvalue(self, wrld, curr, d, name):
        # Terminal state
        if self.evaluateState(wrld)[0] == state.SAFE or d == self.expectiDepth:
            return self.utility(wrld, name), curr
        if self.evaluateState(wrld)[0] == state.DEAD:
            return -10000, curr
        v = -math.inf
        action = (0, 0)
        for a in self.getNeighbors(curr, wrld, [obstacles.EXIT]):
            # simulate a new world where we make the move
            newWrld = SensedWorld.from_world(wrld)
            character = next(iter(newWrld.characters.values()))[0]
            new_move = self.calculateD((character.x, character.y), (a[0], a[1]))
            character.move(new_move[0], new_move[1])
            monster = self.getMonster(newWrld, name)
            monster.move(0, 0)
            newerWrld = newWrld.next()[0]
            val = self.expvalue(newerWrld, a, d + 1, name)
            if val > v:
                v = val
                action = a
        return v, action

    # Expected Value part of expectimax
    def expvalue(self, wrld, act, d, name):
        if self.evaluateState(wrld)[0] == state.SAFE or d == self.expectiDepth:
            return self.utility(wrld, name)
        v = 0
        mcurr = self.getMonster(wrld, name)
        possible_moves = self.getNeighbors((mcurr.x, mcurr.y), wrld, [obstacles.PLAYER])
        for a in possible_moves:
            p = 1.0/len(possible_moves)
            # Predict a step ahead using simulated world
            newWrld = SensedWorld.from_world(wrld)
            monster = self.getMonster(newWrld, name)
            new_move = self.calculateD((monster.x, monster.y), (a[0], a[1]))
            monster.move(new_move[0], new_move[1])
            try:
                character = next(iter(newWrld.characters.values()))[0]
            except(IndexError, StopIteration):
                return -10000
            character.move(0, 0)
            newerWrld = newWrld.next()[0]
            value = self.maxvalue(newerWrld, act, d+1, name)[0]
            v = v + p*value
        return v

    # Alpha Beta Minimax
    # Max value for Alpha-Beta Pruning
    def miniMaxvalue(self, wrld, alpha, beta, curr, d, name):
        # Terminal State is we are safe or depth reached
        if self.evaluateState(wrld)[0] == state.SAFE or d == self.minimaxDepth:
            return self.utility(wrld, name), curr
        if self.evaluateState(wrld)[0] == state.DEAD:
            return -10000, curr
        v = -math.inf
        action = (0, 0)
        for a in self.getNeighbors(curr, wrld, [obstacles.EXIT, obstacles.PLAYER]):
            # Simulate a new world where we made that action
            newWrld = SensedWorld.from_world(wrld)
            character = next(iter(newWrld.characters.values()))[0]
            new_move = self.calculateD((character.x, character.y), (a[0], a[1]))
            monster = self.getMonster(newWrld, name)
            character.move(new_move[0], new_move[1])
            monster.move(0, 0)
            newerWrld = newWrld.next()[0]
            val = self.minvalue(newerWrld, alpha, beta, a, d+1, name)
            if val > v:
                v = val
                action = a
            if v >= beta:
                return v, a
            alpha = max(alpha, v)
        return v, action

    # Min value for Minimax Alpha-Beta Pruning
    def minvalue(self, wrld, alpha, beta, act, d, name):
        # Terminal State is we are safe or depth reached
        if self.evaluateState(wrld)[0] == state.SAFE or d == self.minimaxDepth:
            return self.utility(wrld, name)
        v = math.inf
        mcurr = self.getMonster(wrld, name)
        possible_moves = self.getNeighbors((mcurr.x, mcurr.y), wrld, [obstacles.PLAYER, obstacles.EXIT, obstacles.MONSTER])
        for a in possible_moves:
            # Simulate a new world where we made that action
            newWrld = SensedWorld.from_world(wrld)
            monster = self.getMonster(newWrld, name)
            new_move = self.calculateD((monster.x, monster.y), (a[0], a[1]))
            monster.move(new_move[0], new_move[1])
            try:
                character = next(iter(newWrld.characters.values()))[0]
            except(IndexError, StopIteration):
                return -10000
            character.move(0, 0)
            newerWrld = newWrld.next()[0]
            val, act = self.miniMaxvalue(newerWrld, alpha, beta, act, d + 1, name)
            v = min(v, val)
            if v <= alpha:
                return v
            beta = min(beta, v)
        return v

    # Main utility function for terminal states
    def utility(self, wrld, name):
        # Utility for stupid monster
        if name == 'stupid':
            return 6*(1/(1 + self.exit_utility(wrld))) - 1*(1/((1 + self.monster_utility(wrld, name))**2))
        # Utility for non-stupid monster
        else:
            return 20 * (1 / (1 + self.exit_utility(wrld))) - 50 * (1 / ((1 + self.monster_utility(wrld, name)) ** 2)) + self.dpangle(wrld, name)

    # Calculate Vector between us, the monster, and the exit
    def dpangle(self, wrld, name):
        try:
            chara = next(iter(wrld.characters.values()))
            character = chara[0]
        except (IndexError, StopIteration):
            return -10
        # Vector for character to exit
        e = self.destination
        loc = (character.x, character.y)
        ce = tuple(map(operator.sub, e, loc))
        eu = self.calculateH(e, loc)

        if ce == (0, 0) or eu == 0:
            return 10000
        # Vector for character to monster
        monster = self.getMonster(wrld, name)
        mu = self.calculateH((monster.x, monster.y), loc)
        cm = tuple(map(operator.sub, (monster.x, monster.y), loc))
        if cm == (0, 0) or mu == 0:
            return -10000
        # Dot product
        dp = (ce[0] * cm[0]) + (ce[1] * cm[1])
        cosangle = dp / (eu * mu)

        try:
             angle = math.degrees(math.acos(cosangle))
        except(ValueError):
            return -10

        if self.exit_utility(wrld) <= 4:
            return 10
        # Return values based on if it is higher or lower than 90 degrees
        if angle >= 90:
            return eu
        else:
            return -mu

    # Gets the monster in the current world with a name
    def getMonster(self, wrld, name):
        for monster in list(wrld.monsters.values()):
            if monster[0].name == name:
                return monster[0]
        return MonsterEntity('dead', [0], 0, 0)

    # Utility function for the distance to the exit
    def exit_utility(self, wrld):
        try:
            chara = next(iter(wrld.characters.values()))
            character = chara[0]
        except (IndexError, StopIteration):
            return 10
        loc = (character.x, character.y)
        e = self.destination
        exit_came_from, exit_cost_so_far = self.AStar(wrld, loc, (e[0], e[1]), [obstacles.EXIT])
        counter = 0
        path = (e[0], e[1])
        while path != loc:
            try:
                path = exit_came_from[path]
            except (KeyError):
                return self.calculateH(loc, e)
            counter += 1
        if counter == -1:
            return counter
        return counter

    # Utility function for the distance to the monster
    def monster_utility(self, wrld, name):
        try:
            chara = next(iter(wrld.characters.values()))
            character = chara[0]
        except (IndexError, StopIteration):
            return -10
        m = self.getMonster(wrld, name)
        if m.name == 'dead':
            return 100
        loc = (character.x, character.y)
        mloc = (m.x, m.y)
        monster_came_from, monster_cost_so_far = self.AStar(wrld, loc, mloc, [obstacles.MONSTER, obstacles.PLAYER, obstacles.EXIT])
        counter = 0
        path = mloc
        while path != loc:
            try:
                path = monster_came_from[path]
            except (KeyError):
                return 100
            counter += 1
        return counter

    # A Star algorithm
    def AStar(self, wrld, start, goal, list_of_e):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():

            current = frontier.get()[1]
            if current == goal:
                break

            for next in self.getNeighbors(current, wrld, list_of_e):
                new_cost = cost_so_far[current] + self.calculateH(next, current)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.calculateH(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current
        return came_from, cost_so_far

    # Heuristic calculation - returns euclidean distance
    def calculateH(self, loc1, loc2):
        (x1, y1) = loc1
        (x2, y2) = loc2
        return math.sqrt(((loc1[0] - loc2[0]) ** 2) + ((loc1[1] - loc2[1]) ** 2))

    # Calculates the dx and dy between two locations
    def calculateD(self, loc1, loc2):
        (x1, y1) = loc1
        (x2, y2) = loc2
        return ((x2 - x1), (y2 - y1))

    # Returns the neighbors of a particular location according to the obstacles passed in - obstacles passed in ARE AVAILABLE to be considered neighbors
    def getNeighbors(self, loc, wrld, list_of_e):
        list_of_N = []
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (loc[0] + dx >= 0) and (loc[0] + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Make sure the monster is moving
                    if (dx != 0) or (dy != 0):
                        # Avoid out-of-bound indexing
                        if (loc[1] + dy >= 0) and (loc[1] + dy < wrld.height()):
                            # No need to check impossible moves
                            if obstacles.EXIT in list_of_e:
                                if wrld.exit_at(loc[0] + dx, loc[1] + dy):
                                    list_of_N.append((loc[0] + dx, loc[1] + dy))
                            if obstacles.MONSTER in list_of_e:
                                if wrld.monsters_at(loc[0] + dx, loc[1] + dy):
                                    list_of_N.append((loc[0] + dx, loc[1] + dy))
                            if obstacles.PLAYER in list_of_e:
                                if wrld.characters_at(loc[0] + dx, loc[1] + dy):
                                    list_of_N.append((loc[0] + dx, loc[1] + dy))
                            if obstacles.WALL in list_of_e:
                                if wrld.wall_at(loc[0] + dx, loc[1] + dy):
                                    list_of_N.append((loc[0] + dx, loc[1] + dy))
                            if wrld.empty_at(loc[0] + dx, loc[1] + dy):
                                list_of_N.append((loc[0] + dx, loc[1] + dy))
        return list_of_N

    # Checks if location is in range of a bomb
    def bomb_check(self, loc, wrld):
        bomb_range = wrld.expl_range
        for dx in range(-bomb_range, bomb_range):
            # Avoid out-of-bound indexing
            if (loc[0] + dx >= 0) and (loc[0] + dx < wrld.width()):
                if wrld.bomb_at((loc[0] + dx), loc[1]):
                    return True
        for dy in range(-bomb_range, bomb_range):
            # Avoid out-of-bound indexing
            if (loc[1] + dy >= 0) and (loc[1] + dy < wrld.height()):
                if wrld.bomb_at(loc[0], (loc[1] + dy)):
                    return True
        return False

    # Checks if location is in range of an explosion
    def expl_check(self, loc, wrld):
        bomb_range = wrld.expl_range
        for dx in range(-bomb_range, bomb_range):
            # Avoid out-of-bound indexing
            if (loc[0] + dx >= 0) and (loc[0] + dx < wrld.width()):
                if wrld.explosion_at((loc[0] + dx), loc[1]):
                    return True
        for dy in range(-bomb_range, bomb_range):
            # Avoid out-of-bound indexing
            if (loc[1] + dy >= 0) and (loc[1] + dy < wrld.height()):
                if wrld.explosion_at(loc[0], (loc[1] + dy)):
                    return True
        return False

    #Returns states and potentially a list of threats
    def evaluateState(self, wrld):
        monsters = []
        try:
            chara = next(iter(wrld.characters.values()))
            character = chara[0]
        except (IndexError, StopIteration):
            return state.DEAD, []
        try:
            monsters = list(wrld.monsters.values())
        except (StopIteration):
            pass

        loc = (character.x, character.y)


        counters = {}
        #Calculate each distance to the monster
        for monster in monsters:
            m = monster[0]
            monsterType = m.name
            mloc = (m.x, m.y)
            monster_came_from, monster_cost_so_far = self.AStar(wrld, loc, mloc, [obstacles.MONSTER, obstacles.PLAYER, obstacles.EXIT])
            counter = 0
            path = mloc
            while path != loc:
                try:
                    path = monster_came_from[path]
                except (KeyError):
                    counter = 100
                    break
                counter += 1
            counters[monsterType] = counter
        counts = [(k, v) for k, v in counters.items() if v <= 4]
        flag = False
        monsterTypes = []
        for count in counts:
            if count[1] <= self.bound:
                flag = True
                monsterTypes.append((count[0], count[1]))
        threats = []
        # Sort the monster list in order of closest
        monsterTypes.sort(key=lambda x: x[1])
        for monster in monsterTypes:
            threats.append(monster[0])
        if flag:
            return state.UNSAFE, threats

        if (wrld.bombs or wrld.explosions):
            return state.NEAR_BOMB, []

        # Does safe path exist?
        came_from, cost_so_far = self.AStar(wrld, loc, wrld.exitcell, [obstacles.EXIT])
        for path in came_from:
            if (path == wrld.exitcell):
                return state.SAFE, []
        return state.BLOCKED, []


class state(Enum):
    SAFE = 1
    UNSAFE = 2
    DEAD = 3
    NEAR_BOMB = 4
    BLOCKED = 5

class obstacles(Enum):
    EXIT = 1
    MONSTER = 2
    WALL = 3
    BOMB = 4
    EXPLOSION = 5
    PLAYER = 6