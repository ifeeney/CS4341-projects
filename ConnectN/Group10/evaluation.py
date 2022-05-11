####################
# Evaluation Class #
####################


# Class used to evaluate a board in terminal and non-terminal conditions
class Evaluation(object):
    def __init__(self, board, agent):
        self.board = board
        self.width = board.w
        self.height = board.h
        self.agent = agent

    # Function used to evaluate a board by scoring based on the established heuristics
    def score(self):
        # Cycle through all the spaces with tokens and score them
        score = 0
        # Go for center
        for y in range(self.height):
            for x in range(self.width):
                if self.board.board[y][x] == self.agent.player:
                    score += (-(y + 1) * 2) + (((self.width / 2) - (abs((self.width / 2) - x))) + 1) * 50
        # (N-1) In a row implementation
        # score += 100 * self.find_connect_3()[0]
        # score += -100 * self.find_connect_3()[1]
        # Terminal scoring
        if self.board.get_outcome() == self.agent.player:
            score += 1000
        # Blocks enemy moves
        if self.board.get_outcome() == self.agent.enemy:
            score -= 1000
        return score

    # Function to find N-1 pieces in a line
    def find_connect_3(self):
        currPlayerPoints = 0
        enemy_player_points = 0
        for x in range(self.width):
            for y in range(self.height):
                for dx, dy in [(1, 0), (1, 1), (0, 1), (1, -1)]:  # Check each direction at each spot
                    xpos = x
                    ypos = y
                    if xpos >= self.height or ypos >= self.width: break
                    seq = 0
                    startToken = self.board.board[x][y]
                    if startToken == 0: break  # Not a player
                    for i in range(0, 2):  # Check in direction (dy, dx)
                        xpos = xpos + dx
                        ypos = ypos + dy
                        if xpos >= self.height or ypos >= self.width: break  # Out of bounds
                        currentToken = self.board.board[xpos][ypos]
                        if currentToken != startToken: break
                        seq = seq + 1
                    if seq >= 2:
                        if startToken == self.agent.player:
                            currPlayerPoints = currPlayerPoints + 1
                        else:
                            enemy_player_points = enemy_player_points + 1
        #print(currPlayerPoints, enemy_player_points)
        return currPlayerPoints, enemy_player_points

    # Function to find a space of N spots with N-1 player pieces
    # This allows us to fill in the remaining spot and win the match
    def find_three_of_four(self):
        currPlayerPoints = 0
        enemy_player_points = 0
        for x in range(self.width):
            for y in range(self.height):
                for dx, dy in [(1, 0), (1, 1), (0, 1), (1, -1)]:  # Check each direction at each spot
                    xpos = x
                    ypos = y
                    if xpos >= self.height or ypos >= self.width: break
                    seq = 0
                    startToken = self.board.board[x][y]
                    if startToken == 0: break  # Not a player
                    for i in range(0, 3):  # Check in direction (dy, dx)
                        xpos = xpos + dx
                        ypos = ypos + dy
                        if xpos >= self.height or ypos >= self.width: break  # Out of bounds
                        currentToken = self.board.board[xpos][ypos]
                        if currentToken == startToken or currentToken == 0:
                            if currentToken == startToken:
                                seq = seq + 1
                        else:
                            break
                    if seq >= 2:
                        if startToken == self.agent.player:
                            currPlayerPoints = currPlayerPoints + 1
                        else:
                            enemy_player_points = enemy_player_points + 1
        return currPlayerPoints, enemy_player_points

    def empty_spaces_below(self, x, y):
        """Return 0 if there is an even number of empty spaces at and below (x,y)
            and 1 if there is an odd number"""
        empty_spaces = 1
        # while(x < self.h - 1):
        #    x = x + 1
        while (x > 0):
            x = x - 1
            if (self.board[x][y] == 0):
                empty_spaces = empty_spaces + 1
            else:
                break
        #print(empty_spaces)
        if (empty_spaces % 2 == 0): return 0
        return 1
