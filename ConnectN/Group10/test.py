import random
import game
import agent
import alpha_beta_agent as aba


##############
# Test Class #
##############

# Test suite used for testing agent against random
def run_test_suite(ABPlayer, depth, time, x, y, n):
    seed = 1234
    random.seed(seed)

    max = 25
    AlphaBetaVictories = 0
    for i in range(0, 25):

        #
        # Random vs. AlphaBeta
        #
        goSecond = game.Game(x,  # width
                             y,  # height
                             n,  # tokens in a row to win
                             agent.RandomAgent("random"),  # player 1
                             aba.AlphaBetaAgent("alphabeta", depth))  # player 2

        # AlphaBeta vs Random
        #
        goFirst = game.Game(x,  # width
                            y,  # height
                            n,  # tokens in a row to win
                            aba.AlphaBetaAgent("alphabeta", depth),  # player 1
                            agent.RandomAgent("random"))  # player 2

        if ABPlayer == 1:
            outcome = goFirst.timed_go(time)
            if outcome == 1:
                AlphaBetaVictories += 1
        else:
            outcome = goSecond.timed_go(time)
            if outcome == 2:
                AlphaBetaVictories += 1
        seed += random.randint(1, 100)
        #print("RANDOM SEED: " + str(seed))
        random.seed(seed)
        print("Game " + str(i) + " complete")
    print("AlphaBeta won " + str(AlphaBetaVictories) + " out of " + str(max))


# Test Connect-4
run_test_suite(2, 5, 15, 6, 7, 4)
# Test Connect-5
#run_test_suite(2, 5, 15, 8, 10, 5)
