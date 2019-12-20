#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems
from os import times

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    return 0

def heur_manhattan_distance(state):
#OPTIONAL
    '''Manhattan distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses Manhattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the Manhattan distances between each xanadu and the escape hatch.
    manhattan_distances = 0
    center = (state.width - 1) // 2

    for rover in state.xanadus:
        manhattan_distance = abs(rover[0] - center) + abs(rover[1] - center)
        manhattan_distances += manhattan_distance

    return manhattan_distances

def heur_L_distance(state):
    #IMPLEMENT
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses mahnattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the L distances between each xanadu and the escape hatch.
    L_distances = 0
    center = (state.width - 1) // 2

    for rover in state.xanadus:
        if (rover[0] == center and rover[1] == center):
            L_distance = 0
        elif (rover[0] == center or rover[1] == center):
            L_distance = 1
        else:
            L_distance = 2

        L_distances += L_distance

    return L_distances

def _approx_center(state: LunarLockoutState, rover: tuple) -> int:
    """
    Helper function for heur_alternate.
    Returns the least number of moves a robot is requite to make
    """
    center = (state.width - 1) // 2

    for robot in state.robots:
        # Case 1: vertical alignment
        if (rover[0] == robot[0]):
            if (rover[1] < robot[1] and robot[1] == center + 1):
                return 0
            elif (rover[1] > robot[1] and robot[1] == center - 1):
                return 0

        # Case 2: horizontal alignment
        else:
            if (rover[0] < robot[0] and robot[0] == center + 1):
                return 0
            elif (rover[0] > robot[0] and robot[0] == center - 1):
                return 0

    for xanadu in state.xanadus:
        if (xanadu != rover):
            if (rover[0] == xanadu[0]):
                if (rover[1] < xanadu[1] and xanadu[1] == center + 1):
                    return 0
                elif (rover[1] > xanadu[1] and xanadu[1] == center - 1):
                    return 0

            else:
                if (rover[0] < xanadu[0] and robot[0] == center + 1):
                    return 0
                elif (rover[0] > xanadu[0] and xanadu[0] == center - 1):
                    return 0

    # No robots or other rovers are in an optimal position, so we need to
    # move the robot at least once
    return 2

def _approx_quadrant(state: LunarLockoutState, rover: tuple) -> int:
    """
    Helper function for heur_alternate.
    Returns the least number of moves a robot is requite to make
    """
    center = (state.width - 1) // 2

    # Check if there are any robots or xanadus in the same row or column
    for robot in state.robots:
        if (rover[0] == robot[0] or rover[1] == robot[1]):
            return 0

    for xanadu in state.xanadus:
        if (rover != xanadu):
            if (rover[0] == xanadu[0] or rover[1] == xanadu[1]):
                return 0

    # Need to move at least one robot or xanadu past this point
    # Calculate the optimal position for this rover
    # Case 1: Upper left quadrant
    if (rover[0] < center and rover[1] > center):
        optimal = (center + 1, rover[1])
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (center, rover[1]))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (center, rover[1]))

        optimal = (rover[0], center + 1)
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (rover[0], center))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (rover[0], center))


    # Case 2: Upper right quadrant
    elif (rover[0] > center and rover[1] > center):
        optimal = (rover[0], center + 1)
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (rover[0], center))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (rover[0], center))

        optimal = (center - 1, rover[1])
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (center, rover[1]))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (center, rover[1]))

    # Case 3: Lower left quad
    elif (rover[0] < center and rover[1] < center):
        optimal = (center + 1, rover[1])
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (center, rover[1]))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (center, rover[1]))

        optimal = (rover[0], center - 1)
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (rover[0], center))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (rover[0], center))

    # Case 4: Lower right quad
    elif (rover[0] > center and rover[1] < center):
        optimal = (rover[0], center - 1)
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (rover[0], center))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (rover[0], center))

        optimal = (center - 1, rover[1])
        for robot in state.robots:
            if (robot == optimal):
                return _approx_center(state, (center, rover[1]))
        for xanadu in state.xanadus:
            if (xanadu != rover):
                if (xanadu == optimal):
                    return _approx_center(state, (center, rover[1]))

    # Need at least two robot moves
    return 2


    # IMPLEMENT
def heur_alternate(state):
    """
    a better lunar lockout heuristic
    INPUT: a lunar lockout state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.

    <heur_alternate> is an improved version of the L distance heuristic because
    for any state <s>, heur_alt(s) >= heur_L_dist(s). In other words, heur_alt
    heuristic completely dominates the L distance heuristic. Thus, this means
    that a search using L distance will have to check additional nodes for the
    goal state because it doesn't predict the goal state as accurately as
    alternate heuristic.

    Note: I probably could have overestimated my alt heuristic a bit for better
    runtime, but I wasn't aware we could make it inadmissable until I read
    piazza, and this still passes the 20 tests.
    """
    #Your function should return a numeric value for the estimate of the distance to the goal.
    alt_approx = 0
    center = (state.width - 1) // 2

    # Check if there is a robot standing on the center or if there are no
    # robots anywhere around the center
    for robot in state.robots:
        if (robot[0] == center and robot[1] == center):
            alt_approx += 1
        elif (robot[0] == center and (robot[1] != center + 1 or robot[1] != center - 1)):
            alt_approx += 1
        elif ((robot[0] != center + 1 or robot[0] != center - 1) and robot[1] == center):
            alt_approx += 1

    for rover in state.xanadus:
        if (rover[0] == center and rover[1] == center):
            pass

        elif (rover[0] == center or rover[1] == center):
            # The rover needs to make at least one move
            alt_approx += (1 + _approx_center(state, rover))

        else:
            # The rover nees to make at least two moves
            alt_approx += (2 + _approx_quadrant(state, rover))

    return alt_approx

def fval_function(sN, weight):
    #IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a LunarLockoutState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 2):
    #IMPLEMENT
    """
    Provides an implementation of anytime weighted a-star, as described in the HW1 handout
    INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)
    OUTPUT: A goal state (if a goal is found), else False
    implementation of weighted astar algorithm
    """
    # Set up search engine
    wrapped_fval_func = lambda sN: fval_function(sN, weight)
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, lockout_goal_state, heur_fn, wrapped_fval_func)

    # Set the time limit
    stop_time = times()[0] + timebound      # start_time + timebound

    # Initialize variables and run the first search
    costbound = (float('inf'), float('inf'), float('inf'))
    candidate = se.search(timebound)
    goal_state = False      # best goal state seen so far

    # Search for goal state until current time >= stop_time
    while (times()[0] < stop_time):
        if (candidate is False):   # no goal state was found
            return goal_state

        fval = heur_fn(candidate) + candidate.gval
        if (fval < costbound[2]):       # elif
            costbound = (candidate.gval, heur_fn(candidate), fval)
            goal_state = candidate

        candidate = se.search(stop_time - times()[0], costbound)

    return goal_state

def anytime_gbfs(initial_state, heur_fn, timebound=2):
    #OPTIONAL
    """
    Provides an implementation of anytime greedy best-first search.  This iteratively uses greedy best first search,
    At each iteration, however, a cost bound is enforced.  At each iteration the cost of the current "best" solution
    is used to set the cost bound for the next iteration.  Only paths within the cost bound are considered at each iteration.
    INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)
    OUTPUT: A goal state (if a goal is found), else False
    """
    # Set up search engine
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, lockout_goal_state, heur_fn)

    # Set the time limit
    stop_time = times()[0] + timebound      # start_time + timebound

    # Initialize variables and run the first search
    costbound = (float('inf'), float('inf'), float('inf'))
    candidate = se.search(timebound)
    goal_state = False      # best goal state seen so far

    # Search for goal state until current time >= stop_time
    while (times()[0] < stop_time):
        if (candidate is False):   # no goal state was found
            return goal_state

        elif (heur_fn(candidate) < costbound[1]):
            fval = heur_fn(candidate) + candidate.gval
            costbound = (candidate.gval, heur_fn(candidate), fval)
            goal_state = candidate

        candidate = se.search(stop_time - times()[0], costbound)

    return goal_state

PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3),)),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")
  print("Running A-star")

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******")
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_alternate)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")

  solved = 0; unsolved = []; counter = 0; percent = 0;
  print("Running Anytime Weighted A-star")

  for i in range(len(PROBLEMS)):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]
    weight = 4
    final = anytime_weighted_astar(s0, heur_alternate, weight, timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")

  solved = 0; unsolved = []; counter = 0; percent = 0;
  print("Running Anytime GBFS")

  for i in range(len(PROBLEMS)):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]
    final = anytime_gbfs(s0, heur_alternate, timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")
