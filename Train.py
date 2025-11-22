import heapq

class TrainSwitchPuzzle:
    def __init__(self, graph, start, goal, heuristic_map):
        self.graph = graph
        self.start = start
        self.goal = goal
        self.heuristic_map = heuristic_map

    def goal_test(self, state):
        return state == self.goal

    def neighbors(self, state):
        return self.graph.get(state, {})

# UNIFORM COST SEARCH 
def uniform_cost_search(problem):
    frontier = [(0, problem.start, [problem.start])]
    explored = set()

    while frontier:
        cost, node, path = heapq.heappop(frontier)
        if problem.goal_test(node):
            return cost, path
        if node in explored:
            continue
        explored.add(node)
        for neighbor, step_cost in problem.neighbors(node).items():
            heapq.heappush(frontier, (cost + step_cost, neighbor, path + [neighbor]))
    return None, None

#  A* SEARCH 
def a_star_search(problem):
    frontier = [(0, problem.start, [problem.start])]
    explored = set()

    while frontier:
        cost, node, path = heapq.heappop(frontier)
        if problem.goal_test(node):
            return cost, path
        if node in explored:
            continue
        explored.add(node)
        for neighbor, step_cost in problem.neighbors(node).items():
            g = cost + step_cost
            h = problem.heuristic_map.get(neighbor, 0)
            f = g + h
            heapq.heappush(frontier, (f, neighbor, path + [neighbor]))
    return None, None


if __name__ == "__main__":
    
    # Define graph: 
    graph = {
        'A': {'B': 2, 'C': 5},
        'B': {'D': 3, 'E': 4},
        'C': {'E': 2},
        'D': {'E': 1},
        'E': {}
    }

    # Estimated heuristic times to goal 'E'
    heuristic_map = {'A': 6, 'B': 4, 'C': 2, 'D': 1, 'E': 0}

    problem = TrainSwitchPuzzle(graph, 'A', 'E', heuristic_map)

    cost_ucs, path_ucs = uniform_cost_search(problem)
    cost_astar, path_astar = a_star_search(problem)

    print(f"UCS Path: {path_ucs}, Total Cost: {cost_ucs}")
    print(f"A* Path: {path_astar}, Total Cost: {cost_astar}")