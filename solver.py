import time
import sys


class puzzle:
    def __init__(self, config, n, parent=None, action="Initial", cost=0, manhattan=0):
        self.n = n
        self.config = config
        self.cost = cost
        self.manhattan = manhattan
        self.parent = parent
        self.action = action
        self.children = []
        for i, item in enumerate(self.config):
            if item == 0:
                self.blank_row = i // self.n
                self.blank_col = i % self.n

    def display(self):
        for i in range(self.n):
            line = []
            offset = i * self.n
            for j in range(self.n):
                line.append(self.config[offset + j])
            print(line)

    def move_left(self):
        if self.blank_col == 0:
            return None
        else:
            index = self.blank_row * self.n + self.blank_col
            target = index - 1
            new_config = list(self.config)
            new_config[index], new_config[target] = new_config[target], new_config[index]
            #           self.blank_col -= 1
            return puzzle(tuple(new_config), self.n, action="LEFT", parent=self, cost=self.cost + 1)

    def move_right(self):
        if self.blank_col == self.n - 1:
            return None
        else:
            index = self.blank_row * self.n + self.blank_col
            target = index + 1
            new_config = list(self.config)
            new_config[index], new_config[target] = new_config[target], new_config[index]
            #           self.blank_col += 1
            return puzzle(tuple(new_config), self.n, action="RIGHT", parent=self, cost=self.cost + 1)

    def move_up(self):
        if self.blank_row == 0:
            return None
        else:
            index = self.blank_row * self.n + self.blank_col
            target = index - self.n
            new_config = list(self.config)
            new_config[index], new_config[target] = new_config[target], new_config[index]
            #            self.blank_row -= 1
            return puzzle(tuple(new_config), self.n, action="UP", parent=self, cost=self.cost + 1)

    def move_down(self):
        if self.blank_row == self.n - 1:
            return None
        else:
            index = self.blank_row * self.n + self.blank_col
            target = index + self.n
            new_config = list(self.config)
            new_config[index], new_config[target] = new_config[target], new_config[index]
            #           self.blank_row += 1
            return puzzle(tuple(new_config), self.n, action="DOWN", parent=self, cost=self.cost + 1)

    def expand(self):
        if len(self.children) == 0:
            up_child = self.move_up()
            if up_child is not None:
                self.children.append(up_child)
            down_child = self.move_down()
            if down_child is not None:
                self.children.append(down_child)
            left_child = self.move_left()
            if left_child is not None:
                self.children.append(left_child)
            right_child = self.move_right()
            if right_child is not None:
                self.children.append(right_child)
        return self.children


def search(puzzle_state, goal, mode):
    start = time.perf_counter()
    if mode == "bfs":
        index = 0
    elif mode == "dfs":
        index = -1
    frontier = []
    explored = set()
    nodes_expanded = 0
    tmp = []
    max_depth = puzzle_state.cost
    frontier.append(puzzle_state)
    while frontier:
        print(nodes_expanded)
        state = frontier[index]
        frontier.pop(index)
        explored.add(state.config)
        test_goal(state, goal, start, nodes_expanded, max_depth)
        nodes_expanded += 1
        for child in state.expand():
            if child.config not in explored:
                if mode == "bfs":
                    frontier.append(child)
                elif mode == "dfs":
                    tmp.append(child)
                explored.add(child.config)
                max_depth = get_max_depth(max_depth, child.cost)
        while tmp:   # for dfs
            frontier.append(tmp.pop())
    print("Not solvable")
    return False


def A_search(puzzle_state, goal):
    start = time.perf_counter()
    frontier = []
    explored = set()
    nodes_expanded = 0
    max_depth = puzzle_state.cost
    frontier.append(puzzle_state)
    while frontier:
        print(nodes_expanded)
        index = manhattan_search(frontier)
        state = frontier[index]
        frontier.pop(index)
        explored.add(state.config)
        test_goal(state, goal, start, nodes_expanded, max_depth)
        nodes_expanded += 1
        for child in state.expand():
            if child.config not in explored:
                child.manhattan = child.cost + calculate_manhattan_dist(child.config, goal, child.n)
                frontier.append(child)
                explored.add(child.config)
                max_depth = get_max_depth(max_depth, child.cost)
    print("Not solvable")
    return False

def manhattan_search(frontier):
    current = frontier[0]
    index = 0
    for i, state in enumerate(frontier):
        if state.manhattan < current.manhattan:
            current = state
            index = i
    return index



def calculate_manhattan_dist(config, goal, n):
    cost = 0
    for dex, item in enumerate(config):
        target = goal[item]
        target_row = target // n
        target_col = target % n
        current_row = dex // n
        current_col = dex % n
        cost += abs(current_col - target_col) + abs(current_row - target_row)
    return cost



def get_max_depth(max, cost):
    if cost > max:
        return cost
    else:
        return max


def ram():
    if sys.platform == "win32":
        import psutil
        return psutil.Process().memory_info().rss
    else:
        # Note: if you execute Python from cygwin,
        # the sys.platform is "cygwin"
        # the grading system's sys.platform is "linux2"
        import resource
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss


def write_output(path, cost, nodes_expanded, run_time, max):
    print(f"path: {path}\n"
          f"cost:{cost}\n"
          f"nodes expanded: {nodes_expanded}\n"
          f"search depth: {cost}\n"
          f"max search depth: {max}\n"
          f"running time: {run_time}\n"
          f"max ram usage: {ram() / 1024 / 1024}")
    exit(0)


def test_goal(state, goal, start, nodes_expanded, max_depth):
    if state.config != goal:
        return True
    elif state.config == goal:
        path = []
        cost = state.cost
        path.append(state.action)
        while True:
            if state.parent.action == "Initial":
                path.reverse()
                finish = time.perf_counter()
                run_time = f'{round(finish - start, 2)}second(s)'
                write_output(path, cost, nodes_expanded, run_time, max_depth)
                break
            path.append(state.parent.action)
            state = state.parent
        return False


def main():
    n_ = 3
    config_ = (1,0,2,7,5,4,8,6,3)
    goal = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    hard_state = puzzle(config_, n_)
    #search(hard_state, goal, "bfs")
    A_search(hard_state, goal)

if __name__ == '__main__': main()