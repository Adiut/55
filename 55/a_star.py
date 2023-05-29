# Function to calculate the Manhattan distance heuristic
def heuristic(node, goal):
    x1, y1 = node
    x2, y2 = goal
    return abs(x2 - x1) + abs(y2 - y1)

# Function to find the node with the lowest f-score
def find_min_f_score(nodes, f_scores):
    min_node = None
    min_score = float('inf')
    for node in nodes:
        if f_scores[node] < min_score:
            min_node = node
            min_score = f_scores[node]
    return min_node

# Function to perform A* algorithm
def astar(graph, start, goal):
    open_set = {start}
    closed_set = set()
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, goal)}
    came_from = {}

    while open_set:
        current = find_min_f_score(open_set, f_scores)

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        open_set.remove(current)
        closed_set.add(current)

        for neighbor, distance in graph[current].items():
            tentative_g_score = g_scores[current] + distance

            if neighbor in closed_set and tentative_g_score >= g_scores.get(neighbor, float('inf')):
                continue

            if neighbor not in open_set or tentative_g_score < g_scores[neighbor]:
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = g_scores[neighbor] + heuristic(neighbor, goal)
                open_set.add(neighbor)

    return None

# Function to reconstruct the path
def reconstruct_path(came_from, start, goal):
    path = [goal]
    current = goal

    while current != start:
        current = came_from[current]
        path.insert(0, current)

    return path

# Function to create a graph from user input
def create_graph():
    graph = {}
    vertices = int(input("Enter the number of vertices: "))

    for _ in range(vertices):
        node = tuple(map(int, input("Enter node coordinates (x, y): ").split()))
        neighbors = {}

        num_neighbors = int(input(f"Enter the number of neighbors for node {node}: "))
        for _ in range(num_neighbors):
            neighbor, distance = map(int, input("Enter neighbor coordinates (x, y) and distance: ").split())
            neighbors[tuple(neighbor)] = distance

        graph[node] = neighbors

    return graph

# Main program
def main():
    graph = create_graph()
    start_node = tuple(map(int, input("Enter the coordinates of the start node (x, y): ").split()))
    goal_node = tuple(map(int, input("Enter the coordinates of the goal node (x, y): ").split()))

    path = astar(graph, start_node, goal_node)

    if path:
        print("Path found:")
        for node in path:
            print(node)
    else:
        print("Path not found.")

if __name__ == "__main__":
    main()
# Function to calculate the Manhattan distance heuristic
def heuristic(node, goal):
    x1, y1 = node
    x2, y2 = goal
    return abs(x2 - x1) + abs(y2 - y1)

# Function to find the node with the lowest f-score
def find_min_f_score(nodes, f_scores):
    min_node = None
    min_score = float('inf')
    for node in nodes:
        if f_scores[node] < min_score:
            min_node = node
            min_score = f_scores[node]
    return min_node

# Function to perform A* algorithm
def astar(graph, start, goal):
    open_set = {start}
    closed_set = set()
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, goal)}
    came_from = {}

    while open_set:
        current = find_min_f_score(open_set, f_scores)

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        open_set.remove(current)
        closed_set.add(current)

        for neighbor, distance in graph[current].items():
            tentative_g_score = g_scores[current] + distance

            if neighbor in closed_set and tentative_g_score >= g_scores.get(neighbor, float('inf')):
                continue

            if neighbor not in open_set or tentative_g_score < g_scores[neighbor]:
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = g_scores[neighbor] + heuristic(neighbor, goal)
                open_set.add(neighbor)

    return None

# Function to reconstruct the path
def reconstruct_path(came_from, start, goal):
    path = [goal]
    current = goal

    while current != start:
        current = came_from[current]
        path.insert(0, current)

    return path

# Function to create a graph from user input
def create_graph():
    graph = {}
    vertices = int(input("Enter the number of vertices: "))

    for _ in range(vertices):
        node = tuple(map(int, input("Enter node coordinates (x, y): ").split()))
        neighbors = {}

        num_neighbors = int(input(f"Enter the number of neighbors for node {node}: "))
        for _ in range(num_neighbors):
            neighbor, distance = map(int, input("Enter neighbor coordinates (x, y) and distance: ").split())
            neighbors[tuple(neighbor)] = distance

        graph[node] = neighbors

    return graph

# Main program
def main():
    graph = create_graph()
    start_node = tuple(map(int, input("Enter the coordinates of the start node (x, y): ").split()))
    goal_node = tuple(map(int, input("Enter the coordinates of the goal node (x, y): ").split()))

    path = astar(graph, start_node, goal_node)

    if path:
        print("Path found:")
        for node in path:
            print(node)
    else:
        print("Path not found.")

if __name__ == "__main__":
    main()

