import argparse
import heapq

class PathPlanning:
    def __init__(self, room, robots, rendezvous):
        self.room = room
        self.robots = robots
        self.rendezvous = rendezvous
        self.rows = len(room)
        self.cols = len(room[0])

    def is_valid(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols and self.room[x][y] == 0

    def heuristic(self, x, y):
        return abs(x - self.rendezvous[0]) + abs(y - self.rendezvous[1])

    def aStar(self, start):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # movement directions
        open_set = []
        
        heapq.heappush(open_set, (0, start)) #push starting position of robot
        g_score = {start: 0}
        came_from = {}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in visited:
                continue #if node has been visited, move on
            visited.add(current)

            if current == self.rendezvous:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(current)
                return path[::-1]

            x, y = current
            for dx, dy in directions:   #calculate heuristics and apply to A*
                neighbor = (x + dx, y + dy)
                if not self.is_valid(*neighbor):
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(*neighbor)
                    heapq.heappush(open_set, (f_score, neighbor))
        return []

    def find_paths(self):
        print()
        paths = {}
        for i, start in enumerate(self.robots):
            path = self.aStar(start)
            paths[f"Robot {i+1}"] = path
        return paths

def parse_input_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    # Remove comments and empty lines
    lines = [line.split('//')[0].strip() for line in lines if line.strip() and not line.strip().startswith('//')]

    # Parse room dimensions
    dimensions = lines.pop(0).split()
    rows, cols = int(dimensions[0]), int(dimensions[1])

    # Parse number of robots
    num_robots = int(lines.pop(0))

    # Parse robots' initial positions
    robots = []
    for _ in range(num_robots):
        pos = lines.pop(0).split()
        robots.append((int(pos[0]), int(pos[1])))  # Note: (row, col)

    # Parse rendezvous point
    rendezvous_point = lines.pop(0).split()
    rendezvous = (int(rendezvous_point[0]), int(rendezvous_point[1]))  # Note: (row, col)

    # Parse room layout
    room = []
    for line in lines:
        room_line = [int(char) for char in line]
        room.append(room_line)

    return dimensions, room, robots, rendezvous

# Main Execution
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Path Planning algorithm program for Robots")
    parser.add_argument("input_file", type=str, help="Path to the input file/ input file name")
    args = parser.parse_args()

    input_filename = args.input_file  # Get the input file from the command line
    input_filename = 'input4.txt'  # uncomment to specify or hard code input file name
    dimensions, room, robots, rendezvous = parse_input_file(input_filename)

    print("Room Dimensions: "+str(dimensions))
    print("Number of robots: "+str(len(robots)))
    print("Robot positions: "+str(robots))
    print("Rendezvous/Meeting point: "+str(rendezvous))

    planPaths = PathPlanning(room, robots, rendezvous)
    paths = planPaths.find_paths()

    # Output Paths
    for robot, path in paths.items():
        print(f"{robot}: {path if path else 'No path found'}")
