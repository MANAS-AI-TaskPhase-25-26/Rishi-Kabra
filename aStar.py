import heapq
import numpy as np
import cv2

numeric_grid = np.load('aStar_data.npy')
standard_vis = cv2.imread('costmap_visual.png')

def aStar(grid, start, goal):
    height, width = grid.shape

    list = []
    heapq.heappush(list, (0, 0, start[0], start[1], []))

    visited = set()

    directions = [
        (0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]
    
    while list:
        _, g_cost, x, y, path = heapq.heappop(list)
        if (x, y) in visited:
            continue

        current_path = path + [(x, y)]
        
        if (x, y) == goal:
            return current_path
        
        visited.add((x, y))

        for dx, dy, move_weight in directions:
            nx, ny = x+dx, y+dy

            if 0<=nx <width and 0 <= ny < height:
                pixel_val = grid[ny, nx]
                if pixel_val < 30 or pixel_val == 255: 
                    continue

                elevation_penalty = (255 - pixel_val) * 1.5

                new_g_cost = g_cost + move_weight + elevation_penalty
                h_cost = np.sqrt((goal[0] - nx)**2 + (goal[1] - ny)**2)
                f_cost = new_g_cost + h_cost

                if (nx, ny) not in visited:
                    heapq.heappush(list, (f_cost, new_g_cost, nx, ny, current_path))

    return None



with open('pathPoints.txt', 'r') as file:
    for line in file:
        points = line.split(',')
        start_point = (int(points[0]), int(points[1]))
        end_point = (int(points[2]), int(points[3]))


print("Planning path...")
path = aStar(numeric_grid, start_point, end_point)

if path:
    path_pts = np.array(path, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(standard_vis, [path_pts], isClosed=False, color=(0, 255, 0), thickness=4)
    cv2.circle(standard_vis, path[0], 5, (255, 0, 0), -1)   
    cv2.circle(standard_vis, path[-1], 5, (0, 0, 255), -1) 
    print("Path rendered successfully.")
else:
    print("No path to display.")

cv2.namedWindow("Planned Path on Standard Costmap", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Planned Path on Standard Costmap", 1200, 800)
cv2.imshow("Planned Path on Standard Costmap", standard_vis)
cv2.waitKey(0) 
cv2.destroyAllWindows()