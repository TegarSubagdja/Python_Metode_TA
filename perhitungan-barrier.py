import numpy as np
import heapq
import math

# Representasi peta: 2 = Start, 3 = Goal, 1 = Obstacle, 0 = Free space
map_grid = np.array([
    [2, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 3]
])

# Fungsi untuk menghitung jarak Euclidean
def euclidean_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

# Cari koordinat dari nilai tertentu dalam grid
def find_coordinates(grid, value):
    result = np.argwhere(grid == value)
    return tuple(result[0]) if result.size > 0 else None

# Fungsi untuk menghitung Barrier Raster Coefficient (P)
def compute_barrier_coefficient(current, goal, grid):
    x1, y1 = current
    x2, y2 = goal
    x_min, x_max = min(x1, x2), max(x1, x2)
    y_min, y_max = min(y1, y2), max(y1, y2)
    
    obstacle_count = np.sum(grid[x_min:x_max + 1, y_min:y_max + 1] == 1)
    total_area = (x_max - x_min + 1) * (y_max - y_min + 1)
    
    P = obstacle_count / total_area if total_area > 0 else 0.01  # Avoid division by zero
    return max(P, 0.01)  # Ensure P is non-zero to avoid log issues

# A* Algorithm with Barrier Raster Coefficient and Turn Penalty
def a_star_search(grid, turn_penalty_coefficient=1.0):
    # Temukan titik start dan goal
    start = find_coordinates(grid, 2)
    goal = find_coordinates(grid, 3)
    
    if not start or not goal:
        raise ValueError("Start or Goal node not found in the grid.")
    
    rows, cols = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))  # Priority queue (f_score, node)
    came_from = {}  # Untuk melacak jalur

    # G-score (biaya dari start ke node saat ini)
    g_score = {start: 0}
    # F-score (g_score + heuristic)
    f_score = {start: euclidean_distance(start, goal)}
    
    # Closed list untuk melacak node yang sudah diproses
    closed_list = set()

    # Set neighbor offsets (horizontal, vertical, diagonal)
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while open_list:
        # Ambil node dengan f_score terendah
        current = heapq.heappop(open_list)[1]
        closed_list.add(current)  # Tambahkan ke closed list
        
        # Jika goal tercapai, rekonstruksi jalur
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Balikkan jalur
        
        # Proses semua neighbor
        for offset in neighbors:
            neighbor = (current[0] + offset[0], current[1] + offset[1])
            
            # Pastikan neighbor valid (dalam grid dan bukan halangan)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 1:
                if neighbor in closed_list:
                    continue  # Skip jika sudah di closed list
                
                # Hitung g_score baru
                tentative_g_score = g_score[current] + (euclidean_distance(current, neighbor) if offset[0] != 0 and offset[1] != 0 else 1)
                
                # Hitung Barrier Raster Coefficient (P)
                P = compute_barrier_coefficient(current, goal, grid)
                h = (1 - math.log(P)) * euclidean_distance(neighbor, goal)
                
                # Hitung Turn Penalty
                if current in came_from:
                    prev = came_from[current]
                    dx1, dy1 = goal[0] - current[0], goal[1] - current[1]
                    dx2, dy2 = neighbor[0] - current[0], neighbor[1] - current[1]
                    turn_penalty = abs(dx1 * dy2 - dx2 * dy1) * turn_penalty_coefficient
                else:
                    turn_penalty = 0
                
                # Hitung total f_score
                f = tentative_g_score + h + turn_penalty
                
                # Jika g_score baru lebih baik, atau neighbor belum dikunjungi
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Simpan jalur terbaik ke neighbor
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = f

                    # Tambahkan neighbor ke open_list jika belum ada
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        print(f"Added to open list: {neighbor}")
                
                # Tampilkan perhitungan g, h, f
                print(f"Neighbor: {neighbor}")
                print(f"  g = {tentative_g_score:.2f}")
                print(f"  h = {h:.2f} (P = {P:.2f})")
                print(f"  Turn Penalty = {turn_penalty:.2f}")
                print(f"  f = {f:.2f}")
        
        print(f"Closed list: {list(closed_list)}")
        print('=========================================')
    
    return None  # Tidak ada jalur ditemukan

# Fungsi untuk mengganti path dengan nilai 5
def mark_path_on_map(grid, path):
    output_grid = grid.copy()
    for step in path:
        if output_grid[step] not in (2, 3):  # Jangan ubah start (2) dan goal (3)
            output_grid[step] = 5
    return output_grid

# Jalankan algoritma A*
path = a_star_search(map_grid, turn_penalty_coefficient=1.0)

# Tampilkan hasil
if path:
    print("Path found:")
    print([tuple(map(int, step)) for step in path])  # Format koordinat sebagai angka biasa
    # Tandai path pada peta
    result_grid = mark_path_on_map(map_grid, path)
    print("\nMap with path marked as 5:")
    print(result_grid)
else:
    print("No path found.")
