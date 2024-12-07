import numpy as np
import heapq
import math

class AStarPathfinder:
    def __init__(self, grid):
        """
        Inisialisasi A* pathfinder
        grid: numpy array dengan format:
            0 = Free space
            1 = Obstacle/Wall
            2 = Start point
            3 = Goal/Target
        """
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.start = self._find_coordinates(2)  # Start point (value 2)
        self.goal = self._find_coordinates(3)   # Goal point (value 3)
        
        # Definisi arah gerakan (horizontal, vertikal, diagonal)
        self.neighbors = [
            (-1, 0),  # Atas
            (1, 0),   # Bawah
            (0, -1),  # Kiri
            (0, 1),   # Kanan
            (-1, -1), # Diagonal kiri atas
            (-1, 1),  # Diagonal kanan atas
            (1, -1),  # Diagonal kiri bawah
            (1, 1)    # Diagonal kanan bawah
        ]

    def _find_coordinates(self, value):
        """Mencari koordinat dari nilai tertentu dalam grid"""
        result = np.argwhere(self.grid == value)
        if result.size == 0:
            raise ValueError(f"Nilai {value} tidak ditemukan dalam grid")
        return tuple(result[0])

    def _euclidean_distance(self, node1, node2):
        """Menghitung jarak Euclidean antara dua node"""
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    def _get_movement_cost(self, offset):
        """Menghitung biaya pergerakan (1 untuk orthogonal, √2 untuk diagonal)"""
        return math.sqrt(offset[0]**2 + offset[1]**2)

    def find_path(self, debug=True):
        """
        Mencari jalur menggunakan algoritma A*
        debug: Boolean untuk menampilkan informasi debugging
        """
        if debug:
            print("Starting A* pathfinding...")
            print(f"Start position: {self.start}")
            print(f"Goal position: {self.goal}")

        # Inisialisasi struktur data
        open_list = []  # Priority queue untuk node yang akan diperiksa
        heapq.heappush(open_list, (0, self.start))
        came_from = {}  # Untuk melacak jalur
        g_score = {self.start: 0}  # Biaya dari start ke setiap node
        f_score = {self.start: self._euclidean_distance(self.start, self.goal)}
        closed_list = set()  # Set untuk node yang sudah diperiksa
        
        step = 1
        while open_list:
            current = heapq.heappop(open_list)[1]
            
            if current == self.goal:
                if debug:
                    print("\nGoal reached! Reconstructing path...")
                return self._reconstruct_path(came_from)
            
            closed_list.add(current)
            
            if debug:
                print(f"\nStep {step}: Examining node {current}")
                print(f"Current g_score: {g_score[current]:.3f}")
            
            # Periksa semua tetangga
            for offset in self.neighbors:
                neighbor = (current[0] + offset[0], current[1] + offset[1])
                
                # Validasi neighbor
                if not (0 <= neighbor[0] < self.rows and 
                       0 <= neighbor[1] < self.cols and 
                       self.grid[neighbor] != 1):
                    continue
                
                if neighbor in closed_list:
                    continue
                
                # Hitung skor
                movement_cost = self._get_movement_cost(offset)
                tentative_g_score = g_score[current] + movement_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Ditemukan jalur yang lebih baik ke neighbor
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    h_score = self._euclidean_distance(neighbor, self.goal)
                    f_score[neighbor] = tentative_g_score + h_score
                    
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        
                        if debug:
                            print(f"\nNeighbor {neighbor}:")
                            print(f"  Movement cost: {movement_cost:.3f}")
                            print(f"  g_score: {tentative_g_score:.3f}")
                            print(f"  h_score: {h_score:.3f}")
                            print(f"  f_score: {f_score[neighbor]:.3f}")
            
            if debug:
                print(f"Closed list: {list(closed_list)}")
                print("=" * 80)
            
            step += 1
        
        if debug:
            print("\nNo path found!")
        return None

    def _reconstruct_path(self, came_from):
        """Merekonstruksi jalur dari goal ke start"""
        current = self.goal
        path = []
        
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        
        return path[::-1]  # Balik jalur agar dari start ke goal

    def mark_path_on_grid(self, path):
        """Menandai jalur pada grid dengan nilai 5"""
        if path is None:
            return self.grid
            
        marked_grid = self.grid.copy()
        for x, y in path:
            if marked_grid[x, y] not in (2, 3):  # Jangan ubah start dan goal
                marked_grid[x, y] = 5
        return marked_grid

def main():
    # Contoh penggunaan
    # Buat grid peta (2 = Start, 3 = Goal, 1 = Obstacle, 0 = Free space)
    map_grid = np.array([
        [0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [2, 0, 0, 1, 3]
    ])

    # Buat instance pathfinder
    pathfinder = AStarPathfinder(map_grid)
    
    print("Peta awal:")
    print(map_grid)
    print("\nKeterangan:")
    print("0 = Free space")
    print("1 = Obstacle/Wall")
    print("2 = Start point")
    print("3 = Goal point")
    print("\nMencari jalur...")
    
    # Cari jalur
    path = pathfinder.find_path(debug=True)
    
    if path:
        print("\nJalur ditemukan!")
        print("Koordinat jalur (baris, kolom):")
        for step in path:
            print(f"  {step}")
            
        # Tandai jalur pada peta
        marked_grid = pathfinder.mark_path_on_grid(path)
        print("\nPeta dengan jalur (5 = jalur):")
        print(marked_grid)
    else:
        print("\nTidak ditemukan jalur yang mungkin!")

if __name__ == "__main__":
    main()
    input("\nTekan Enter untuk menutup program...")