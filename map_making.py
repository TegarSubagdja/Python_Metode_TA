import pygame
import numpy as np

# Konfigurasi grid
GRID_SIZE = 5
CELL_SIZE = 100
WIDTH = GRID_SIZE * CELL_SIZE
HEIGHT = GRID_SIZE * CELL_SIZE

# Warna untuk setiap elemen grid dalam kode HEX
colors = {
    0: "#FFFFFF",  # Ruang kosong (putih)
    1: "#000000",  # Rintangan (hitam)
    2: "#00FF00",  # Start (hijau)
    3: "#FF0000",  # Goal (merah)
    4: "#0000FF"   # Line (biru)
}

# Inisialisasi grid
map_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

# Inisialisasi Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Interactive Grid Editor")
font = pygame.font.SysFont(None, 24)

# Status aktif (0 = ruang kosong, 1 = rintangan, 2 = start, 3 = goal, 4 = line)
active_mode = 1  # Default mode rintangan
lines = []  # Menyimpan semua garis sebagai ((x1, y1), (x2, y2))

def hex_to_rgb(hex_code):
    """Mengubah kode HEX menjadi tuple RGB."""
    hex_code = hex_code.lstrip('#')
    return tuple(int(hex_code[i:i + 2], 16) for i in (0, 2, 4))


def draw_grid(grid):
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            value = grid[row, col]
            color = hex_to_rgb(colors.get(value, "#FFFFFF"))
            pygame.draw.rect(
                screen,
                color,
                (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            )
            # Gambar garis grid
            pygame.draw.rect(
                screen,
                (200, 200, 200),
                (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE),
                1
            )


def display_mode(text):
    mode_text = font.render(f"Mode: {text}", True, (0, 0, 0))
    screen.blit(mode_text, (10, HEIGHT - 30))


def get_cell_center(row, col):
    """Menghitung titik tengah dari grid pada baris dan kolom tertentu."""
    x = col * CELL_SIZE + CELL_SIZE // 2
    y = row * CELL_SIZE + CELL_SIZE // 2
    return x, y


def draw_lines():
    """Menggambar semua garis yang tersimpan di daftar lines."""
    for line in lines:
        pygame.draw.line(screen, hex_to_rgb("#0000FF"), line[0], line[1], 3)  # Width 3


running = True
drawing_line = False  # Apakah sedang menggambar garis
start_cell = None     # Titik awal garis dalam koordinat grid

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Klik kiri untuk menggambar atau menetapkan sel di grid
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_SIZE, y // CELL_SIZE

            if active_mode == 4:  # Mode garis
                if not drawing_line:
                    start_cell = (row, col)  # Titik awal dalam koordinat grid
                    drawing_line = True
                else:
                    end_cell = (row, col)  # Titik akhir dalam koordinat grid
                    # Hitung titik tengah untuk kedua sel
                    start_center = get_cell_center(*start_cell)
                    end_center = get_cell_center(*end_cell)
                    lines.append((start_center, end_center))  # Simpan garis
                    drawing_line = False

            elif active_mode in [1, 2, 3, 0]:  # Mode grid-based
                if active_mode == 1:  # Obstacle mode (toggle)
                    map_grid[row, col] = 0 if map_grid[row, col] == 1 else 1
                elif active_mode == 2:  # Start mode
                    map_grid[map_grid == 2] = 0  # Hapus start lama
                    map_grid[row, col] = 2
                elif active_mode == 3:  # Goal mode
                    map_grid[map_grid == 3] = 0  # Hapus goal lama
                    map_grid[row, col] = 3
                elif active_mode == 0:  # Clear mode
                    map_grid[row, col] = 0

        # Ganti mode dengan kombinasi tombol
        if event.type == pygame.KEYDOWN:
            if event.mod & pygame.KMOD_CTRL:  # Jika Ctrl ditekan
                if event.key == pygame.K_s:  # Ctrl + S untuk start
                    active_mode = 2
                elif event.key == pygame.K_g:  # Ctrl + G untuk goal
                    active_mode = 3
                elif event.key == pygame.K_o:  # Ctrl + O untuk obstacle
                    active_mode = 1
                elif event.key == pygame.K_c:  # Ctrl + C untuk ruang kosong
                    active_mode = 0
                elif event.key == pygame.K_l:  # Ctrl + L untuk line
                    active_mode = 4

    # Gambar ulang layar
    screen.fill(hex_to_rgb("#FFFFFF"))
    draw_grid(map_grid)
    draw_lines()  # Gambar semua garis
    mode_texts = {
        0: "Clear (Ctrl + C)",
        1: "Obstacle (Ctrl + O)",
        2: "Start (Ctrl + S)",
        3: "Goal (Ctrl + G)",
        4: "Line (Ctrl + L)"
    }
    display_mode(mode_texts.get(active_mode, "Unknown"))
    pygame.display.flip()

pygame.quit()
