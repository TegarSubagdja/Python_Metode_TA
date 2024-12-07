import pygame
import numpy as np

# Konfigurasi grid
GRID_SIZE = 5
CELL_SIZE = 100
WIDTH = GRID_SIZE * CELL_SIZE
HEIGHT = GRID_SIZE * CELL_SIZE

# Warna untuk setiap elemen grid dalam kode HEX
colors = {
    0: "#FFFFFF",  # Ruang kosong
    1: "#17252a",  # Rintangan
    2: "#3aafa9",  # Start
    3: "#FF0021",  # Goal
    4: "#3f6184",  # Garis
    5: "#FFFF00",  # Open List (kuning)
    6: "#FFA500",  # Close List (oranye)
    7: "#778899",  # Warna abu-abu
    8: "#e8175d",  # Warna pink
}

# Inisialisasi grid
map_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

# Inisialisasi Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Interactive Grid Editor")
font = pygame.font.SysFont(None, 24)

# Status aktif (0 = ruang kosong, 1 = rintangan, 2 = start, 3 = goal, 4 = garis, 5 = open, 6 = close, 7 = gray)
active_mode = 1  # Default mode rintangan
lines = []  # Menyimpan semua garis sebagai ((x1, y1), (x2, y2))

# Variabel untuk mengontrol apakah koordinat ditampilkan atau tidak
show_coordinates = False

# Fungsi untuk mengonversi kode HEX menjadi RGB
def hex_to_rgb(hex_code):
    """Mengubah kode HEX menjadi tuple RGB."""
    hex_code = hex_code.lstrip('#')
    return tuple(int(hex_code[i:i + 2], 16) for i in (0, 2, 4))

# Fungsi untuk menggambar grid, termasuk koordinat jika diaktifkan
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
            # Jika koordinat diaktifkan, tampilkan koordinat di pojok kanan atas
            if show_coordinates:
                coord_text = font.render(f"{col},{row}", True, (0, 0, 0))
                screen.blit(coord_text, (col * CELL_SIZE + CELL_SIZE - 40, row * CELL_SIZE + 5))

# Fungsi untuk menampilkan mode aktif di layar
def display_mode(text):
    mode_text = font.render(f"Mode: {text}", True, (0, 0, 0))
    screen.blit(mode_text, (10, HEIGHT - 30))

# Fungsi untuk menggambar garis-garis
def draw_lines():
    """Menggambar semua garis yang tersimpan di daftar lines."""
    for line in lines:
        pygame.draw.line(screen, hex_to_rgb(colors[4]), line[0], line[1], 3)  # Width 3

# Fungsi untuk menyimpan gambar
def save_image():
    """Menyimpan grid dan path sebagai file gambar PNG."""
    filename = "grid_path.png"
    pygame.image.save(screen, filename)
    print(f"Grid dan path berhasil disimpan sebagai '{filename}'")

# Program utama
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
                    start_center = (start_cell[1] * CELL_SIZE + CELL_SIZE // 2, start_cell[0] * CELL_SIZE + CELL_SIZE // 2)
                    end_center = (end_cell[1] * CELL_SIZE + CELL_SIZE // 2, end_cell[0] * CELL_SIZE + CELL_SIZE // 2)
                    lines.append((start_center, end_center))  # Simpan garis
                    drawing_line = False

            elif active_mode in [1, 2, 3, 0, 5, 6, 7, 8]:  # Mode grid-based
                if active_mode == 1:  # Mode rintangan
                    map_grid[row, col] = 0 if map_grid[row, col] == 1 else 1
                elif active_mode == 2:  # Mode start
                    map_grid[map_grid == 2] = 0  # Hapus start lama
                    map_grid[row, col] = 2
                elif active_mode == 3:  # Mode goal
                    map_grid[map_grid == 3] = 0  # Hapus goal lama
                    map_grid[row, col] = 3
                elif active_mode == 0:  # Mode kosong (clear)
                    map_grid[row, col] = 0
                elif active_mode == 5:  # Mode open list
                    map_grid[row, col] = 5
                elif active_mode == 6:  # Mode close list
                    map_grid[row, col] = 6
                elif active_mode == 7:  # Mode abu-abu
                    map_grid[row, col] = 7
                elif active_mode == 8:  # Mode pink
                    map_grid[row, col] = 8

        # Ganti mode dengan kombinasi tombol
        if event.type == pygame.KEYDOWN:
            if pygame.key.get_mods() & pygame.KMOD_CTRL:  # Jika Ctrl ditekan
                if event.key == pygame.K_s:  # Ctrl + S untuk start
                    active_mode = 2
                elif event.key == pygame.K_g:  # Ctrl + G untuk goal
                    active_mode = 3
                elif event.key == pygame.K_o:  # Ctrl + O untuk obstacle
                    active_mode = 1
                elif event.key == pygame.K_c:  # Ctrl + C untuk ruang kosong
                    active_mode = 0
                elif event.key == pygame.K_l:  # Ctrl + L untuk garis
                    active_mode = 4
                elif event.key == pygame.K_u:  # Ctrl + U untuk open list
                    active_mode = 5
                elif event.key == pygame.K_x:  # Ctrl + X untuk close list
                    active_mode = 6
                elif event.key == pygame.K_e:  # Ctrl + E untuk warna abu-abu
                    active_mode = 7
                elif event.key == pygame.K_q:  # Ctrl + Q untuk warna pink
                    active_mode = 8
                elif event.key == pygame.K_p:  # Ctrl + P untuk save
                    save_image()
                elif event.key == pygame.K_r:  # Ctrl + R untuk reset grid
                    map_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
                    lines = []
                elif event.key == pygame.K_i:  # Ctrl + I untuk menampilkan koordinat
                    show_coordinates = not show_coordinates

    # Gambar ulang layar
    screen.fill(hex_to_rgb("#FFFFFF"))
    draw_grid(map_grid)
    draw_lines()  # Gambar semua garis

    # Tampilkan teks mode aktif
    mode_texts = {
        0: "Clear (Ctrl + C)",
        1: "Obstacle (Ctrl + O)",
        2: "Start (Ctrl + S)",
        3: "Goal (Ctrl + G)",
        4: "Line (Ctrl + L)",
        5: "Open List (Ctrl + U)",
        6: "Close List (Ctrl + X)",
        7: "Gray (Ctrl + E)",
        8: "Pink (Ctrl + Q)"
    }
    # display_mode(mode_texts.get(active_mode, "Unknown"))

    pygame.display.flip()

pygame.quit()
