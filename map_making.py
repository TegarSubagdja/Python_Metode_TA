import os
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
try:
    pygame.init()
except Exception as e:
    print(f"Error initializing Pygame: {e}")
    exit(1)

try:
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
except Exception as e:
    print(f"Error setting up display: {e}")
    exit(1)

pygame.display.set_caption("Interactive Grid Editor")
font = pygame.font.SysFont(None, 24)

# Status aktif (0 = ruang kosong, 1 = rintangan, 2 = start, 3 = goal, 4 = garis, 5 = open, 6 = close, 7 = gray)
active_mode = 1  # Default mode rintangan
lines = []  # Menyimpan semua garis sebagai ((x1, y1), (x2, y2))

# Fungsi untuk mengonversi kode HEX menjadi RGB
def hex_to_rgb(hex_code):
    """Mengubah kode HEX menjadi tuple RGB."""
    hex_code = hex_code.lstrip('#')
    return tuple(int(hex_code[i:i + 2], 16) for i in (0, 2, 4))

# Fungsi untuk menggambar grid
def draw_grid(grid):
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            value = grid[row, col]
            color = hex_to_rgb(colors.get(value, "#FFFFFF"))
            try:
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
            except Exception as e:
                print(f"Error drawing grid: {e}")

# Fungsi untuk menampilkan mode aktif di layar
def display_mode(text):
    try:
        mode_text = font.render(f"Mode: {text}", True, (0, 0, 0))
        screen.blit(mode_text, (10, HEIGHT - 30))
    except Exception as e:
        print(f"Error displaying mode: {e}")

# Fungsi untuk menggambar garis-garis
def draw_lines():
    """Menggambar semua garis yang tersimpan di daftar lines."""
    for line in lines:
        try:
            pygame.draw.line(screen, hex_to_rgb(colors[4]), line[0], line[1], 3)  # Width 3
        except Exception as e:
            print(f"Error drawing line: {e}")

# Fungsi untuk menyimpan gambar
def save_image():
    """Menyimpan grid dan path sebagai file gambar PNG."""
    try:
        # Tentukan nama file dan path
        filename = "grid_path.png"
        filepath = filename

        # Cek apakah file sudah ada
        if os.path.exists(filepath):
            # Jika sudah ada, hapus file lama
            os.remove(filepath)

        # Simpan hasil gambar ke file
        pygame.image.save(screen, filepath)
        print(f"Grid dan path berhasil disimpan sebagai '{filepath}'")
    except Exception as e:
        print(f"Error saving image: {e}")
        print("Unable to save image. Please check your file permissions.")

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
            try:
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
                    # Jika elemen yang diklik memiliki nilai yang sama dengan mode aktif, hapus elemen tersebut
                    if map_grid[row, col] == active_mode:
                        map_grid[row, col] = 0
                    else:
                        # Jika mode aktif berbeda, tetapkan sesuai dengan mode aktif
                        map_grid[row, col] = active_mode
            except Exception as e:
                print(f"Error handling mouse click: {e}")

        # Ganti mode dengan kombinasi tombol
        if event.type == pygame.KEYDOWN:
            try:
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
            except Exception as e:
                print(f"Error handling keyboard input: {e}")

    # Gambar ulang layar
    try:
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
        display_mode(mode_texts.get(active_mode, "Unknown"))

        pygame.display.flip()
    except Exception as e:
        print(f"Error updating display: {e}")

try:
    pygame.quit()
except Exception as e:
    print(f"Error quitting Pygame: {e}")
