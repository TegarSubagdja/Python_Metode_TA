import pygame

# Pygame Initialization
pygame.init()

# Screen settings
WIDTH, HEIGHT = 400, 400
TILE_SIZE = 100  # Ukuran tile/grid
ROWS, COLS = HEIGHT // TILE_SIZE, WIDTH // TILE_SIZE  # Jumlah baris dan kolom

# Colors
WHITE = (255, 255, 255)
GREY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Initialize screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pygame Coordinate System")
clock = pygame.time.Clock()

# Poin-poin untuk membuktikan
grid_points = [(0, 0), (1,2)]  # (row, col)

def draw_grid():
    """Draw the grid."""
    for row in range(ROWS):
        for col in range(COLS):
            x, y = col * TILE_SIZE, row * TILE_SIZE  # Convert (row, col) to (x, y)
            pygame.draw.rect(screen, WHITE, (x, y, TILE_SIZE, TILE_SIZE))
            pygame.draw.rect(screen, GREY, (x, y, TILE_SIZE, TILE_SIZE), 1)

def draw_points(points):
    """Draw points on the grid."""
    for row, col in points:
        x = row * TILE_SIZE
        y = col * TILE_SIZE
        pygame.draw.rect(screen, RED, (x, y, TILE_SIZE, TILE_SIZE))

def main():
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(WHITE)
        draw_grid()
        draw_points(grid_points)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
