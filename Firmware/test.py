import serial
import pygame
import threading

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

WIDTH, HEIGHT = 1600, 1600
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE = 50  # pixels per unit (adjust for zoom)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Token Position Visualizer")
clock = pygame.time.Clock()

# Shared token data structure
tokens = []
tokens_lock = threading.Lock()

# Serial reading thread
def read_serial():
    global tokens
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    while True:
        line = ser.readline().decode('utf-8').strip()  # Read and decode line
        new_tokens = []
        if line:
            try:
                entries = line.split(',')
                for entry in entries:
                    parts = entry.strip().split()
                    if len(parts) >= 3:
                        x1, y1 = float(parts[0]), float(parts[1])
                        new_tokens.append((x1, y1))
            except ValueError:
                continue  # Skip malformed lines
        with tokens_lock:
            tokens = new_tokens

# Start serial reader thread
threading.Thread(target=read_serial, daemon=True).start()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((30, 30, 30))
    pygame.draw.line(screen, (100, 100, 255), (CENTER_X, 0), (CENTER_X, HEIGHT), 1)
    pygame.draw.line(screen, (100, 100, 255), (0, CENTER_Y), (WIDTH, CENTER_Y), 1)

    with tokens_lock:
        for token in tokens:
            (x1, y1) = token
            pos1 = (int(CENTER_X + x1 * SCALE), int(CENTER_Y - y1 * SCALE))
            pygame.draw.circle(screen, (0, 200, 0), pos1, 6)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
