import serial
import pygame
import threading
import math
import cmath
import numpy as np


SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

WIDTH, HEIGHT = 1200, 1200
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE = 30  # pixels per unit (adjust for zoom)
GRID_SIZE = 100  # resolution of complex plane grid
RESP_HEIGHT = 600

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT + RESP_HEIGHT))
pygame.display.set_caption("Complex Function Visualizer")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 14)

# Shared token data structure
tokens = []  # Each token: (x, y, order)
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
                        x, y, order = float(parts[0]), float(parts[1]), int(parts[2])
                        new_tokens.append((x, y, order))
            except ValueError:
                continue  # Skip malformed lines
        with tokens_lock:
            tokens = new_tokens

# Start serial reader thread
threading.Thread(target=read_serial, daemon=True).start()

def compute_transfer(z, tokens):
    H = 1
    for x, y, order in tokens:
        root = complex(x, y)
        if (z - root != 0):
            H *= (z - root) ** order
    return H

def draw_complex_plane(screen, tokens):
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            re = -16 + (32 * i / GRID_SIZE)
            im = -16 + (32 * j / GRID_SIZE)
            z = complex(re, im)
            H = compute_transfer(z, tokens)
            mag = abs(H)
            phase = cmath.phase(H)

            brightness = min(100, 40 * math.log1p(mag))  # Scale to 0–100
            hue = ((phase + math.pi) / (2 * math.pi)) * 360  # Scale to 0–360

            color = pygame.Color(0)
            color.hsva = (hue, 100, brightness, 100)

            x = int(CENTER_X + re * SCALE)
            y = int(CENTER_Y - im * SCALE)
            if 0 <= x < WIDTH and 0 <= y < HEIGHT:
                screen.set_at((x, y), color)
                
def draw_frequency_response(screen, tokens):
    freqs = np.logspace(-1, 1, WIDTH)
    mags = []
    phases = []
    for omega in freqs:
        z = cmath.exp(1j * omega)+ 1e-6
        H = compute_transfer(z, tokens)
        mags.append(20 * math.log10(abs(H)))
        phases.append(cmath.phase(H))

    max_mag = max(mags)
    min_mag = min(mags)

    for i in range(1, WIDTH):
        mag1 = int(HEIGHT + RESP_HEIGHT - ((mags[i - 1] - min_mag) * RESP_HEIGHT / (max_mag - min_mag + 1e-6)))
        mag2 = int(HEIGHT + RESP_HEIGHT - ((mags[i] - min_mag) * RESP_HEIGHT / (max_mag - min_mag + 1e-6)))
        pygame.draw.line(screen, (255, 255, 0), (i - 1, mag1), (i, mag2))

        phase1 = int(HEIGHT + RESP_HEIGHT - ((phases[i - 1] + math.pi) * RESP_HEIGHT / (2 * math.pi)))
        phase2 = int(HEIGHT + RESP_HEIGHT - ((phases[i] + math.pi) * RESP_HEIGHT / (2 * math.pi)))
        pygame.draw.line(screen, (0, 255, 255), (i - 1, phase1), (i, phase2))

    # Draw vertical lines for poles and zeros
    for x, y, order in tokens:
        freq = math.atan2(y, x)  # Approximate frequency of effect
        if freq <= 0:
            continue
        x_pos = int(WIDTH * (math.log10(freq) - math.log10(0.1)) / (math.log10(np.pi) - math.log10(0.1)))
        if 0 <= x_pos < WIDTH:
            color = (0, 255, 0) if order > 0 else (255, 0, 0)
            pygame.draw.line(screen, color, (x_pos, HEIGHT), (x_pos, HEIGHT + RESP_HEIGHT), 1)

    # Draw axes
    pygame.draw.line(screen, (200, 200, 200), (0, HEIGHT + RESP_HEIGHT), (WIDTH, HEIGHT + RESP_HEIGHT))  # x-axis
    pygame.draw.line(screen, (200, 200, 200), (0, HEIGHT), (0, HEIGHT + RESP_HEIGHT))  # y-axis left

    # Add axis labels and ticks
    mag_label = font.render("Magnitude (dB)", True, (255, 255, 0))
    phase_label = font.render("Phase (rad)", True, (0, 255, 255))
    screen.blit(mag_label, (10, HEIGHT + 5))
    screen.blit(phase_label, (10, HEIGHT + 25))

    # Tick labels for frequency (log scale)
    for decade in range(-1, 2):
        x_pos = int(WIDTH * (math.log10(10**decade) - math.log10(0.1)) / (math.log10(np.pi) - math.log10(0.1)))
        freq_label = font.render(f"10^{decade}", True, (200, 200, 200))
        screen.blit(freq_label, (x_pos, HEIGHT + RESP_HEIGHT - 20))

    # Tick labels for magnitude and phase
    for dB in range(int(min_mag), int(max_mag), 10):
        y_pos = int(HEIGHT + RESP_HEIGHT - ((dB - min_mag) * RESP_HEIGHT / (max_mag - min_mag + 1e-6)))
        db_label = font.render(f"{dB}", True, (255, 255, 0))
        screen.blit(db_label, (40, y_pos))

    for rad in range(-3, 4):
        y_pos = int(HEIGHT + RESP_HEIGHT - ((rad + math.pi) * RESP_HEIGHT / (2 * math.pi)))
        rad_label = font.render(f"{rad}", True, (0, 255, 255))
        screen.blit(rad_label, (70, y_pos))

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((30, 30, 30))

    with tokens_lock:
        draw_complex_plane(screen, tokens)

        # Draw center lines
        pygame.draw.line(screen, (100, 100, 255), (CENTER_X, 0), (CENTER_X, HEIGHT), 1)  # Vertical center line
        pygame.draw.line(screen, (100, 100, 255), (0, CENTER_Y), (WIDTH, CENTER_Y), 1)    # Horizontal center line

        # Draw tokens
        for x, y, order in tokens:
            pos = (int(CENTER_X + x * SCALE), int(CENTER_Y - y * SCALE))
            radius = 6 + abs(order) * 2
            if order > 0:
                pygame.draw.circle(screen, (0, 255, 0), pos, radius, 2)  # Zero
            elif order < 0:
                pygame.draw.line(screen, (255, 0, 0), (pos[0] - radius, pos[1] - radius), (pos[0] + radius, pos[1] + radius), 2)
                pygame.draw.line(screen, (255, 0, 0), (pos[0] - radius, pos[1] + radius), (pos[0] + radius, pos[1] - radius), 2)  # Pole

        draw_frequency_response(screen, tokens)

    pygame.display.flip()
    clock.tick(10)

pygame.quit()
