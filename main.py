import math
import os
import random
import time

import pygame
import serial

# VIRTUAL DIMENSIONS
MAP_W, MAP_H = 2266, 1275
TOP_BAR_H = 250
SIDEBAR_W = 550
MAP_MARGIN = 50

VIRTUAL_W = MAP_W + SIDEBAR_W
VIRTUAL_H = MAP_H + TOP_BAR_H

# Starting window and adjustable map size
START_W, START_H = 800, 600
USA_WIDTH_MILES = 2800
PX_PER_MILE = (MAP_W - (MAP_MARGIN * 2)) / USA_WIDTH_MILES

# Colors
GOLD = (218, 165, 32)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
CYAN = (0, 180, 220)
SPACE_BLACK = (8, 8, 15)
SIDEBAR_BG = (15, 15, 22)
HEADER_BG = (20, 20, 30)
MAP_COLOR = (100, 105, 145)


class Entry:
    def __init__(self, x, y, accel):
        self.x = x
        self.y = y
        self.accel = accel


class CraterCreator:
    def __init__(self):
        pygame.init()
        if os.path.exists("leaderboard.json"):
            os.remove("leaderboard.json")

        self.screen = pygame.display.set_mode((START_W, START_H), pygame.RESIZABLE)
        self.canvas = pygame.Surface((VIRTUAL_W, VIRTUAL_H))

        self.map_img = pygame.image.load("usa_map.png").convert_alpha()
        self.map_img = pygame.transform.scale(
            self.map_img, (MAP_W - (MAP_MARGIN * 2), MAP_H - (MAP_MARGIN * 2))
        )

        # Fonts
        self.font_header = pygame.font.SysFont("Impact", 70)
        self.font_data_lbl = pygame.font.SysFont("Consolas", 45, bold=True)
        self.font_data_val = pygame.font.SysFont("Consolas", 65, bold=True)

        self.sensor_vel = 0.0
        self.asteroid_vel = 0.0
        self.crater_miles = 0.0
        self.impact_pos = (MAP_W // 2, TOP_BAR_H + (MAP_H // 2))
        self.leaderboard = []
        self.flash_alpha = 0
        self.desktop_mode = False

        self.entry: Entry = Entry(0, 0, 0)

        self.last_time = time.time()

        # Serial connection to ESP32
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=10)
        except serial.SerialException:
            self.desktop_mode = True

    def calculate_crater(self, sensor_vel):
        simulated_kms = sensor_vel * 60.0
        return round(20 + (math.log10(simulated_kms + 1) * 160), 2)

    def trigger_crater(self, vel, pos):
        """Shared logic for placing a crater and updating leaderboard."""
        self.sensor_vel = round(vel, 2)
        self.asteroid_vel = self.sensor_vel * 5.8
        self.crater_miles = self.calculate_crater(self.sensor_vel)
        self.impact_pos = pos
        self.leaderboard.append({"vel": self.crater_miles})
        self.leaderboard = sorted(
            self.leaderboard, key=lambda x: x["vel"], reverse=True
        )[:10]
        self.flash_alpha = 180

    def crater_from_accelerometer(self, accel: float) -> None:
        pos = (
            MAP_MARGIN + random.random() * (MAP_W - 2 * MAP_MARGIN),
            TOP_BAR_H + MAP_MARGIN + random.random() * (MAP_H - 2 * MAP_MARGIN),
        )
        vel = accel * 10
        self.trigger_crater(vel, pos)

    def trigger_random_crater(self):
        """Fire a crater at a random map location with a random velocity."""
        vel = random.uniform(0.5, 5.0)
        pos = (
            MAP_MARGIN + random.random() * (MAP_W - 2 * MAP_MARGIN),
            TOP_BAR_H + MAP_MARGIN + random.random() * (MAP_H - 2 * MAP_MARGIN),
        )
        self.trigger_crater(vel, pos)

    # Physical box dimensions in mm
    BOX_X = 355.6  # 14 inches
    BOX_Y = 228.6  # 9 inches
    WALL_TOLERANCE = 5.0  # mm

    # Sensor Y positions (1/4, 2/4, 3/4 of 9")
    SENSOR_POSITIONS = [57.15, 114.3, 171.45]

    # FOV correction — readings beyond 25deg are stretched, scale them back
    FOV_ACCURATE = 25
    FOV_MAX = 40

    def draw_canvas(self):
        # Map Area
        pygame.draw.rect(self.canvas, MAP_COLOR, (0, TOP_BAR_H, MAP_W, MAP_H))

        # Stat Panel and Leaderboard
        pygame.draw.rect(self.canvas, HEADER_BG, (0, 0, VIRTUAL_W, TOP_BAR_H))
        pygame.draw.rect(self.canvas, SIDEBAR_BG, (MAP_W, TOP_BAR_H, SIDEBAR_W, MAP_H))

        # Separating Lines
        pygame.draw.line(self.canvas, GOLD, (0, TOP_BAR_H), (VIRTUAL_W, TOP_BAR_H), 10)
        pygame.draw.line(self.canvas, GOLD, (MAP_W, TOP_BAR_H), (MAP_W, VIRTUAL_H), 10)

        # Add margins to map
        self.canvas.blit(self.map_img, (MAP_MARGIN, TOP_BAR_H + MAP_MARGIN))

        # Top Bar
        title = self.font_header.render("ORBITAL IMPACT COMMAND", True, GOLD)
        if self.desktop_mode:
            title = self.font_header.render(
                "!!!DEBUG MODE!!! ORBITAL IMPACT COMMAND", True, RED
            )
        self.canvas.blit(title, (50, 20))

        stats = [
            ("SENSOR m/s", f"{self.sensor_vel:.2f}", CYAN),
            ("ASTEROID km/s", f"{self.asteroid_vel:.2f}", GOLD),
            ("CRATER MILES", f"{self.crater_miles:.2f}", WHITE),
        ]

        for i, (l, v, c) in enumerate(stats):
            self.canvas.blit(self.font_data_lbl.render(l, True, c), (50 + i * 750, 110))
            self.canvas.blit(
                self.font_data_val.render(v, True, WHITE), (50 + i * 750, 165)
            )

        # Leaderboard
        lb_title = self.font_header.render("TOP CRATER SIZES", True, GOLD)
        self.canvas.blit(lb_title, (MAP_W + 50, TOP_BAR_H + 40))

        for i in range(10):
            if i < len(self.leaderboard):
                val = self.leaderboard[i]["vel"]
            else:
                val = 0.0

            color = WHITE if val > 0 else (60, 60, 75)
            txt = self.font_data_lbl.render(f"{i + 1}. {val} km", True, color)
            self.canvas.blit(txt, (MAP_W + 60, TOP_BAR_H + 160 + i * 95))

        # Crater Visual
        if self.crater_miles > 0:
            rad_px = (self.crater_miles * PX_PER_MILE) / 2
            overlay = pygame.Surface((VIRTUAL_W, VIRTUAL_H), pygame.SRCALPHA)
            pygame.draw.circle(overlay, (220, 50, 0, 190), self.impact_pos, rad_px)
            self.canvas.blit(overlay, (0, 0))

    def run(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return

            if not self.desktop_mode and self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                print(line)

                if line.startswith("A@E#F:"):
                    val = float(line[6:])
                    self.entry.accel = abs(val)

                if line.startswith("X@E#F:"):
                    val = float(line[6:])
                    # accel.x maps to map x: normalize [-0.25,0.25] -> map pixel range
                    t = max(0.0, min(1.0, (val + 0.25) / 0.5))
                    self.entry.x = MAP_MARGIN + t * (MAP_W - 2 * MAP_MARGIN)

                if line.startswith("Y@E#F:"):
                    val = float(line[6:])
                    # accel.z maps to map y: normalize [-0.25,0.25] -> map pixel range
                    t = max(0.0, min(1.0, (val + 0.25) / 0.5))
                    self.entry.y = TOP_BAR_H + MAP_MARGIN + t * (MAP_H - 2 * MAP_MARGIN)
                    print(
                        f"entry: {self.entry.x=}, {self.entry.y=}, {self.entry.accel=}"
                    )
                    self.trigger_crater(self.entry.accel, [self.entry.x, self.entry.y])

                if line.startswith("B@E#F:"):
                    val = float(line[6:])
                    if not self.is_wall(val):
                        self.bob = val
                        self.last_time = time.time()

                elif line.startswith("C@E#F:"):
                    val = float(line[6:])
                    if not self.is_wall(val):
                        self.carol = val
                        self.last_time = time.time()

                elif line.startswith("V@E#F:"):
                    accel = float(line[6:])
                    vel = accel  # replace with your formula if needed
                    if time.time() - self.last_time >= 5:
                        self.trigger_random_crater()
                    else:
                        try:
                            pos = self.abc_crater_pos(self.alice, self.bob, self.carol)
                            self.trigger_crater(vel, pos)
                        except NotImplementedError:
                            pass

            self.draw_canvas()
            curr_w, curr_h = self.screen.get_size()
            scaled_frame = pygame.transform.smoothscale(self.canvas, (curr_w, curr_h))
            self.screen.blit(scaled_frame, (0, 0))

            if self.flash_alpha > 0:
                f = pygame.Surface((curr_w, curr_h))
                f.fill(WHITE)
                f.set_alpha(self.flash_alpha)
                self.screen.blit(f, (0, 0))
                self.flash_alpha -= 15

            pygame.display.flip()


if __name__ == "__main__":
    CraterCreator().run()
