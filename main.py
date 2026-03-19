import math
import os
import random

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
WHITE = (255, 255, 255)
CYAN = (0, 180, 220)
SPACE_BLACK = (8, 8, 15)
SIDEBAR_BG = (15, 15, 22)
HEADER_BG = (20, 20, 30)
MAP_COLOR = (100, 105, 145)


class CraterCreator:
    def __init__(self):

        # Reset leaderboard (maybe remove in final, just good for development)
        pygame.init()
        if os.path.exists("leaderboard.json"):
            os.remove("leaderboard.json")

        self.screen = pygame.display.set_mode((START_W, START_H), pygame.RESIZABLE)
        self.canvas = pygame.Surface((VIRTUAL_W, VIRTUAL_H))

        self.map_img = pygame.image.load("usa_map.png").convert_alpha()
        # Scale map to fit in the margins
        self.map_img = pygame.transform.scale(
            self.map_img, (MAP_W - (MAP_MARGIN * 2), MAP_H - (MAP_MARGIN * 2))
        )

        # Random fonts I felt fit
        self.font_header = pygame.font.SysFont("Impact", 70)
        self.font_data_lbl = pygame.font.SysFont("Consolas", 45, bold=True)
        self.font_data_val = pygame.font.SysFont("Consolas", 65, bold=True)

        self.sensor_vel = 0.0
        self.asteroid_vel = 0.0
        self.crater_miles = 0.0
        self.impact_pos = (MAP_W // 2, TOP_BAR_H + (MAP_H // 2))
        self.leaderboard = []
        self.flash_alpha = 0

        # Serial connection to ESP32
        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)

    def calculate_crater(self, sensor_vel):
        if sensor_vel < 0.1:
            return 0
        simulated_kms = sensor_vel * 6.0
        # Equation Gemini made for me to scale realistically
        return round(20 + (math.log10(simulated_kms + 1) * 160), 2)

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

        # Top Bar (Mission Control)
        title = self.font_header.render("ORBITAL IMPACT COMMAND", True, GOLD)
        self.canvas.blit(title, (50, 20))

        stats = [
            ("SENSOR m/s", f"{self.sensor_vel:.2f}", CYAN),
            ("ASTEROID km/s", f"{self.asteroid_vel:.2f}", GOLD),
            ("CRATER MILES", f"{self.crater_miles:.2f}", WHITE),
        ]

        # Enumerates stats onto text field (AI-Gen)
        for i, (l, v, c) in enumerate(stats):
            self.canvas.blit(self.font_data_lbl.render(l, True, c), (50 + i * 750, 110))
            self.canvas.blit(
                self.font_data_val.render(v, True, WHITE), (50 + i * 750, 165)
            )

        # Leaderboard
        lb_title = self.font_header.render("TOP CRATER SIZES", True, GOLD)
        self.canvas.blit(lb_title, (MAP_W + 50, TOP_BAR_H + 40))

        # Make unfilled leaderboards grey 0.0 slots
        for i in range(10):
            if i < len(self.leaderboard):
                val = self.leaderboard[i]["vel"]
            else:
                val = 0.0

            if val > 0:
                color = WHITE
            else:
                color = (60, 60, 75)

            txt = self.font_data_lbl.render(f"{val} km", True, color)
            self.canvas.blit(txt, (MAP_W + 60, TOP_BAR_H + 160 + i * 95))

        # Crater Visual
        if self.crater_miles > 0:
            rad_px = (self.crater_miles * PX_PER_MILE) / 2
            # Makes transparent red circle for impact
            overlay = pygame.Surface((VIRTUAL_W, VIRTUAL_H), pygame.SRCALPHA)
            pygame.draw.circle(overlay, (220, 50, 0, 190), self.impact_pos, rad_px)
            self.canvas.blit(overlay, (0, 0))

    def run(self):
        while True:
            for event in pygame.event.get():
                # Handle memory leak stuff my guide told me to do
                if event.type == pygame.QUIT:
                    return

                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return

            # Read serial data from ESP32
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("V@E#F:"):
                    try:
                        vel = float(line[6:])
                        self.sensor_vel = round(vel, 2)
                        self.asteroid_vel = self.sensor_vel * 5.8
                        self.crater_miles = self.calculate_crater(self.sensor_vel)

                        # Generate random target (Change if we do the yz matrix stuff Asher was talking about)
                        self.impact_pos = (
                            random.randint(MAP_MARGIN + 200, MAP_W - MAP_MARGIN - 200),
                            random.randint(
                                TOP_BAR_H + MAP_MARGIN + 200, VIRTUAL_H - MAP_MARGIN - 200
                            ),
                        )

                        # Add to leaderboard if necessary and set flash to occur
                        self.leaderboard.append({"vel": self.crater_miles})
                        self.leaderboard = sorted(
                            self.leaderboard, key=lambda x: x["vel"], reverse=True
                        )[:10]
                        self.flash_alpha = 180
                    except ValueError:
                        pass

            # Update the screen and frame settings
            self.draw_canvas()
            curr_w, curr_h = self.screen.get_size()
            scaled_frame = pygame.transform.smoothscale(self.canvas, (curr_w, curr_h))
            self.screen.blit(scaled_frame, (0, 0))

            # Flash handle
            if self.flash_alpha > 0:
                f = pygame.Surface((curr_w, curr_h))
                f.fill(WHITE)
                f.set_alpha(self.flash_alpha)
                self.screen.blit(f, (0, 0))
                self.flash_alpha -= 15

            pygame.display.flip()


if __name__ == "__main__":
    CraterCreator().run()
