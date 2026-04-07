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

        # Sensor values
        self.alice = 0.0
        self.bob = 0.0
        self.carol = 0.0
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

    def trigger_random_crater(self):
        """Fire a crater at a random map location with a random velocity."""
        vel = random.uniform(0.5, 5.0)
        pos = (
            MAP_MARGIN + random.random() * (MAP_W - 2 * MAP_MARGIN),
            TOP_BAR_H + MAP_MARGIN + random.random() * (MAP_H - 2 * MAP_MARGIN),
        )
        self.trigger_crater(vel, pos)

    # Physical box dimensions in mm
    BOX_X = 355.6   # 14 inches
    BOX_Y = 228.6   # 9 inches
    WALL_TOLERANCE = 5.0  # mm

    # Sensor Y positions (1/4, 2/4, 3/4 of 9")
    SENSOR_POSITIONS = [57.15, 114.3, 171.45]

    # FOV correction — readings beyond 25deg are stretched, scale them back
    FOV_ACCURATE = 25
    FOV_MAX = 40

    def is_wall(self, reading):
        """True if reading is at or near the far wall."""
        return reading >= (self.BOX_X - self.WALL_TOLERANCE)

    def correct_fov(self, reading):
        """
        Fringe readings (25-40 deg FOV) report further than reality.
        We don't know the angle directly, but larger readings from nearby
        objects are a proxy. Scale readings down proportionally in the
        fringe zone. This is a best-effort correction for a kids exhibit.
        """
        # Fringe effect starts to matter beyond ~60% of max depth
        fringe_start = self.BOX_X * 0.6
        if reading <= fringe_start:
            return reading
        # Linearly scale back up to ~25% reduction at max depth
        t = (reading - fringe_start) / (self.BOX_X - fringe_start)
        correction = 1.0 - (t * 0.25)
        return reading * correction

    def trilaterate(self, sensors):
        """
        Given a list of (y_pos, distance) pairs, find best X,Y estimate.
        Uses least-squares if 3 sensors, direct solve if 2, 
        and falls back to single sensor if only 1.
        """
        if len(sensors) == 0:
            return None

        if len(sensors) == 1:
            # Only one sensor — we know distance but not angle,
            # so place it directly in front of that sensor
            sy, d = sensors[0]
            return (d, sy)

        if len(sensors) == 2:
            # Two circles — find intersection, pick point with x > 0
            (y1, r1), (y2, r2) = sensors
            x1, x2 = 0, 0  # both sensors at x=0
            d = abs(y2 - y1)
            if d == 0 or d > r1 + r2:
                # No intersection, take midpoint at average distance
                return ((r1 + r2) / 2, (y1 + y2) / 2)
            a = (r1**2 - r2**2 + d**2) / (2 * d)
            h_sq = r1**2 - a**2
            h = math.sqrt(max(h_sq, 0))
            mid_y = y1 + a * (y2 - y1) / d
            # Two candidate x positions — take positive one (into the box)
            x_candidate = h
            return (x_candidate, mid_y)

        # Three sensors — least squares trilateration
        # Linearize: subtract last equation from first two to remove x^2, y^2
        (y1, r1), (y2, r2), (y3, r3) = sensors
        # All sensors at x=0
        A = []
        b = []
        for (ya, ra), (yb, rb) in [((y1,r1),(y3,r3)), ((y2,r2),(y3,r3))]:
            A.append([0 - 0, 2*(yb - ya)])  # 2*(xb-xa), 2*(yb-ya) — x terms cancel
            b.append(rb**2 - ra**2 - yb**2 + ya**2)

        # Solve 2x2 system manually
        a00, a01 = A[0]
        a10, a11 = A[1]
        det = a00*a11 - a01*a10

        if abs(det) < 1e-6:
            # Degenerate — fall back to two-sensor solve
            return self.trilaterate(sensors[:2])

        # Since sensors are all at x=0, x terms vanish — solve for y first
        # then back-substitute for x using first sensor
        y_est = (b[0]*a10 - b[1]*a00) / (a01*a10 - a11*a00) if (a01*a10 - a11*a00) != 0 else (y1+y2+y3)/3
        x_sq = r1**2 - (y_est - y1)**2
        x_est = math.sqrt(max(x_sq, 0))

        return (x_est, y_est)

    def abc_crater_pos(self, alice, bob, carol):
        """
        Convert sensor readings to a canvas (x, y) position.
        Filters wall readings, corrects FOV, trilaterates, 
        then maps to canvas coordinates.
        """
        raw = [(self.SENSOR_POSITIONS[0], alice),
               (self.SENSOR_POSITIONS[1], bob),
               (self.SENSOR_POSITIONS[2], carol)]

        # Filter out wall readings and zero/unset values
        valid = [(sy, self.correct_fov(d)) for sy, d in raw
                 if d > 0 and not self.is_wall(d)]

        if not valid:
            return None

        result = self.trilaterate(valid)
        if result is None:
            return None

        px, py = result

        # Clamp to box bounds
        px = max(0, min(px, self.BOX_X))
        py = max(0, min(py, self.BOX_Y))

        # Normalize to 0-1 then map to canvas
        x_pct = px / self.BOX_X
        y_pct = py / self.BOX_Y

        cx = MAP_MARGIN + x_pct * (MAP_W - 2 * MAP_MARGIN)
        cy = TOP_BAR_H + MAP_MARGIN + y_pct * (MAP_H - 2 * MAP_MARGIN)

        return (cx, cy)

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
                    if not self.is_wall(val):
                        self.alice = val
                        self.last_time = time.time()

                elif line.startswith("B@E#F:"):
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
