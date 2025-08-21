import math
import random
import sys
from typing import Tuple

import pygame


class Config:
    window_size_px: int = 820
    background_color: Tuple[int, int, int] = (15, 17, 21)
    rim_inner_color: Tuple[int, int, int] = (27, 32, 48)
    rim_outer_color: Tuple[int, int, int] = (43, 49, 70)
    spoke_color_a: Tuple[int, int, int] = (255, 255, 255)
    spoke_color_b: Tuple[int, int, int] = (220, 225, 238)

    pixels_per_meter: float = 200.0
    gravity_m_s2: float = 5.0
    air_drag_per_second: float = 0.10
    wall_restitution: float = 0.25
    mu_static: float = 0.45
    mu_kinetic: float = 0.35
    inward_radial_accel_m_s2: float = 1.8
    wheel_angular_friction_per_second: float = 0.25
    ball_angular_friction_per_second: float = 2.0
    ball_spin_gain: float = 6.0
    rim_thickness_px: float = 24.0
    ball_radius_px: float = 10.0
    spoke_count: int = 36
    # Smaller tap impulse for slow spin
    spin_impulse_min: float = 0.6
    spin_impulse_max: float = 1.2
    ball_boost_speed_m_s: float = 2.2


class Ball:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0


class SpinningWheelSimulation:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.width = cfg.window_size_px
        self.height = cfg.window_size_px
        self.center_x = self.width * 0.5
        self.center_y = self.height * 0.5

        visual_radius = min(self.width, self.height) * 0.45
        # Inner rim radius (where the ball makes contact)
        self.wheel_radius_px = max(40.0, visual_radius - cfg.rim_thickness_px)

        self.wheel_angle = 0.0
        self.wheel_omega = 0.4

        self.ball = Ball()
        self.ball_angle: float = 0.0
        self.ball_omega: float = 0.0
        self.place_ball_near_rim()

    def resize(self, new_width: int, new_height: int) -> None:
        self.width = new_width
        self.height = new_height
        self.center_x = self.width * 0.5
        self.center_y = self.height * 0.5

        visual_radius = min(self.width, self.height) * 0.45
        self.wheel_radius_px = max(40.0, visual_radius - self.cfg.rim_thickness_px)

        # Keep ball within rim
        self.place_ball_near_rim()

    def place_ball_near_rim(self) -> None:
        angle = random.random() * math.tau
        r = self.wheel_radius_px - self.cfg.ball_radius_px - 1.0
        ux = math.cos(angle)
        uy = math.sin(angle)
        self.ball.x = self.center_x + ux * r
        self.ball.y = self.center_y + uy * r

        tangential_x = -uy
        tangential_y = ux
        boost = self.cfg.ball_boost_speed_m_s * self.cfg.pixels_per_meter
        self.ball.vx = tangential_x * boost
        self.ball.vy = tangential_y * boost

    def randomize_spin(self) -> None:
        # Add a small positive angular impulse; do not reset state
        self.wheel_omega += random.uniform(
            self.cfg.spin_impulse_min, self.cfg.spin_impulse_max
        )

    def update(self, dt: float) -> None:
        g = self.cfg.gravity_m_s2 * self.cfg.pixels_per_meter
        self.ball.vy += g * dt

        air_k = max(0.0, 1.0 - self.cfg.air_drag_per_second * dt)
        self.ball.vx *= air_k
        self.ball.vy *= air_k

        self.ball.x += self.ball.vx * dt
        self.ball.y += self.ball.vy * dt

        omega_k = max(0.0, 1.0 - self.cfg.wheel_angular_friction_per_second * dt)
        self.wheel_omega *= omega_k
        self.wheel_angle += self.wheel_omega * dt

        # Ball rotational damping
        ball_spin_k = max(0.0, 1.0 - self.cfg.ball_angular_friction_per_second * dt)
        self.ball_omega *= ball_spin_k

        # Collision with inner rim
        dx = self.ball.x - self.center_x
        dy = self.ball.y - self.center_y
        dist = math.hypot(dx, dy) or 1e-6
        nx = dx / dist
        ny = dy / dist
        allowed = self.wheel_radius_px - self.cfg.ball_radius_px

        if dist > allowed:
            penetration = dist - allowed
            self.ball.x -= nx * penetration
            self.ball.y -= ny * penetration

            v_normal = self.ball.vx * nx + self.ball.vy * ny
            tx = -ny
            ty = nx
            v_tangential = self.ball.vx * tx + self.ball.vy * ty

            v_n = v_normal
            if v_n > 0.0:
                v_n = -v_n * self.cfg.wall_restitution

            surface_speed = self.wheel_omega * allowed
            # Slip speed including ball spin
            slip = v_tangential - surface_speed - self.ball_omega * allowed

            # Approximate normal force magnitude (unit mass)
            N = max(0.0, (v_tangential * v_tangential) / max(1.0, allowed) + max(0.0, (self.cfg.gravity_m_s2 * self.cfg.pixels_per_meter) * ny))
            Jmax_static = self.cfg.mu_static * N * dt
            Jmax_kinetic = self.cfg.mu_kinetic * N * dt

            # Static friction impulse to eliminate slip: J_s = -slip / 3 (solid disk)
            J_static = -slip / 3.0
            if abs(J_static) <= Jmax_static:
                Jt = J_static
            else:
                Jt = -math.copysign(Jmax_kinetic, slip)

            v_t = v_tangential + Jt

            self.ball.vx = v_n * nx + v_t * tx
            self.ball.vy = v_n * ny + v_t * ty

            inward_accel = self.cfg.inward_radial_accel_m_s2 * self.cfg.pixels_per_meter
            self.ball.vx += -nx * inward_accel * dt
            self.ball.vy += -ny * inward_accel * dt

            # Update ball rotation from friction impulse: dOmega = -2 * Jt / R (solid disk)
            R = max(1.0, self.cfg.ball_radius_px)
            self.ball_omega += -2.0 * Jt / R

        # Integrate ball rotation
        self.ball_angle += self.ball_omega * dt

    def draw(self, screen: pygame.Surface) -> None:
        screen.fill(self.cfg.background_color)

        # Wheel (striped ring using alternating wedges)
        outer_radius = self.wheel_radius_px + self.cfg.rim_thickness_px
        inner_radius = self.wheel_radius_px

        # Base outer disk
        pygame.draw.circle(
            screen,
            self.cfg.rim_outer_color,
            (int(self.center_x), int(self.center_y)),
            int(outer_radius),
        )

        # Alternating translucent wedges
        step = math.tau / self.cfg.spoke_count
        for i in range(self.cfg.spoke_count):
            a0 = self.wheel_angle + i * step
            a1 = self.wheel_angle + (i + 1) * step
            x0 = self.center_x + math.cos(a0) * outer_radius
            y0 = self.center_y + math.sin(a0) * outer_radius
            x1 = self.center_x + math.cos(a1) * outer_radius
            y1 = self.center_y + math.sin(a1) * outer_radius
            color = (255, 255, 255, 40) if i % 2 == 0 else (255, 255, 255, 80)
            wedge = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
            pygame.draw.polygon(
                wedge,
                color,
                [(self.center_x, self.center_y), (x0, y0), (x1, y1)],
            )
            screen.blit(wedge, (0, 0))

        # Inner hole to form the ring
        pygame.draw.circle(
            screen,
            self.cfg.background_color,
            (int(self.center_x), int(self.center_y)),
            int(inner_radius),
        )

        # Optional thin spokes overlay
        self._draw_spokes(screen, radius=outer_radius - self.cfg.rim_thickness_px * 0.5)

        # Center hub
        hub_radius = max(10, int(self.cfg.rim_thickness_px * 0.4))
        pygame.draw.circle(screen, (58, 65, 90), (int(self.center_x), int(self.center_y)), hub_radius)

        # Ball body
        pygame.draw.circle(
            screen,
            (255, 255, 255),
            (int(self.ball.x), int(self.ball.y)),
            int(self.cfg.ball_radius_px),
        )
        pygame.draw.circle(
            screen,
            (0, 0, 0),
            (int(self.ball.x), int(self.ball.y)),
            int(self.cfg.ball_radius_px),
            1,
        )

        # Ball stripes (two orthogonal diameters rotated by ball_angle)
        r = self.cfg.ball_radius_px
        ca = math.cos(self.ball_angle)
        sa = math.sin(self.ball_angle)
        ex0 = self.ball.x - ca * r
        ey0 = self.ball.y - sa * r
        ex1 = self.ball.x + ca * r
        ey1 = self.ball.y + sa * r
        pygame.draw.line(screen, (31, 36, 52), (ex0, ey0), (ex1, ey1), width=3)
        mx0 = self.ball.x + sa * r
        my0 = self.ball.y - ca * r
        mx1 = self.ball.x - sa * r
        my1 = self.ball.y + ca * r
        pygame.draw.line(screen, (31, 36, 52), (mx0, my0), (mx1, my1), width=3)

    def _draw_spokes(self, screen: pygame.Surface, radius: float) -> None:
        step = math.tau / self.cfg.spoke_count
        angle = self.wheel_angle
        for i in range(self.cfg.spoke_count):
            a = angle + i * step
            ca = math.cos(a)
            sa = math.sin(a)
            x0 = self.center_x + ca * (radius * 0.60)
            y0 = self.center_y + sa * (radius * 0.60)
            x1 = self.center_x + ca * (radius * 0.98)
            y1 = self.center_y + sa * (radius * 0.98)
            color = self.cfg.spoke_color_a if i % 2 == 0 else self.cfg.spoke_color_b
            pygame.draw.line(screen, color, (x0, y0), (x1, y1), width=2)


def main() -> None:
    pygame.init()
    pygame.display.set_caption("Spinning Wheel with Ball (Gravity + Inward Friction)")
    cfg = Config()
    screen = pygame.display.set_mode((cfg.window_size_px, cfg.window_size_px), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    sim = SpinningWheelSimulation(cfg)

    running = True
    # Warm-up for smoother first frame
    clock.tick(60)

    while running:
        dt_ms = clock.tick(120)  # Cap to ~120 FPS
        # Clamp dt for stability on hiccups
        dt_ms = max(0, min(32, dt_ms))
        dt = dt_ms / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                sim.resize(event.w, event.h)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                sim.randomize_spin()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    sim.randomize_spin()
                elif event.key == pygame.K_r:
                    sim.place_ball_near_rim()

        sim.update(dt)
        sim.draw(screen)
        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()

