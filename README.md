# Spinning Wheel Physics Simulation

A physics-based simulation of a spinning wheel with a ball inside, featuring realistic physics including gravity, friction, and collision detection.

## Features

- **Realistic Physics**: Gravity, air resistance, wall friction, and inward radial damping
- **Interactive Controls**: Click or tap to add spin to the wheel
- **Visual Feedback**: Striped wheel and ball for clear rotation visualization
- **Coulomb Friction Model**: Proper static/kinetic friction for realistic ball behavior
- **Responsive Design**: Works on desktop and mobile devices

## How to Use

1. Open `wheel.html` in any modern web browser
2. Click or tap anywhere on the wheel to add a small spin impulse
3. Watch the ball interact with the spinning wheel using realistic physics
4. The ball will roll without slipping when friction allows, and slip when it can't

## Physics Details

- **Gravity**: Downward acceleration (5.0 m/s²)
- **Friction**: Static (μs = 0.45) and kinetic (μk = 0.35) coefficients
- **Collision**: Elastic collision with energy loss (restitution = 0.25)
- **Air Resistance**: Velocity-dependent damping

## Files

- `wheel.html` - Main simulation (HTML5 Canvas + JavaScript)
- `py-wheel/spin_wheel.py` - Python/Pygame version with identical physics

## Live Demo

Visit the [GitHub Pages site](https://[your-username].github.io/[repo-name]/wheel.html) to try the simulation online.

## Technical Notes

Built with vanilla JavaScript and HTML5 Canvas. No external dependencies required. The simulation runs at 60+ FPS and includes proper physics integration with collision detection and response.
