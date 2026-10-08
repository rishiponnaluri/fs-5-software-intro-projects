"""Run and graph the Formula Slug PID car speed simulation."""

import matplotlib.pyplot as plt
from pid import (
    make_car,
    update,
    calculate_desired_acceleration,
    acceleration_to_throttle_percentage,
)

# Start with P-only (set K_I and K_D to 0), then try all 3 terms.
K_P = 0.8
K_I = 0.3
K_D = 0.05
STEPS = 550

car = make_car(desired_v=20.0, dt=0.1)

# Track the car at each time step.
times = [car["t"]]
velocities = [car["v"]]
errors = [car["desired_v"] - car["v"]]

for step in range(STEPS):
    desired_acceleration, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    throttle = acceleration_to_throttle_percentage(desired_acceleration)

    # Extension 2: Increase road resistance for part of the simulation.
    # Observe how quickly PID restores the target speed when it changes.
    friction = 3.0 if 20 <= car["t"] < 35 else 2.0
    update(car, throttle, friction=friction)

    times.append(car["t"])
    velocities.append(car["v"])
    errors.append(car["desired_v"] - car["v"])

fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
axes[0].plot(times, velocities, label="Actual velocity")
axes[0].axhline(car["desired_v"], linestyle="--", label="Target velocity")
axes[0].set_ylabel("Velocity (m/s)")
axes[0].set_title("PID car speed control")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(times, errors, label="Velocity error")
axes[1].axhline(0, linestyle="--", label="Zero error")
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Error (m/s)")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig("pid_results.png", dpi=150)
print(f"Target velocity: {car['desired_v']:.2f} m/s")
print(f"Final velocity:  {car['v']:.2f} m/s")
print(f"Final error:     {car['desired_v'] - car['v']:.4f} m/s")
print("Saved graph to pid_results.png")
plt.show()
