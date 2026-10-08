"""PID speed controller for Formula Slug's simplified 1D car model."""

import numpy as np


def make_car(desired_v: float = 20.0, dt: float = 0.1) -> dict:
    """Create the car state dictionary supplied by the starter template."""
    return {
        "v": 0.0,
        "a": 0.0,
        "t": 0.0,
        "x": 0.0,
        "dt": dt,
        "desired_v": desired_v,
        "step": 0,
        "error_prev": None,
        "net_integral": 0.0,
    }


def update(
    car: dict,
    throttle_perc: float,
    mass: float = 1000,
    max_throttle_force: float = 5000,
    friction: float = 2.0,
) -> None:
    """Update position, velocity and time using the supplied physics model."""
    force = throttle_perc * max_throttle_force
    car["a"] = (force / mass) - friction
    car["v"] += car["a"] * car["dt"]
    car["x"] += car["v"] * car["dt"]
    car["t"] += car["dt"]
    car["step"] += 1


def calculate_desired_acceleration(
    car: dict, K_P: float, K_I: float = 0.0, K_D: float = 0.0
) -> tuple[float, float]:
    """Return desired acceleration and speed error using PID control."""
    # P: How far are we from the target velocity?
    error = car["desired_v"] - car["v"]
    proportional = K_P * error

    # I: Sum the error over time. Limit it to avoid integral windup.
    car["net_integral"] += error * car["dt"]
    car["net_integral"] = float(np.clip(car["net_integral"], -10.0, 10.0))
    integral = K_I * car["net_integral"]

    # D: How quickly is the error changing?
    if car["error_prev"] is None:
        derivative = 0.0  # No previous error exists on the first step.
    else:
        derivative = (error - car["error_prev"]) / car["dt"]
    car["error_prev"] = error

    desired_acceleration = proportional + integral + K_D * derivative
    return desired_acceleration, error


def acceleration_to_throttle_percentage(
    acceleration_desired: float,
    mass: float = 1000,
    max_throttle_force: float = 5000,
) -> float:
    """Convert acceleration into a throttle value from -1 to 1."""
    max_acceleration = max_throttle_force / mass  # F = m*a
    throttle_percentage = acceleration_desired / max_acceleration
    return float(np.clip(throttle_percentage, -1.0, 1.0))
