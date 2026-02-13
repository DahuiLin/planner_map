#!/usr/bin/env python3
"""
Spline Trajectory Generator - Generates smooth continuous trajectories from waypoints
"""
import numpy as np
from scipy import interpolate
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class TrajectoryPoint:
    """Represents a point on the trajectory"""
    x: float
    y: float
    z: float
    velocity: float
    time: float


class SplineTrajectoryGenerator:
    """
    Generates smooth continuous trajectories using cubic B-spline interpolation.

    Features:
    - Cubic B-spline interpolation for smooth continuous paths
    - Velocity constraint enforcement (max velocity)
    - Configurable sampling time (dt)
    - Ensures trajectory stays close to original waypoints
    """

    def __init__(self, dt: float = 0.1, max_velocity: float = 5.0):
        """
        Initialize the spline trajectory generator

        Args:
            dt: Sampling time interval in seconds (default: 0.1s = 10Hz)
            max_velocity: Maximum allowed velocity in m/s (default: 5.0 m/s)
        """
        self.dt = dt
        self.max_velocity = max_velocity

    def generate_trajectory(self, waypoints: List[Tuple[float, float, float]]) -> List[TrajectoryPoint]:
        """
        Generate a smooth spline trajectory from waypoints

        Args:
            waypoints: List of (x, y, z) tuples representing the path waypoints

        Returns:
            List of TrajectoryPoint objects sampled at dt intervals
        """
        if len(waypoints) < 2:
            return []

        # Extract x, y, z coordinates
        waypoints_array = np.array(waypoints)
        x_points = waypoints_array[:, 0]
        y_points = waypoints_array[:, 1]
        z_points = waypoints_array[:, 2]

        # If we have only 2 waypoints, use linear interpolation
        if len(waypoints) == 2:
            return self._linear_trajectory(waypoints)

        # For 3+ waypoints, use cubic B-spline interpolation
        # Calculate cumulative distance along waypoints for parameterization
        distances = np.zeros(len(waypoints))
        for i in range(1, len(waypoints)):
            dx = x_points[i] - x_points[i-1]
            dy = y_points[i] - y_points[i-1]
            dz = z_points[i] - z_points[i-1]
            distances[i] = distances[i-1] + np.sqrt(dx**2 + dy**2 + dz**2)

        total_distance = distances[-1]

        if total_distance < 1e-6:
            # All waypoints are at the same location
            return []

        # Normalize distances to [0, 1] for spline parameter
        u = distances / total_distance

        # Create cubic B-spline for each dimension
        # k=3 for cubic, s=0 for exact interpolation through points
        try:
            # Use splprep for parametric spline
            tck, u_new = interpolate.splprep([x_points, y_points, z_points],
                                             u=u, k=min(3, len(waypoints)-1), s=0)
        except Exception as e:
            # Fallback to linear interpolation if spline fails
            return self._linear_trajectory(waypoints)

        # Calculate the velocity profile considering max_velocity constraint
        # We'll adjust the time parameterization to respect velocity limits
        trajectory = self._sample_trajectory_with_velocity_constraint(tck, total_distance)

        return trajectory

    def _sample_trajectory_with_velocity_constraint(self, tck, total_distance: float) -> List[TrajectoryPoint]:
        """
        Sample the spline trajectory respecting velocity constraints

        Args:
            tck: Spline representation from splprep
            total_distance: Total path length in meters

        Returns:
            List of TrajectoryPoint objects
        """
        # Sample the spline at fine resolution to estimate curvature
        u_fine = np.linspace(0, 1, 1000)
        points_fine = interpolate.splev(u_fine, tck)

        # Calculate velocities along the path
        # Start with constant velocity, then adjust based on max_velocity
        avg_velocity = min(self.max_velocity, total_distance / (self.dt * 100))

        # Calculate distances along the fine-sampled path
        x_fine, y_fine, z_fine = points_fine
        distances_fine = np.zeros(len(u_fine))

        for i in range(1, len(u_fine)):
            dx = x_fine[i] - x_fine[i-1]
            dy = y_fine[i] - y_fine[i-1]
            dz = z_fine[i] - z_fine[i-1]
            distances_fine[i] = distances_fine[i-1] + np.sqrt(dx**2 + dy**2 + dz**2)

        # Estimate total time based on max velocity
        # Use trapezoidal velocity profile: accelerate, cruise, decelerate
        # For simplicity, use constant velocity at max_velocity
        total_time = total_distance / self.max_velocity

        # Sample at dt intervals
        num_samples = int(total_time / self.dt) + 1
        time_samples = np.linspace(0, total_time, num_samples)

        # Convert time to distance (assuming constant velocity)
        distance_samples = time_samples * self.max_velocity

        # Clamp distances to total_distance
        distance_samples = np.clip(distance_samples, 0, total_distance)

        # Find corresponding u values for each distance
        u_samples = np.interp(distance_samples, distances_fine, u_fine)

        # Evaluate spline at these u values
        points_sampled = interpolate.splev(u_samples, tck)
        x_sampled, y_sampled, z_sampled = points_sampled

        # Calculate velocities between consecutive points
        trajectory = []
        for i in range(len(u_samples)):
            if i < len(u_samples) - 1:
                dx = x_sampled[i+1] - x_sampled[i]
                dy = y_sampled[i+1] - y_sampled[i]
                dz = z_sampled[i+1] - z_sampled[i]
                velocity = np.sqrt(dx**2 + dy**2 + dz**2) / self.dt

                # Clamp velocity to max_velocity
                velocity = min(velocity, self.max_velocity)
            else:
                # Last point has zero velocity
                velocity = 0.0

            trajectory.append(TrajectoryPoint(
                x=float(x_sampled[i]),
                y=float(y_sampled[i]),
                z=float(z_sampled[i]),
                velocity=velocity,
                time=time_samples[i]
            ))

        return trajectory

    def _linear_trajectory(self, waypoints: List[Tuple[float, float, float]]) -> List[TrajectoryPoint]:
        """
        Create a simple linear trajectory between waypoints

        Args:
            waypoints: List of (x, y, z) tuples

        Returns:
            List of TrajectoryPoint objects
        """
        if len(waypoints) < 2:
            return []

        # Calculate total distance
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            dz = waypoints[i][2] - waypoints[i-1][2]
            total_distance += np.sqrt(dx**2 + dy**2 + dz**2)

        # Calculate time based on max velocity
        total_time = total_distance / self.max_velocity

        # Sample at dt intervals
        num_samples = int(total_time / self.dt) + 1
        time_samples = np.linspace(0, total_time, num_samples)

        trajectory = []
        for t in time_samples:
            # Find position at time t along the piecewise linear path
            distance_traveled = t * self.max_velocity

            # Find which segment we're on
            cumulative_dist = 0.0
            for i in range(1, len(waypoints)):
                dx = waypoints[i][0] - waypoints[i-1][0]
                dy = waypoints[i][1] - waypoints[i-1][1]
                dz = waypoints[i][2] - waypoints[i-1][2]
                segment_length = np.sqrt(dx**2 + dy**2 + dz**2)

                if cumulative_dist + segment_length >= distance_traveled:
                    # Interpolate within this segment
                    alpha = (distance_traveled - cumulative_dist) / segment_length if segment_length > 0 else 0
                    x = waypoints[i-1][0] + alpha * dx
                    y = waypoints[i-1][1] + alpha * dy
                    z = waypoints[i-1][2] + alpha * dz

                    trajectory.append(TrajectoryPoint(
                        x=x, y=y, z=z,
                        velocity=self.max_velocity if t < total_time else 0.0,
                        time=t
                    ))
                    break

                cumulative_dist += segment_length

        return trajectory

    def validate_trajectory(self, trajectory: List[TrajectoryPoint]) -> Tuple[bool, str]:
        """
        Validate that trajectory meets constraints

        Args:
            trajectory: List of TrajectoryPoint objects

        Returns:
            Tuple of (is_valid, message)
        """
        if not trajectory:
            return False, "Empty trajectory"

        # Check velocity constraints
        max_vel = max(point.velocity for point in trajectory)
        if max_vel > self.max_velocity * 1.01:  # Allow 1% tolerance
            return False, f"Velocity exceeds limit: {max_vel:.2f} > {self.max_velocity:.2f} m/s"

        return True, "Trajectory valid"
