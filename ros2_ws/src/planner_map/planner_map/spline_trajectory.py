#!/usr/bin/env python3
"""
Clothoid Trajectory Generator - Generates smooth continuous trajectories using clothoid curves
"""
import numpy as np
from scipy import integrate, optimize
from scipy.special import fresnel
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
    curvature: float = 0.0


class ClothoidSegment:
    """
    Represents a clothoid (Euler spiral) segment

    Clothoids have linearly varying curvature: κ(s) = κ0 + κ'·s
    where s is arc length, κ0 is initial curvature, κ' is curvature rate
    """

    def __init__(self, x0: float, y0: float, theta0: float,
                 kappa0: float, kappa1: float, length: float):
        """
        Initialize a clothoid segment

        Args:
            x0, y0: Starting position
            theta0: Starting heading angle (radians)
            kappa0: Initial curvature (1/radius)
            kappa1: Final curvature
            length: Arc length of segment
        """
        self.x0 = x0
        self.y0 = y0
        self.theta0 = theta0
        self.kappa0 = kappa0
        self.kappa1 = kappa1
        self.length = length
        self.kappa_rate = (kappa1 - kappa0) / length if length > 0 else 0.0

    def evaluate(self, s: float) -> Tuple[float, float, float, float]:
        """
        Evaluate position, heading, and curvature at arc length s

        Args:
            s: Arc length along the clothoid (0 to self.length)

        Returns:
            (x, y, theta, kappa) at arc length s
        """
        if s <= 0:
            return self.x0, self.y0, self.theta0, self.kappa0

        s = min(s, self.length)

        # Curvature at s
        kappa_s = self.kappa0 + self.kappa_rate * s

        # Use Fresnel integrals for clothoid evaluation
        # For a clothoid with kappa(s) = a*s, the position is:
        # x(s) = sqrt(pi/a) * C(s*sqrt(a/pi))
        # y(s) = sqrt(pi/a) * S(s*sqrt(a/pi))
        # where C and S are Fresnel cosine and sine integrals

        # For general clothoid, we need to integrate
        # This is an approximation using numerical integration
        if abs(self.kappa_rate) < 1e-10:
            # Constant curvature (circular arc or straight line)
            if abs(self.kappa0) < 1e-10:
                # Straight line
                x = self.x0 + s * np.cos(self.theta0)
                y = self.y0 + s * np.sin(self.theta0)
                theta = self.theta0
            else:
                # Circular arc
                radius = 1.0 / self.kappa0
                dtheta = s * self.kappa0
                x = self.x0 + radius * (np.sin(self.theta0 + dtheta) - np.sin(self.theta0))
                y = self.y0 - radius * (np.cos(self.theta0 + dtheta) - np.cos(self.theta0))
                theta = self.theta0 + dtheta
        else:
            # General clothoid - use numerical integration
            n_steps = max(10, int(s / 0.1) + 1)
            s_vals = np.linspace(0, s, n_steps)

            # Integrate to get theta
            kappa_vals = self.kappa0 + self.kappa_rate * s_vals
            try:
                # Try newer scipy API first
                from scipy.integrate import cumulative_trapezoid
                theta_vals = self.theta0 + cumulative_trapezoid(kappa_vals, s_vals, initial=0)
            except ImportError:
                # Fall back to older scipy API
                theta_vals = self.theta0 + integrate.cumtrapz(kappa_vals, s_vals, initial=0)

            # Integrate to get x, y
            dx_vals = np.cos(theta_vals)
            dy_vals = np.sin(theta_vals)

            try:
                # Try newer scipy API
                from scipy.integrate import trapezoid
                x = self.x0 + trapezoid(dx_vals, s_vals)
                y = self.y0 + trapezoid(dy_vals, s_vals)
            except ImportError:
                # Fall back to older scipy API
                x = self.x0 + integrate.trapz(dx_vals, s_vals)
                y = self.y0 + integrate.trapz(dy_vals, s_vals)
            theta = theta_vals[-1]

        return x, y, theta, kappa_s


class ClothoidTrajectoryGenerator:
    """
    Generates smooth continuous trajectories using clothoid curves.

    Clothoids (Euler spirals) are curves with linearly varying curvature,
    making them ideal for vehicle path planning as they provide:
    - Smooth transitions between straight and curved sections
    - Natural steering angle progression
    - Comfortable and predictable vehicle motion
    - Better handling of curvature constraints
    """

    def __init__(self, dt: float = 0.1, max_velocity: float = 5.0, max_curvature: float = 0.5):
        """
        Initialize the clothoid trajectory generator

        Args:
            dt: Sampling time interval in seconds (default: 0.1s = 10Hz)
            max_velocity: Maximum allowed velocity in m/s (default: 5.0 m/s)
            max_curvature: Maximum allowed curvature in 1/m (default: 0.5 = 2m radius)
        """
        self.dt = dt
        self.max_velocity = max_velocity
        self.max_curvature = max_curvature

    def generate_trajectory(self, waypoints: List[Tuple[float, float, float]]) -> List[TrajectoryPoint]:
        """
        Generate a smooth clothoid trajectory from waypoints

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

        # For 3+ waypoints, generate clothoid segments
        clothoid_segments = self._generate_clothoid_path(x_points, y_points, z_points)

        if not clothoid_segments:
            # Fallback to linear interpolation if clothoid generation fails
            return self._linear_trajectory(waypoints)

        # Sample the clothoid path with velocity constraints
        trajectory = self._sample_clothoid_trajectory(clothoid_segments)

        return trajectory

    def _generate_clothoid_path(self, x_points: np.ndarray, y_points: np.ndarray,
                                z_points: np.ndarray) -> List[Tuple[ClothoidSegment, float, float]]:
        """
        Generate clothoid segments connecting waypoints

        Returns:
            List of (clothoid_segment, z_start, z_end) tuples
        """
        segments = []

        for i in range(len(x_points) - 1):
            x0, y0 = x_points[i], y_points[i]
            x1, y1 = x_points[i + 1], y_points[i + 1]
            z0, z1 = z_points[i], z_points[i + 1]

            # Calculate heading angle between waypoints
            dx = x1 - x0
            dy = y1 - y0
            length = np.sqrt(dx**2 + dy**2)

            if length < 1e-6:
                continue

            theta0 = np.arctan2(dy, dx)

            # For simplicity, use straight line segments (zero curvature)
            # A more sophisticated approach would fit clothoids considering
            # heading continuity at waypoints
            segment = ClothoidSegment(
                x0=x0, y0=y0,
                theta0=theta0,
                kappa0=0.0,  # Start with zero curvature
                kappa1=0.0,  # End with zero curvature
                length=length
            )

            segments.append((segment, z0, z1))

        # Optionally smooth curvature transitions between segments
        segments = self._smooth_curvature_transitions(segments, x_points, y_points, z_points)

        return segments

    def _smooth_curvature_transitions(self, segments: List[Tuple[ClothoidSegment, float, float]],
                                     x_points: np.ndarray, y_points: np.ndarray,
                                     z_points: np.ndarray) -> List[Tuple[ClothoidSegment, float, float]]:
        """
        Apply curvature smoothing to create more natural transitions

        This uses a simplified approach with gentle curvature changes
        """
        if len(segments) < 2:
            return segments

        smoothed_segments = []

        for i, (seg, z0, z1) in enumerate(segments):
            # Calculate desired curvature based on trajectory direction changes
            if i < len(segments) - 1:
                next_seg = segments[i + 1][0]

                # Angle change between segments
                delta_theta = next_seg.theta0 - seg.theta0

                # Normalize angle to [-pi, pi]
                delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

                # Estimate curvature for smooth transition
                # Use a fraction of the angle change over the segment length
                if abs(delta_theta) > 1e-6 and seg.length > 1e-6:
                    target_kappa = delta_theta / (2 * seg.length)
                    target_kappa = np.clip(target_kappa, -self.max_curvature, self.max_curvature)
                else:
                    target_kappa = 0.0

                # Create clothoid with gentle curvature change
                smoothed_seg = ClothoidSegment(
                    x0=seg.x0, y0=seg.y0,
                    theta0=seg.theta0,
                    kappa0=0.0 if i == 0 else smoothed_segments[-1][0].kappa1,
                    kappa1=target_kappa,
                    length=seg.length
                )
            else:
                # Last segment ends with zero curvature
                smoothed_seg = ClothoidSegment(
                    x0=seg.x0, y0=seg.y0,
                    theta0=seg.theta0,
                    kappa0=smoothed_segments[-1][0].kappa1 if smoothed_segments else 0.0,
                    kappa1=0.0,
                    length=seg.length
                )

            smoothed_segments.append((smoothed_seg, z0, z1))

        return smoothed_segments

    def _sample_clothoid_trajectory(self, segments: List[Tuple[ClothoidSegment, float, float]]) -> List[TrajectoryPoint]:
        """
        Sample the clothoid path with velocity constraints
        """
        # Calculate total path length
        total_length = sum(seg.length for seg, _, _ in segments)

        if total_length < 1e-6:
            return []

        # Calculate total time based on max velocity
        total_time = total_length / self.max_velocity

        # Sample at dt intervals
        num_samples = int(total_time / self.dt) + 1
        time_samples = np.linspace(0, total_time, num_samples)

        trajectory = []

        for t in time_samples:
            # Calculate distance traveled at constant velocity
            distance = t * self.max_velocity
            distance = min(distance, total_length)

            # Find which segment we're on
            cumulative_length = 0.0
            current_seg = None
            z_start, z_end = 0.0, 0.0
            s_in_segment = 0.0

            for seg, z0, z1 in segments:
                if cumulative_length + seg.length >= distance:
                    current_seg = seg
                    z_start, z_end = z0, z1
                    s_in_segment = distance - cumulative_length
                    break
                cumulative_length += seg.length

            if current_seg is None:
                # Use last segment
                current_seg, z_start, z_end = segments[-1]
                s_in_segment = current_seg.length

            # Evaluate clothoid at this arc length
            x, y, theta, kappa = current_seg.evaluate(s_in_segment)

            # Interpolate z coordinate
            if current_seg.length > 1e-6:
                alpha = s_in_segment / current_seg.length
                z = z_start + alpha * (z_end - z_start)
            else:
                z = z_start

            # Calculate velocity (constant for now)
            velocity = self.max_velocity if t < total_time else 0.0

            trajectory.append(TrajectoryPoint(
                x=float(x),
                y=float(y),
                z=float(z),
                velocity=velocity,
                time=t,
                curvature=float(kappa)
            ))

        return trajectory

    def _linear_trajectory(self, waypoints: List[Tuple[float, float, float]]) -> List[TrajectoryPoint]:
        """
        Create a simple linear trajectory between waypoints (fallback)
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
                        time=t,
                        curvature=0.0
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

        # Check curvature constraints
        max_curv = max(abs(point.curvature) for point in trajectory)
        if max_curv > self.max_curvature * 1.01:  # Allow 1% tolerance
            return False, f"Curvature exceeds limit: {max_curv:.3f} > {self.max_curvature:.3f} 1/m"

        return True, "Trajectory valid"


# Alias for backward compatibility
SplineTrajectoryGenerator = ClothoidTrajectoryGenerator
