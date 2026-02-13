#!/usr/bin/env python3
"""
Test script for ClothoidTrajectoryGenerator
Tests the clothoid trajectory generation with sample waypoints
"""

import sys
import os

# Add the module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'ros2_ws/src/planner_map'))

from planner_map.spline_trajectory import ClothoidTrajectoryGenerator


def test_simple_trajectory():
    """Test with a simple straight-line trajectory"""
    print("=" * 60)
    print("Test 1: Simple Straight Line Trajectory")
    print("=" * 60)

    # Create generator with dt=0.1s, max_velocity=5.0 m/s, max_curvature=0.5 1/m
    generator = ClothoidTrajectoryGenerator(dt=0.1, max_velocity=5.0, max_curvature=0.5)

    # Simple waypoints: straight line from (0,0,0) to (10,0,0)
    waypoints = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0)
    ]

    trajectory = generator.generate_trajectory(waypoints)

    print(f"Input waypoints: {len(waypoints)}")
    print(f"Generated trajectory points: {len(trajectory)}")

    if trajectory:
        print(f"\nFirst point: x={trajectory[0].x:.2f}, y={trajectory[0].y:.2f}, z={trajectory[0].z:.2f}, "
              f"v={trajectory[0].velocity:.2f} m/s, κ={trajectory[0].curvature:.3f} 1/m, t={trajectory[0].time:.2f}s")
        print(f"Last point:  x={trajectory[-1].x:.2f}, y={trajectory[-1].y:.2f}, z={trajectory[-1].z:.2f}, "
              f"v={trajectory[-1].velocity:.2f} m/s, κ={trajectory[-1].curvature:.3f} 1/m, t={trajectory[-1].time:.2f}s")

        # Check velocity and curvature constraints
        max_vel = max(point.velocity for point in trajectory)
        max_curv = max(abs(point.curvature) for point in trajectory)
        print(f"\nMax velocity in trajectory: {max_vel:.2f} m/s")
        print(f"Max curvature in trajectory: {max_curv:.3f} 1/m")
        print(f"Velocity constraint (max): {generator.max_velocity:.2f} m/s")
        print(f"Curvature constraint (max): {generator.max_curvature:.3f} 1/m")

        is_valid, message = generator.validate_trajectory(trajectory)
        print(f"Trajectory valid: {is_valid} - {message}")

    print()
    return trajectory


def test_curved_trajectory():
    """Test with a curved trajectory"""
    print("=" * 60)
    print("Test 2: Curved Trajectory (L-shape)")
    print("=" * 60)

    # Create generator with dt=0.1s, max_velocity=3.0 m/s, max_curvature=0.5 1/m
    generator = ClothoidTrajectoryGenerator(dt=0.1, max_velocity=3.0, max_curvature=0.5)

    # L-shaped waypoints
    waypoints = [
        (0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 5.0, 0.0),
        (10.0, 10.0, 0.0)
    ]

    trajectory = generator.generate_trajectory(waypoints)

    print(f"Input waypoints: {len(waypoints)}")
    print(f"Generated trajectory points: {len(trajectory)}")

    if trajectory:
        print(f"\nFirst point: x={trajectory[0].x:.2f}, y={trajectory[0].y:.2f}, z={trajectory[0].z:.2f}, "
              f"v={trajectory[0].velocity:.2f} m/s, κ={trajectory[0].curvature:.3f} 1/m, t={trajectory[0].time:.2f}s")
        print(f"Last point:  x={trajectory[-1].x:.2f}, y={trajectory[-1].y:.2f}, z={trajectory[-1].z:.2f}, "
              f"v={trajectory[-1].velocity:.2f} m/s, κ={trajectory[-1].curvature:.3f} 1/m, t={trajectory[-1].time:.2f}s")

        # Sample some intermediate points
        print(f"\nSample intermediate points:")
        for i in [len(trajectory)//4, len(trajectory)//2, 3*len(trajectory)//4]:
            point = trajectory[i]
            print(f"  Point {i}: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}, "
                  f"v={point.velocity:.2f} m/s, κ={point.curvature:.3f} 1/m, t={point.time:.2f}s")

        # Check velocity and curvature constraints
        max_vel = max(point.velocity for point in trajectory)
        max_curv = max(abs(point.curvature) for point in trajectory)
        print(f"\nMax velocity in trajectory: {max_vel:.2f} m/s")
        print(f"Max curvature in trajectory: {max_curv:.3f} 1/m")
        print(f"Velocity constraint (max): {generator.max_velocity:.2f} m/s")
        print(f"Curvature constraint (max): {generator.max_curvature:.3f} 1/m")

        is_valid, message = generator.validate_trajectory(trajectory)
        print(f"Trajectory valid: {is_valid} - {message}")

    print()
    return trajectory


def test_complex_trajectory():
    """Test with a complex S-curve trajectory"""
    print("=" * 60)
    print("Test 3: Complex S-Curve Trajectory")
    print("=" * 60)

    # Create generator with dt=0.05s, max_velocity=8.0 m/s, max_curvature=0.3 1/m
    generator = ClothoidTrajectoryGenerator(dt=0.05, max_velocity=8.0, max_curvature=0.3)

    # S-curve waypoints
    waypoints = [
        (0.0, 0.0, 0.0),
        (5.0, 2.0, 0.0),
        (10.0, 5.0, 0.0),
        (15.0, 7.0, 0.0),
        (20.0, 8.0, 0.0),
        (25.0, 7.0, 0.0),
        (30.0, 5.0, 0.0),
        (35.0, 2.0, 0.0),
        (40.0, 0.0, 0.0)
    ]

    trajectory = generator.generate_trajectory(waypoints)

    print(f"Input waypoints: {len(waypoints)}")
    print(f"Generated trajectory points: {len(trajectory)}")
    print(f"Sampling time (dt): {generator.dt}s")

    if trajectory:
        print(f"\nFirst point: x={trajectory[0].x:.2f}, y={trajectory[0].y:.2f}, z={trajectory[0].z:.2f}, "
              f"v={trajectory[0].velocity:.2f} m/s, κ={trajectory[0].curvature:.3f} 1/m, t={trajectory[0].time:.2f}s")
        print(f"Last point:  x={trajectory[-1].x:.2f}, y={trajectory[-1].y:.2f}, z={trajectory[-1].z:.2f}, "
              f"v={trajectory[-1].velocity:.2f} m/s, κ={trajectory[-1].curvature:.3f} 1/m, t={trajectory[-1].time:.2f}s")

        # Check velocity and curvature constraints
        max_vel = max(point.velocity for point in trajectory)
        max_curv = max(abs(point.curvature) for point in trajectory)
        avg_vel = sum(point.velocity for point in trajectory) / len(trajectory)
        print(f"\nMax velocity in trajectory: {max_vel:.2f} m/s")
        print(f"Average velocity: {avg_vel:.2f} m/s")
        print(f"Max curvature in trajectory: {max_curv:.3f} 1/m")
        print(f"Velocity constraint (max): {generator.max_velocity:.2f} m/s")
        print(f"Curvature constraint (max): {generator.max_curvature:.3f} 1/m")

        is_valid, message = generator.validate_trajectory(trajectory)
        print(f"Trajectory valid: {is_valid} - {message}")

    print()
    return trajectory


def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("CLOTHOID TRAJECTORY GENERATOR TEST SUITE")
    print("=" * 60)
    print()

    try:
        # Run tests
        test_simple_trajectory()
        test_curved_trajectory()
        test_complex_trajectory()

        print("=" * 60)
        print("ALL TESTS COMPLETED SUCCESSFULLY")
        print("=" * 60)
        print("\nClothoid trajectories provide:")
        print("  ✓ Linear curvature change (smooth steering)")
        print("  ✓ Natural vehicle motion")
        print("  ✓ Comfortable transitions")
        print("  ✓ Better for vehicle path planning")
        print()

        return 0

    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
