#!/usr/bin/env python3
"""
Integration tests for the Planner Map system.
Tests the interaction between ROS2 nodes and the web interface.
"""

import sys
import time
import requests


def test_web_api_status():
    """Test that the web API is responding."""
    print("Testing Web API status endpoint...")
    try:
        response = requests.get("http://localhost:8000/api/status", timeout=5)
        if response.status_code == 200:
            print("✓ Web API status endpoint is working")
            return True
        else:
            print(f"✗ Web API status returned status code: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Failed to connect to Web API: {e}")
        return False


def test_web_api_map():
    """Test that the map API endpoint is responding."""
    print("\nTesting Web API map endpoint...")
    try:
        response = requests.get("http://localhost:8000/api/map", timeout=5)
        if response.status_code == 200:
            print("✓ Web API map endpoint is working")
            data = response.json()
            print(f"  Map data keys: {list(data.keys())}")
            return True
        else:
            print(f"✗ Web API map returned status code: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Failed to get map from Web API: {e}")
        return False


def test_web_api_goal():
    """Test that we can send a goal to the API."""
    print("\nTesting Web API goal endpoint...")
    try:
        goal_data = {
            "pose": {
                "position": {"x": 10.0, "y": 10.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            }
        }
        response = requests.post(
            "http://localhost:8000/api/goal",
            json=goal_data,
            timeout=5
        )
        if response.status_code == 200:
            print("✓ Web API goal endpoint accepted the goal")
            return True
        else:
            print(f"✗ Web API goal returned status code: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Failed to send goal to Web API: {e}")
        return False


def test_web_api_path():
    """Test that the path API endpoint is responding."""
    print("\nTesting Web API path endpoint...")
    try:
        response = requests.get("http://localhost:8000/api/path", timeout=5)
        if response.status_code == 200:
            print("✓ Web API path endpoint is working")
            return True
        else:
            print(f"✗ Web API path returned status code: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Failed to get path from Web API: {e}")
        return False


def test_web_interface():
    """Test that the web interface HTML page loads."""
    print("\nTesting Web interface HTML page...")
    try:
        response = requests.get("http://localhost:8000/", timeout=5)
        if response.status_code == 200 and "html" in response.text.lower():
            print("✓ Web interface HTML page is loading")
            return True
        else:
            print(f"✗ Web interface returned status code: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Failed to load web interface: {e}")
        return False


def main():
    """Run all integration tests."""
    print("=" * 60)
    print("Planner Map Integration Tests")
    print("=" * 60)

    # Wait for services to be ready
    print("\nWaiting for services to be ready...")
    max_retries = 20
    for i in range(max_retries):
        try:
            response = requests.get("http://localhost:8000/api/status", timeout=2)
            if response.status_code == 200:
                print("Services are ready!")
                break
        except Exception:
            pass

        if i == max_retries - 1:
            print("✗ Services failed to start in time")
            sys.exit(1)

        print(f"  Waiting... ({i+1}/{max_retries})")
        time.sleep(3)

    # Run tests
    tests = [
        test_web_api_status,
        test_web_interface,
        test_web_api_map,
        test_web_api_goal,
        test_web_api_path,
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"✗ Test failed with exception: {e}")
            results.append(False)

    # Print summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("\n✓ All tests passed!")
        sys.exit(0)
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
