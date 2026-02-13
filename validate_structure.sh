#!/bin/bash

# Validation script to check if all required files are present

echo "=========================================="
echo "  Planner Map - Structure Validation"
echo "=========================================="
echo ""

ERRORS=0

# Function to check if file exists
check_file() {
    if [ -f "$1" ]; then
        echo "✓ $1"
    else
        echo "✗ $1 - MISSING"
        ((ERRORS++))
    fi
}

# Function to check if directory exists
check_dir() {
    if [ -d "$1" ]; then
        echo "✓ $1/"
    else
        echo "✗ $1/ - MISSING"
        ((ERRORS++))
    fi
}

echo "Checking project structure..."
echo ""

# Root files
check_file "README.md"
check_file "CONTRIBUTING.md"
check_file "ARCHITECTURE.md"
check_file "LICENSE"
check_file "Makefile"
check_file "start.sh"
check_file ".gitignore"
check_file ".dockerignore"

# Docker files
check_file "Dockerfile.ros"
check_file "Dockerfile.web"
check_file "docker-compose.yml"
check_file "docker-compose.dev.yml"

# Directories
check_dir "docker"
check_dir "config"
check_dir "ros2_ws"
check_dir "web_interface"

echo ""
echo "Checking ROS2 structure..."
echo ""

# ROS2 files
check_file "ros2_ws/requirements.txt"
check_file "ros2_ws/src/planner_map/package.xml"
check_file "ros2_ws/src/planner_map/setup.py"
check_file "ros2_ws/src/planner_map/setup.cfg"
check_file "ros2_ws/src/planner_map/planner_map/__init__.py"
check_file "ros2_ws/src/planner_map/planner_map/planner_node.py"
check_file "ros2_ws/src/planner_map/planner_map/map_server.py"
check_file "ros2_ws/src/planner_map/planner_map/ros2_web_bridge.py"
check_file "ros2_ws/src/planner_map/launch/planner_map.launch.py"
check_file "ros2_ws/src/planner_map/config/params.yaml"

echo ""
echo "Checking web interface structure..."
echo ""

# Web interface files
check_file "web_interface/main.py"
check_file "web_interface/requirements.txt"
check_file "web_interface/templates/index.html"
check_file "web_interface/static/style.css"
check_file "web_interface/static/app.js"

echo ""
echo "Checking Docker configuration..."
echo ""

# Docker files
check_file "docker/ros_entrypoint.sh"
check_file "config/example.env"

echo ""
echo "=========================================="

if [ $ERRORS -eq 0 ]; then
    echo "✅ All files present! Structure is valid."
    echo "=========================================="
    exit 0
else
    echo "❌ Found $ERRORS missing files/directories!"
    echo "=========================================="
    exit 1
fi
