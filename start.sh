#!/bin/bash

# Quick start script for Planner Map
# This script helps you get started with the project

set -e

echo "=========================================="
echo "  Planner Map - Quick Start Script"
echo "=========================================="
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Error: Docker is not installed."
    echo "   Please install Docker from https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker compose &> /dev/null; then
    echo "❌ Error: Docker Compose is not installed."
    echo "   Please install Docker Compose from https://docs.docker.com/compose/install/"
    exit 1
fi

echo "✅ Docker is installed: $(docker --version)"
echo "✅ Docker Compose is installed: $(docker compose --version)"
echo ""

# Check if Docker daemon is running
if ! docker info &> /dev/null; then
    echo "❌ Error: Docker daemon is not running."
    echo "   Please start Docker and try again."
    exit 1
fi

echo "✅ Docker daemon is running"
echo ""

# Show menu
echo "What would you like to do?"
echo ""
echo "1) Build and start all services (first time setup)"
echo "2) Start services (if already built)"
echo "3) Stop all services"
echo "4) View logs"
echo "5) Clean up (remove containers and volumes)"
echo "6) Exit"
echo ""
read -p "Enter your choice [1-6]: " choice

case $choice in
    1)
        echo ""
        echo "🔨 Building and starting services..."
        echo "   This may take several minutes on first run."
        docker compose up --build -d
        echo ""
        echo "✅ Services started!"
        echo ""
        echo "🌐 Web interface: http://localhost:8000"
        echo ""
        echo "📊 To view logs: docker compose logs -f"
        echo "🛑 To stop: docker compose down"
        ;;
    2)
        echo ""
        echo "🚀 Starting services..."
        docker compose up -d
        echo ""
        echo "✅ Services started!"
        echo ""
        echo "🌐 Web interface: http://localhost:8000"
        ;;
    3)
        echo ""
        echo "🛑 Stopping services..."
        docker compose down
        echo "✅ Services stopped!"
        ;;
    4)
        echo ""
        echo "📊 Showing logs (Press Ctrl+C to exit)..."
        docker compose logs -f
        ;;
    5)
        echo ""
        read -p "⚠️  This will remove all containers and volumes. Continue? (y/n): " confirm
        if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
            echo "🧹 Cleaning up..."
            docker compose down -v
            rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
            echo "✅ Cleanup complete!"
        else
            echo "Cancelled."
        fi
        ;;
    6)
        echo "Goodbye! 👋"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice. Please run the script again."
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "  For more options, see README.md"
echo "  or use 'make help'"
echo "=========================================="
