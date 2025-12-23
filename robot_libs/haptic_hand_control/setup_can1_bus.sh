#!/bin/bash

# Setup script for CAN bus interface
# This script configures the CAN bus for the RyHand robotic hand

echo "Setting up CAN bus interface for RyHand..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run this script as root (use sudo)"
    exit 1
fi

# Load required kernel modules
echo "Loading CAN kernel modules..."
modprobe can
modprobe can-raw
modprobe can-gw
modprobe slcan
modprobe vcan

# Check if can1 interface already exists
if ip link show can1 >/dev/null 2>&1; then
    echo "CAN interface 'can1' already exists, removing it first..."
    ip link set can1 down
    ip link delete can1
fi

# Create new CAN interface
echo "Creating CAN interface 'can1'..."
ip link add can1 type can bitrate 1000000

# Set up the interface
echo "Configuring CAN interface..."
ip link set can1 up type can bitrate 1000000
ip link set can1 txqueuelen 1000

# Verify the setup
echo "Verifying CAN bus setup..."
if ip link show can1 | grep -q "UP"; then
    echo "✓ CAN bus 'can1' is successfully configured and running"
    echo "Interface details:"
    ip -d -s link show can1
else
    echo "✗ Failed to set up CAN bus interface"
    exit 1
fi

echo ""
echo "CAN bus setup complete!"
echo "You can now run your Python script."
echo ""
echo "To check CAN bus status anytime, run:"
echo "  ip link show can1"
echo ""
echo "To monitor CAN traffic, run:"
echo "  candump can1"
echo ""
echo "To send a test message, run:"
echo "  cansend can1 123#DEADBEEF" 