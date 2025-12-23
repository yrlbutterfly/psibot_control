#!/bin/bash
# RealMan RM75 机械臂环境配置和测试脚本

set -e  # Exit on error

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project directory
PROJECT_DIR="/home/psibot/Documents/psibot_control"

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  RealMan RM75 Setup & Test${NC}"
echo -e "${BLUE}================================${NC}\n"

# Function to print colored messages
print_step() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Check if conda is installed
echo -e "\n${BLUE}Step 1: Checking Conda installation...${NC}"
if ! command -v conda &> /dev/null; then
    print_error "Conda not found! Please install Miniconda or Anaconda first."
    exit 1
fi
print_step "Conda is installed"

# Check if environment exists
echo -e "\n${BLUE}Step 2: Checking Conda environment...${NC}"
if conda env list | grep -q "^psibot "; then
    print_step "Environment 'psibot' already exists"
else
    print_warning "Environment 'psibot' not found. Creating..."
    conda create -n psibot python=3.10 -y
    print_step "Environment created successfully"
fi

# Activate environment
echo -e "\n${BLUE}Step 3: Activating environment...${NC}"
eval "$(conda shell.bash hook)"
conda activate psibot
print_step "Environment activated"

# Install dependencies
echo -e "\n${BLUE}Step 4: Installing dependencies...${NC}"
cd "$PROJECT_DIR"
pip install -q -r requirements.txt
print_step "Dependencies installed"

# Set environment variables
echo -e "\n${BLUE}Step 5: Setting environment variables...${NC}"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PROJECT_DIR/robot_libs/lib
print_step "LD_LIBRARY_PATH configured"

# Check SDK library
echo -e "\n${BLUE}Step 6: Checking SDK library...${NC}"
if [ -f "$PROJECT_DIR/robot_libs/lib/libRM_Base.so" ]; then
    print_step "SDK library found"
else
    print_error "SDK library not found at $PROJECT_DIR/robot_libs/lib/libRM_Base.so"
    exit 1
fi

# Check network connectivity
echo -e "\n${BLUE}Step 7: Checking network connectivity...${NC}"
LEFT_ARM_IP="192.168.100.100"
RIGHT_ARM_IP="192.168.100.101"

if ping -c 1 -W 2 $LEFT_ARM_IP &> /dev/null; then
    print_step "Left arm reachable at $LEFT_ARM_IP"
else
    print_warning "Cannot reach left arm at $LEFT_ARM_IP"
fi

if ping -c 1 -W 2 $RIGHT_ARM_IP &> /dev/null; then
    print_step "Right arm reachable at $RIGHT_ARM_IP"
else
    print_warning "Cannot reach right arm at $RIGHT_ARM_IP"
fi

# Check serial ports (optional)
echo -e "\n${BLUE}Step 8: Checking serial ports...${NC}"
if ls /dev/ttyUSB* &> /dev/null; then
    print_step "Serial ports found:"
    ls -l /dev/ttyUSB*
else
    print_warning "No serial ports found (OK if not using dexterous hands)"
fi

# Summary
echo -e "\n${BLUE}================================${NC}"
echo -e "${GREEN}   Setup Complete!${NC}"
echo -e "${BLUE}================================${NC}\n"

echo -e "${YELLOW}What would you like to do next?${NC}"
echo -e "  1) Run simple arm test (recommended for first time)"
echo -e "  2) Read current joint angles"
echo -e "  3) Run basic movement test"
echo -e "  4) Exit and test manually"
echo -e ""
read -p "Enter your choice (1-4): " choice

case $choice in
    1)
        echo -e "\n${BLUE}Running simple arm test...${NC}\n"
        python test_arm_simple.py
        ;;
    2)
        echo -e "\n${BLUE}Reading joint angles...${NC}\n"
        python get_joint.py
        ;;
    3)
        echo -e "\n${BLUE}Running basic movement test...${NC}\n"
        echo -e "${RED}WARNING: Make sure the workspace is clear!${NC}"
        read -p "Press Enter to continue or Ctrl+C to cancel..."
        python begin.py
        ;;
    4)
        echo -e "\n${GREEN}Environment is ready!${NC}"
        echo -e "\nTo start working, run:"
        echo -e "  ${YELLOW}conda activate psibot${NC}"
        echo -e "  ${YELLOW}export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$PROJECT_DIR/robot_libs/lib${NC}"
        echo -e "  ${YELLOW}cd $PROJECT_DIR${NC}"
        ;;
    *)
        echo -e "\n${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac

echo -e "\n${GREEN}Done!${NC}\n"

