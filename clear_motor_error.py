#!/usr/bin/env python3
"""
Clear motor errors for Ruiyan Hand
Sends CLEAR_MOTOR_ERROR command to all motors
"""
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_libs.haptic_hand_control.ry_hand_interface import SerialInterface, RuiyanInstructionType, RuiyanFingerControlMessage
from robot_libs.haptic_hand_control.ry_hand_controller import RuiyanHandController

def clear_all_motor_errors(port='/dev/ttyACM0'):
    """Clear errors on all motors"""
    print(f"Connecting to {port}...")
    
    # Initialize interface
    interface = SerialInterface(
        port=port,
        baudrate=460800,
        auto_connect=True
    )
    
    if not interface.is_connected():
        print(f"❌ Failed to connect to {port}")
        return False
    
    print("✅ Connected successfully!")
    
    motor_ids = [1, 2, 3, 4, 5, 6]
    
    print("\nClearing motor errors...")
    for motor_id in motor_ids:
        print(f"  Clearing Motor {motor_id}...", end=' ')
        
        # Create clear error message
        clear_msg = RuiyanFingerControlMessage(
            motor_id=motor_id,
            instruction=RuiyanInstructionType.CLEAR_MOTOR_ERROR,
            position=None,
            velocity=None,
            current=None
        )
        
        # Send clear command
        if interface.send_message(clear_msg):
            print("✅")
        else:
            print("❌")
        
        time.sleep(0.1)  # Small delay between commands
    
    print("\n✅ All clear commands sent!")
    print("\nWaiting for motors to reset...")
    time.sleep(1.0)
    
    interface.disconnect()
    print("Done!")
    
    return True

def main():
    port = '/dev/ttyACM0'
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("="*60)
    print("Ruiyan Hand - Clear Motor Errors")
    print("="*60)
    print(f"Port: {port}\n")
    
    try:
        clear_all_motor_errors(port)
        
        print("\n" + "="*60)
        print("Next steps:")
        print("="*60)
        print("1. Check if Motor 1 (Thumb Rotation) is still stuck")
        print("2. Try manually rotating it gently")
        print("3. Run the diagnostic script again:")
        print(f"   python diagnose_ruiyan_hand.py {port}")
        print("4. If still stuck, it may be a hardware issue")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

