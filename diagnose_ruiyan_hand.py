#!/usr/bin/env python3
"""
Diagnostic script for Ruiyan Hand motors
Tests each motor individually with low current to identify issues
"""
import time
import sys
from robot_libs.ruiyan_hand_module import Hand

def test_single_motor(hand, motor_index, motor_name):
    """Test a single motor with incremental positions"""
    print(f"\n{'='*60}")
    print(f"Testing Motor {motor_index}: {motor_name}")
    print(f"{'='*60}")
    
    # Start from all open
    base_pose = [1.0] * 6
    hand.set_angles(base_pose)
    time.sleep(1.0)
    
    # Test positions: 0%, 25%, 50%, 75%, 100%
    test_positions = [0.0, 0.25, 0.5, 0.75, 1.0]
    
    print(f"{'Position':>10} | {'Target':>10} | {'Actual':>10} | {'Error':>10} | Status")
    print("-" * 60)
    
    for pos in test_positions:
        target = list(base_pose)
        target[motor_index] = pos
        hand.set_angles(target)
        time.sleep(0.8)  # Give time to reach position
        
        actual = hand.get_angles()
        error = abs(actual[motor_index] - pos)
        status = "✅ OK" if error < 0.1 else "⚠️ WARN" if error < 0.2 else "❌ ERROR"
        
        print(f"{pos*100:>9.0f}% | {pos:>10.3f} | {actual[motor_index]:>10.3f} | {error:>10.3f} | {status}")
    
    # Return to neutral
    hand.set_angles([1.0] * 6)
    time.sleep(0.5)

def main():
    port = '/dev/ttyACM0'
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Connecting to Ruiyan Hand on {port}...")
    
    try:
        hand = Hand(port=port)
        print("✅ Connected successfully!\n")
        
        # Define motor names according to Aoyi order
        motor_names = [
            "Thumb (拇指)",
            "Index (食指)",
            "Middle (中指)",
            "Ring (无名指)",
            "Pinky (小指)",
            "Thumb Rotation (拇指旋转)"
        ]
        
        # Read initial state
        print("Initial motor positions:")
        initial_angles = hand.get_angles()
        for i, (name, angle) in enumerate(zip(motor_names, initial_angles)):
            print(f"  Motor {i}: {name:25s} = {angle:.3f}")
        
        # Test each motor
        for i in range(6):
            try:
                test_single_motor(hand, i, motor_names[i])
            except KeyboardInterrupt:
                print("\n\n⚠️ Test interrupted by user")
                break
            except Exception as e:
                print(f"\n❌ Error testing motor {i}: {e}")
                continue
        
        # Final report
        print("\n" + "="*60)
        print("DIAGNOSTIC COMPLETE")
        print("="*60)
        print("\nPlease check the results above to identify problematic motors.")
        print("Motors showing consistent errors may have:")
        print("  - Mechanical obstruction")
        print("  - Damaged components")
        print("  - Incorrect current/velocity settings")
        
        # Return to safe position
        hand.set_angles([1.0] * 6)
        hand.close()
        
    except Exception as e:
        print(f"❌ Failed to connect or test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("="*60)
    print("Ruiyan Hand Motor Diagnostic Tool")
    print("="*60)
    print(f"Usage: python {sys.argv[0]} [port]")
    print(f"Default port: /dev/ttyACM0")
    print()
    main()

