#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化目标位置相对于机械臂基座的位置
Visualize target position relative to robot base
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualize_robot_workspace(target_pos, camera_pos):
    """
    Visualize robot workspace with target and camera positions
    
    Args:
        target_pos: Target position in base frame [x, y, z]
        camera_pos: Camera position in base frame [x, y, z]
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Draw robot base (origin)
    ax.scatter([0], [0], [0], c='black', marker='o', s=200, label='Robot Base (Origin)')
    
    # Draw base platform (circle on XY plane)
    theta = np.linspace(0, 2*np.pi, 100)
    base_radius = 0.15  # 15cm radius base
    base_x = base_radius * np.cos(theta)
    base_y = base_radius * np.sin(theta)
    base_z = np.zeros_like(theta)
    ax.plot(base_x, base_y, base_z, 'k-', linewidth=2, alpha=0.5)
    
    # Draw coordinate axes at base
    axis_length = 0.3
    ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', arrow_length_ratio=0.15, linewidth=2, label='+X (Forward)')
    ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', arrow_length_ratio=0.15, linewidth=2, label='+Y (Left)')
    ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', arrow_length_ratio=0.15, linewidth=2, label='+Z (Up)')
    
    # Draw camera position
    ax.scatter([camera_pos[0]], [camera_pos[1]], [camera_pos[2]], 
              c='purple', marker='^', s=300, label=f'Camera\n({camera_pos[0]:.2f}, {camera_pos[1]:.2f}, {camera_pos[2]:.2f})')
    
    # Draw line from base to camera
    ax.plot([0, camera_pos[0]], [0, camera_pos[1]], [0, camera_pos[2]], 
           'purple', linestyle='--', alpha=0.5, linewidth=1)
    
    # Draw target position
    ax.scatter([target_pos[0]], [target_pos[1]], [target_pos[2]], 
              c='red', marker='*', s=500, label=f'Target\n({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})')
    
    # Draw line from base to target
    ax.plot([0, target_pos[0]], [0, target_pos[1]], [0, target_pos[2]], 
           'red', linestyle='--', alpha=0.5, linewidth=1)
    
    # Draw typical workspace boundary (cylinder)
    # Typical robot arm reach: 0.3m to 1.0m
    workspace_radius = 1.0
    workspace_height = 0.8
    
    # Draw cylinder
    z_workspace = np.linspace(0, workspace_height, 50)
    theta_workspace = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid = np.meshgrid(theta_workspace, z_workspace)
    x_grid = workspace_radius * np.cos(theta_grid)
    y_grid = workspace_radius * np.sin(theta_grid)
    
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.1, color='gray')
    
    # Draw table plane (at Z=0 or slightly below)
    table_size = 1.5
    table_x = [-table_size/2, table_size/2, table_size/2, -table_size/2, -table_size/2]
    table_y = [-table_size/2, -table_size/2, table_size/2, table_size/2, -table_size/2]
    table_z = [0, 0, 0, 0, 0]
    ax.plot(table_x, table_y, table_z, 'brown', linewidth=2, alpha=0.3, label='Table Surface (Z=0)')
    
    # Calculate distances
    target_horizontal_dist = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
    camera_horizontal_dist = np.sqrt(camera_pos[0]**2 + camera_pos[1]**2)
    
    # Set labels and title
    ax.set_xlabel('X (m) - Forward(+)/Backward(-)', fontsize=10)
    ax.set_ylabel('Y (m) - Left(+)/Right(-)', fontsize=10)
    ax.set_zlabel('Z (m) - Up(+)/Down(-)', fontsize=10)
    ax.set_title('Robot Workspace - Base Frame Coordinates\n' + 
                 f'Target: {target_horizontal_dist:.2f}m horizontal, {target_pos[2]:.3f}m height',
                 fontsize=12, fontweight='bold')
    
    # Set equal aspect ratio
    max_range = 1.0
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, max_range])
    
    # Add legend
    ax.legend(loc='upper right', fontsize=8)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    # Add text annotations
    info_text = f"""
    Camera Position: ({camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}) m
    - Horizontal distance: {camera_horizontal_dist:.3f} m
    - Height: {camera_pos[2]:.3f} m
    
    Target Position: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) m
    - Horizontal distance: {target_horizontal_dist:.3f} m
    - Height: {target_pos[2]:.3f} m
    
    Target relative to base:
    - X: {target_pos[0]:.3f}m ({'backward' if target_pos[0] < 0 else 'forward'})
    - Y: {target_pos[1]:.3f}m ({'left' if target_pos[1] > 0 else 'right'})
    - Z: {target_pos[2]:.3f}m ({'above table' if target_pos[2] > 0 else 'below table'})
    """
    
    plt.figtext(0.02, 0.5, info_text, fontsize=9, family='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig('robot_workspace_visualization.png', dpi=150, bbox_inches='tight')
    print("\n[INFO] Saved visualization to: robot_workspace_visualization.png")
    plt.show()


def main():
    """Main function"""
    
    print("\n" + "="*70)
    print("  Robot Workspace Visualization")
    print("  机械臂工作空间可视化")
    print("="*70)
    
    # Your actual coordinates
    target_pos = np.array([-0.606, 0.499, 0.036])
    camera_pos = np.array([0.406, 0.206, 0.447])
    
    print("\n[Coordinate System]")
    print("  Origin (0,0,0): Robot base center")
    print("  +X: Forward direction")
    print("  +Y: Left direction")
    print("  +Z: Upward direction")
    
    print("\n[Camera Position]")
    print(f"  Position: ({camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}) m")
    print(f"  - {camera_pos[0]:.3f}m {'forward' if camera_pos[0] > 0 else 'backward'}")
    print(f"  - {camera_pos[1]:.3f}m {'left' if camera_pos[1] > 0 else 'right'}")
    print(f"  - {camera_pos[2]:.3f}m above base")
    
    print("\n[Target Position]")
    print(f"  Position: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) m")
    print(f"  - {abs(target_pos[0]):.3f}m {'forward' if target_pos[0] > 0 else 'backward'}")
    print(f"  - {abs(target_pos[1]):.3f}m {'left' if target_pos[1] > 0 else 'right'}")
    print(f"  - {target_pos[2]:.3f}m above base")
    
    # Calculate horizontal distance
    target_horizontal = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
    print(f"\n[Distance Analysis]")
    print(f"  Target horizontal distance from base: {target_horizontal:.3f} m")
    print(f"  Target height: {target_pos[2]:.3f} m")
    
    # Check if in typical workspace
    if 0.2 <= target_horizontal <= 1.0:
        print(f"  ✓ Horizontal distance is within typical robot reach")
    else:
        print(f"  ⚠️  Horizontal distance may be outside typical reach")
    
    if 0.0 <= target_pos[2] <= 0.6:
        print(f"  ✓ Height is reasonable")
    else:
        print(f"  ⚠️  Height may be unusual")
    
    print("\n[Physical Interpretation]")
    print("  If you stand behind the robot and look at it:")
    if target_pos[0] < 0:
        print(f"  - Target is {abs(target_pos[0]):.2f}m BEHIND the robot base")
    else:
        print(f"  - Target is {target_pos[0]:.2f}m IN FRONT of the robot base")
    
    if target_pos[1] > 0:
        print(f"  - Target is {target_pos[1]:.2f}m to the LEFT")
    else:
        print(f"  - Target is {abs(target_pos[1]):.2f}m to the RIGHT")
    
    print(f"  - Target is {target_pos[2]*100:.1f}cm above the base mounting point")
    
    print("\n[Generating 3D visualization...]")
    visualize_robot_workspace(target_pos, camera_pos)
    
    print("\n" + "="*70)
    print("  Visualization complete!")
    print("="*70)


if __name__ == "__main__":
    main()

