"""
Individual Figure Generation Functions - Complete Implementation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


# ============================================================================
# CONCEPTUAL FIGURES - COMPLETE
# ============================================================================

def create_control_architecture_hierarchy(output_dir):
    """Figure 2.1: Quadrotor Control Architecture Hierarchy - Complete"""
    fig, ax = plt.subplots(figsize=(14, 12))
    ax.set_xlim(0, 14)
    ax.set_ylim(-1, 12)
    ax.axis('off')
    
    ax.text(7, 11.5, 'Quadrotor Control Architecture Hierarchy', fontsize=18, fontweight='bold', ha='center')
    
    # Level 1: Trajectory Planner
    ax.text(7, 10, 'Trajectory Planner\n━━━━━━━━━━━━━━━\nOutputs: $x_d, y_d, z_d, ψ_d$', 
            fontsize=10, ha='center', va='center', 
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#E3F2FD', edgecolor='#1565C0', linewidth=2.5))
    
    ax.annotate('', xy=(7, 8.5), xytext=(7, 9.2), arrowprops=dict(arrowstyle='->', color='black', lw=2))
    
    # Level 2: Position Controller
    ax.text(7, 7.5, 'Position Controller (Backstepping)\n━━━━━━━━━━━━━━━━━━━━━━\nComputes: $φ_d, θ_d$ from $v_x, v_y$', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#E8F5E9', edgecolor='#2E7D32', linewidth=2.5))
    
    ax.annotate('', xy=(3.5, 5.8), xytext=(5.5, 6.7), arrowprops=dict(arrowstyle='->', color='black', lw=2))
    ax.annotate('', xy=(10.5, 5.8), xytext=(8.5, 6.7), arrowprops=dict(arrowstyle='->', color='black', lw=2))
    
    # Level 3: Inner Loops
    ax.text(3.5, 5, 'AISM Controller\n━━━━━━━━━━━━\nAltitude (Z)\nOutput: $u_1$', 
            fontsize=9, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#FFF3E0', edgecolor='#E65100', linewidth=2.5))
    
    ax.text(10.5, 5, 'FDO-NTSM Controller\n━━━━━━━━━━━━━━━\nAttitude (φ, θ, ψ)\nOutput: $u_2, u_3, u_4$', 
            fontsize=9, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#F3E5F5', edgecolor='#7B1FA2', linewidth=2.5))
    
    # FDO box
    ax.text(13, 5, 'FDO\n━━━\n$\\hat{d}$', fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFEBEE', edgecolor='#C62828', linewidth=2))
    ax.annotate('', xy=(11.8, 5), xytext=(12.3, 5), arrowprops=dict(arrowstyle='->', color='#C62828', lw=1.5))
    
    ax.annotate('', xy=(7, 2.8), xytext=(3.5, 3.8), arrowprops=dict(arrowstyle='->', color='black', lw=2))
    ax.annotate('', xy=(7, 2.8), xytext=(10.5, 3.8), arrowprops=dict(arrowstyle='->', color='black', lw=2))
    
    # Level 4: Control Allocation
    ax.text(7, 2, 'Control Allocation\n━━━━━━━━━━━━━━━━━━━\n$u_1,u_2,u_3,u_4$ → $ω_1,ω_2,ω_3,ω_4$', 
            fontsize=9, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#FFCDD2', edgecolor='#B71C1C', linewidth=2.5))
    
    ax.annotate('', xy=(7, 0.3), xytext=(7, 1.2), arrowprops=dict(arrowstyle='->', color='black', lw=2))
    
    # Level 5: Plant
    ax.text(7, -0.5, 'Quadrotor Dynamics\n━━━━━━━━━━━━━━━━━━━\n6-DOF Nonlinear Model', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#E0E0E0', edgecolor='#424242', linewidth=2.5))
    
    # Feedback
    ax.plot([0.8, 0.8], [-0.5, 10], color='#616161', linestyle='--', linewidth=2)
    ax.plot([0.8, 5], [10, 10], color='#616161', linestyle='--', linewidth=2)
    ax.text(0.3, 5, 'State\nFeedback', fontsize=9, ha='center', va='center', rotation=90, color='#616161')
    
    # Disturbance
    ax.text(12, -0.5, 'Disturbances\n$d_1,d_2,d_3,d_4$', fontsize=9, ha='center', va='center', color='#D32F2F')
    ax.annotate('', xy=(9.5, -0.5), xytext=(10.8, -0.5), arrowprops=dict(arrowstyle='->', color='#D32F2F', lw=2))
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_2_1_control_architecture_hierarchy.png', dpi=300, bbox_inches='tight')
    plt.close()


def create_hftc_block_diagram(output_dir):
    """Figure 4.1: HFTC Block Diagram - Complete"""
    fig, ax = plt.subplots(figsize=(18, 10))
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    ax.text(9, 9.5, 'Hybrid Finite-Time Control (HFTC) Block Diagram', fontsize=16, fontweight='bold', ha='center')
    
    # Reference
    ax.text(0.8, 5, 'Reference\n$x_d,y_d,z_d,ψ_d$', fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='black', linewidth=1.5))
    
    # Summing junction
    circle1 = plt.Circle((2.3, 5), 0.25, fill=False, color='black', linewidth=2)
    ax.add_patch(circle1)
    ax.annotate('', xy=(2.05, 5), xytext=(1.5, 5), arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
    
    # Backstepping
    ax.text(4.5, 6.5, 'Backstepping\nPosition Controller\n━━━━━━━━━━━━━\nEq. (26-41)', 
            fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#E8F5E9', edgecolor='#2E7D32', linewidth=2))
    ax.annotate('', xy=(3.2, 6.5), xytext=(2.55, 5.2), arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
    
    # AISM
    ax.text(4.5, 3.5, 'AISM\nAltitude Controller\n━━━━━━━━━━━━━\nEq. (21-25)', 
            fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFF3E0', edgecolor='#E65100', linewidth=2))
    ax.annotate('', xy=(3.2, 3.5), xytext=(2.55, 4.8), arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
    
    # Attitude summing junction
    circle2 = plt.Circle((8.3, 5), 0.25, fill=False, color='black', linewidth=2)
    ax.add_patch(circle2)
    ax.annotate('', xy=(7.8, 5.5), xytext=(6, 6.2), arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
    
    # FDO-NTSM
    ax.text(11, 5, 'FDO-based NTSM\nAttitude Controller\n━━━━━━━━━━━━━━━\nEq. (48-60)', 
            fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#F3E5F5', edgecolor='#7B1FA2', linewidth=2))
    ax.annotate('', xy=(9.3, 5), xytext=(8.55, 5), arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
    
    # FDO
    ax.text(11, 2, 'Finite-Time\nDisturbance Observer\n━━━━━━━━━━━━━━━\nTheorem 1, Eq. (44)', 
            fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFEBEE', edgecolor='#C62828', linewidth=2))
    ax.annotate('', xy=(11, 3.5), xytext=(11, 3), arrowprops=dict(arrowstyle='->', color='#C62828', lw=1.5))
    
    # Control Allocation
    ax.text(14.5, 4, 'Control\nAllocation', fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFCDD2', edgecolor='#B71C1C', linewidth=2))
    ax.annotate('', xy=(13.5, 3), xytext=(5.8, 3), arrowprops=dict(arrowstyle='->', color='#E65100', lw=2))
    ax.annotate('', xy=(14, 5), xytext=(12.8, 5), arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=2))
    
    # Plant
    ax.text(17, 5, 'Quadrotor\nDynamics', fontsize=8, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#E0E0E0', edgecolor='#424242', linewidth=2))
    ax.annotate('', xy=(16, 4.5), xytext=(15.3, 4.2), arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
    
    # Disturbance
    ax.text(17, 7.5, 'Disturbances\n$d(t)=0.5cos(t)$', fontsize=9, ha='center', va='center', color='#D32F2F',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#FFEBEE', edgecolor='#D32F2F', linewidth=1))
    ax.annotate('', xy=(17, 6), xytext=(17, 7), arrowprops=dict(arrowstyle='->', color='#D32F2F', lw=2))
    
    # Feedback
    ax.plot([17, 17, 1.5, 1.5], [3.5, 2, 2, 3.5], color='#616161', linestyle='--', linewidth=1.5)
    ax.plot([1.5, 1.5], [2, 3.5], color='#616161', linestyle='--', linewidth=1.5)
    ax.text(10, 0.5, 'State Feedback', fontsize=9, ha='center', color='#616161')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_4_1_hftc_block_diagram.png', dpi=300, bbox_inches='tight')
    plt.close()


def create_quadrotor_frames_diagram(output_dir):
    """Figure 3.1: Quadrotor Reference Frames - Complete"""
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    # Inertial frame
    origin = [0, 0, 0]
    ax.quiver(*origin, 2.5, 0, 0, color='#D32F2F', arrow_length_ratio=0.08, linewidth=3)
    ax.quiver(*origin, 0, 2.5, 0, color='#388E3C', arrow_length_ratio=0.08, linewidth=3)
    ax.quiver(*origin, 0, 0, 2.5, color='#1976D2', arrow_length_ratio=0.08, linewidth=3)
    ax.text(2.8, 0, 0, '$X_I$', fontsize=12, color='#D32F2F', fontweight='bold')
    ax.text(0, 2.8, 0, '$Y_I$', fontsize=12, color='#388E3C', fontweight='bold')
    ax.text(0, 0, 2.8, '$Z_I$', fontsize=12, color='#1976D2', fontweight='bold')
    
    # Quadrotor position
    quad_pos = np.array([1.8, 1.8, 2.5])
    
    # Body frame
    ax.quiver(*quad_pos, 1.2, 0, 0, color='#C62828', arrow_length_ratio=0.1, linewidth=2.5)
    ax.quiver(*quad_pos, 0, 1.2, 0, color='#2E7D32', arrow_length_ratio=0.1, linewidth=2.5)
    ax.quiver(*quad_pos, 0, 0, 1.2, color='#1565C0', arrow_length_ratio=0.1, linewidth=2.5)
    ax.text(quad_pos[0]+1.4, quad_pos[1], quad_pos[2], '$X_B$', fontsize=11, color='#C62828')
    ax.text(quad_pos[0], quad_pos[1]+1.4, quad_pos[2], '$Y_B$', fontsize=11, color='#2E7D32')
    ax.text(quad_pos[0], quad_pos[1], quad_pos[2]+1.4, '$Z_B$', fontsize=11, color='#1565C0')
    
    # Quadrotor arms
    arm_length = 0.7
    arm_dirs = [[1,1,0], [-1,1,0], [-1,-1,0], [1,-1,0]]
    for arm_dir in arm_dirs:
        arm_dir = np.array(arm_dir) / np.sqrt(2)
        arm_end = quad_pos + arm_dir * arm_length
        ax.plot([quad_pos[0], arm_end[0]], [quad_pos[1], arm_end[1]], [quad_pos[2], arm_end[2]], 'k-', linewidth=4)
        # Rotor
        ax.scatter([arm_end[0]], [arm_end[1]], [arm_end[2]], color='orange', s=100, marker='o')
        # Thrust
        ax.quiver(*arm_end, 0, 0, -0.5, color='cyan', arrow_length_ratio=0.2, linewidth=2)
    
    # Labels
    ax.text(quad_pos[0]+0.8, quad_pos[1]+0.8, quad_pos[2]+0.5, '$ω_1$', fontsize=9)
    ax.text(quad_pos[0]-0.8, quad_pos[1]+0.8, quad_pos[2]+0.5, '$ω_2$', fontsize=9)
    ax.text(quad_pos[0]-0.8, quad_pos[1]-0.8, quad_pos[2]+0.5, '$ω_3$', fontsize=9)
    ax.text(quad_pos[0]+0.8, quad_pos[1]-0.8, quad_pos[2]+0.5, '$ω_4$', fontsize=9)
    
    # Position vector
    ax.plot([0, quad_pos[0]], [0, quad_pos[1]], [0, quad_pos[2]], 'purple', linestyle='--', linewidth=2)
    ax.text(0.9, 0.9, 1.3, '$\\vec{r}$', fontsize=11, color='purple')
    
    # Annotations
    textstr = 'Euler Angles:\n$φ$ : Roll\n$θ$ : Pitch\n$ψ$ : Yaw'
    ax.text2D(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10, verticalalignment='top',
              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    
    textstr2 = 'Control Inputs:\n$u_1$: Thrust\n$u_2$: Roll τ\n$u_3$: Pitch τ\n$u_4$: Yaw τ'
    ax.text2D(0.75, 0.98, textstr2, transform=ax.transAxes, fontsize=10, verticalalignment='top',
              bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9))
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('Quadrotor Reference Frames', fontsize=14, fontweight='bold')
    ax.set_xlim(-0.5, 4)
    ax.set_ylim(-0.5, 4)
    ax.set_zlim(-0.5, 4)
    ax.view_init(elev=20, azim=35)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_3_1_quadrotor_reference_frames.png', dpi=300, bbox_inches='tight')
    plt.close()


def create_backstepping_structure(output_dir):
    """Figure 4.2: Backstepping Structure - Complete"""
    fig, ax = plt.subplots(figsize=(16, 10))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    ax.text(8, 9.5, 'Backstepping Control Structure for Position (X, Y)', fontsize=16, fontweight='bold', ha='center')
    
    # Steps
    ax.text(2, 7, 'STEP 1\n━━━━━━━━━━━━\nPosition Error\n$e_x = x - x_d$\n$e_y = y - y_d$', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#E3F2FD', edgecolor='#1565C0', linewidth=2.5))
    
    ax.annotate('', xy=(4.5, 7), xytext=(3.5, 7), arrowprops=dict(arrowstyle='->', color='black', lw=2.5))
    
    ax.text(6.5, 7, 'STEP 2\n━━━━━━━━━━━━\nVirtual Control\n$α_x = -c_x e_x + ẋ_d$\n$α_y = -c_y e_y + ẏ_d$', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#E8F5E9', edgecolor='#2E7D32', linewidth=2.5))
    
    ax.annotate('', xy=(9, 7), xytext=(8, 7), arrowprops=dict(arrowstyle='->', color='black', lw=2.5))
    
    ax.text(11, 7, 'STEP 3\n━━━━━━━━━━━━\nVelocity Error\n$z_x = ẋ - α_x$\n$z_y = ẏ - α_y$', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#FFF3E0', edgecolor='#E65100', linewidth=2.5))
    
    ax.annotate('', xy=(13.5, 7), xytext=(12.5, 7), arrowprops=dict(arrowstyle='->', color='black', lw=2.5))
    
    ax.text(15, 7, 'STEP 4\n━━━━━━━━━━━━\nControl Law\n$v_x, v_y$', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#FFCDD2', edgecolor='#C62828', linewidth=2.5))
    
    # Output
    ax.text(8, 3.5, 'Desired Attitudes\n━━━━━━━━━━━━━━━━━━━━━━\n$φ_d = arcsin(v_x sinψ - v_y cosψ)$\n$θ_d = arcsin((v_x cosψ + v_y sinψ)/cosφ_d)$', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#F3E5F5', edgecolor='#7B1FA2', linewidth=2.5))
    ax.annotate('', xy=(10, 4.5), xytext=(14, 5.5), arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=2, connectionstyle='arc3,rad=-0.3'))
    
    # Stability
    ax.text(8, 1, 'Lyapunov Stability: $\\dot{V} = -c_x e_x^2 - k_x z_x^2 - c_y e_y^2 - k_y z_y^2 ≤ 0$ ✓', 
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#E8F5E9', edgecolor='#1B5E20', linewidth=2))
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_4_2_backstepping_structure.png', dpi=300, bbox_inches='tight')
    plt.close()


def create_system_architecture(output_dir):
    """Figure 5.1: System Architecture - Complete"""
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 12)
    ax.axis('off')
    
    ax.text(8, 11.5, 'System Architecture: Simulation and Hardware', fontsize=16, fontweight='bold', ha='center')
    
    # Simulation Box
    ax.add_patch(plt.Rectangle((0.5, 6), 7, 5, fill=True, facecolor='#E3F2FD', edgecolor='#1565C0', linewidth=3))
    ax.text(4, 10.7, 'SIMULATION', fontsize=12, fontweight='bold', ha='center', color='#1565C0')
    ax.text(4, 9.5, 'Python/NumPy', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='white', edgecolor='#1565C0'))
    ax.text(2, 8, 'HFTC\nController', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#C8E6C9', edgecolor='#2E7D32'))
    ax.text(6, 8, 'Quadrotor\nDynamics', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#FFECB3', edgecolor='#FF8F00'))
    ax.text(4, 6.5, 'Euler Integration (dt=0.002s)', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#E0E0E0', edgecolor='#424242'))
    
    # Hardware Box
    ax.add_patch(plt.Rectangle((8.5, 6), 7, 5, fill=True, facecolor='#F3E5F5', edgecolor='#7B1FA2', linewidth=3))
    ax.text(12, 10.7, 'HARDWARE', fontsize=12, fontweight='bold', ha='center', color='#7B1FA2')
    ax.text(12, 9.5, 'ROS2 Humble', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='white', edgecolor='#7B1FA2'))
    ax.text(10, 8, 'HFTC\nROS2 Node', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#C8E6C9', edgecolor='#2E7D32'))
    ax.text(14, 8, 'PX4\nAutopilot', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#FFCDD2', edgecolor='#C62828'))
    ax.text(12, 6.5, 'MAVLink/RTPS Bridge', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#E0E0E0', edgecolor='#424242'))
    
    # Common Components Box
    ax.add_patch(plt.Rectangle((2, 1), 12, 4, fill=True, facecolor='#E8F5E9', edgecolor='#1B5E20', linewidth=3))
    ax.text(8, 4.7, 'COMMON CONTROL COMPONENTS', fontsize=11, fontweight='bold', ha='center', color='#1B5E20')
    ax.text(4, 3.5, 'AISM\nAltitude', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#FFF3E0', edgecolor='#E65100', linewidth=2))
    ax.text(8, 3.5, 'Backstepping\nPosition', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#E8F5E9', edgecolor='#2E7D32', linewidth=2))
    ax.text(12, 3.5, 'FDO-NTSM\nAttitude', fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='#F3E5F5', edgecolor='#7B1FA2', linewidth=2))
    ax.text(8, 1.5, 'Finite-Time Disturbance Observer (FDO)', fontsize=10, ha='center', bbox=dict(boxstyle='round', facecolor='#FFEBEE', edgecolor='#C62828', linewidth=2))
    
    # Arrows
    ax.annotate('', xy=(4, 5.8), xytext=(4, 5), arrowprops=dict(arrowstyle='<->', color='#1565C0', lw=2))
    ax.annotate('', xy=(12, 5.8), xytext=(12, 5), arrowprops=dict(arrowstyle='<->', color='#7B1FA2', lw=2))
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_5_1_system_architecture.png', dpi=300, bbox_inches='tight')
    plt.close()


def create_ros2_pipeline(output_dir):
    """Figure 5.2: ROS2 Control Pipeline - Complete"""
    fig, ax = plt.subplots(figsize=(18, 8))
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    ax.text(9, 7.5, 'ROS2 Control Pipeline for PX4 Quadrotor', fontsize=16, fontweight='bold', ha='center')
    
    # Nodes
    nodes = [
        (1.5, 4.5, 'Trajectory\nGenerator', '#E3F2FD', '#1565C0'),
        (4.5, 4.5, 'HFTC\nController', '#E8F5E9', '#2E7D32'),
        (7.5, 4.5, 'Control\nAllocation', '#FFF3E0', '#E65100'),
        (10.5, 4.5, 'PX4 RTPS\nBridge', '#F3E5F5', '#7B1FA2'),
        (13.5, 4.5, 'PX4\nAutopilot', '#FFCDD2', '#C62828'),
        (16.5, 4.5, 'Quadrotor\nHardware', '#E0E0E0', '#424242'),
    ]
    
    for x, y, label, facecolor, edgecolor in nodes:
        ax.text(x, y, label, fontsize=9, ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.4', facecolor=facecolor, edgecolor=edgecolor, linewidth=2))
    
    # Arrows
    for i in range(len(nodes)-1):
        ax.annotate('', xy=(nodes[i+1][0]-1, 4.5), xytext=(nodes[i][0]+1, 4.5),
                    arrowprops=dict(arrowstyle='->', color='black', lw=2))
    
    # Topics
    topics = [('/trajectory', 3), ('/attitude_sp', 6), ('/actuator', 9), ('/fmu/in', 12), ('PWM', 15)]
    for topic, x in topics:
        ax.text(x, 5.5, topic, fontsize=7, ha='center', color='gray', style='italic')
    
    # Rates
    rates = [('50Hz', 3), ('250Hz', 6), ('250Hz', 9), ('250Hz', 12), ('400Hz', 15)]
    for rate, x in rates:
        ax.text(x, 6.2, rate, fontsize=8, ha='center', color='blue', fontweight='bold')
    
    # Feedback
    ax.plot([16.5, 16.5, 1.5, 1.5], [3.5, 2, 2, 3.5], color='#616161', linestyle='--', linewidth=2)
    ax.annotate('', xy=(1.5, 3.5), xytext=(1.5, 2.2), arrowprops=dict(arrowstyle='->', color='#616161', lw=2))
    ax.text(9, 1.5, 'Feedback: /vehicle_odometry, /vehicle_attitude', fontsize=9, ha='center', color='#616161')
    
    # FDO
    ax.text(4.5, 2.5, 'FDO $\\hat{d}$', fontsize=8, ha='center', bbox=dict(boxstyle='round', facecolor='#FFEBEE', edgecolor='#C62828'))
    ax.annotate('', xy=(4.5, 3.5), xytext=(4.5, 3), arrowprops=dict(arrowstyle='->', color='#C62828', lw=1.5))
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_5_2_ros2_control_pipeline.png', dpi=300, bbox_inches='tight')
    plt.close()


# ============================================================================
# SIMULATION RESULT FIGURES
# ============================================================================

def plot_xy_plane_trajectory(states, desired, output_dir):
    """XY Plane Trajectory"""
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(states[:, 0], states[:, 2], 'b-', linewidth=2.5, label='Actual', alpha=0.9)
    ax.plot(desired[:, 0], desired[:, 3], 'r--', linewidth=2.5, label='Desired', alpha=0.9)
    ax.scatter([states[0, 0]], [states[0, 2]], color='green', s=200, marker='o', label='Start', zorder=5)
    ax.scatter([states[-1, 0]], [states[-1, 2]], color='blue', s=200, marker='s', label='End', zorder=5)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_title('XY Trajectory', fontsize=14, fontweight='bold')
    ax.axis('equal')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_6_1_xy_trajectory_tracking.png', dpi=300, bbox_inches='tight')
    plt.close()


def plot_altitude_tracking(t, states, desired, output_dir):
    """Altitude Tracking"""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, states[:, 4], 'b-', linewidth=2.5, label='Actual')
    ax.plot(t, desired[:, 6], 'r--', linewidth=2.5, label='Desired')
    ax.set_ylabel('Altitude z (m)', fontsize=12)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_title('Altitude Tracking', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_6_2_altitude_tracking.png', dpi=300, bbox_inches='tight')
    plt.close()


def plot_position_errors_combined(t, errors, output_dir):
    """Combined Position Errors"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Tracking Errors', fontsize=14, fontweight='bold')
    
    axes[0].plot(t, errors[:, 1], 'b-', linewidth=2)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[0].set_ylabel('X Error (m)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(t, errors[:, 2], 'g-', linewidth=2)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Y Error (m)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    
    axes[2].plot(t, errors[:, 0], 'r-', linewidth=2)
    axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[2].set_ylabel('Z Error (m)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_6_3_xyz_tracking_errors.png', dpi=300, bbox_inches='tight')
    plt.close()


def create_disturbance_response_plot(t, states, desired, errors, output_dir):
    """Disturbance Response"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Disturbance Response', fontsize=14, fontweight='bold')
    
    disturbance = 0.5 * np.cos(t)
    
    axes[0, 0].plot(t, disturbance, 'r-', linewidth=2)
    axes[0, 0].set_ylabel('Disturbance d(t)', fontsize=11)
    axes[0, 0].set_xlabel('Time (s)', fontsize=11)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('Applied Disturbance', fontsize=12, fontweight='bold')
    
    axes[0, 1].plot(t, errors[:, 1], 'b-', linewidth=2, label='X')
    axes[0, 1].plot(t, errors[:, 2], 'g-', linewidth=2, label='Y')
    axes[0, 1].legend()
    axes[0, 1].set_ylabel('Position Error (m)', fontsize=11)
    axes[0, 1].set_xlabel('Time (s)', fontsize=11)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Position Error', fontsize=12, fontweight='bold')
    
    axes[1, 0].plot(t, errors[:, 0], 'r-', linewidth=2)
    axes[1, 0].set_ylabel('Z Error (m)', fontsize=11)
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('Altitude Error', fontsize=12, fontweight='bold')
    
    axes[1, 1].plot(t, errors[:, 3], 'm-', linewidth=2)
    axes[1, 1].set_ylabel('Yaw Error (rad)', fontsize=11)
    axes[1, 1].set_xlabel('Time (s)', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('Yaw Error', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_6_4_disturbance_response.png', dpi=300, bbox_inches='tight')
    plt.close()


def plot_control_inputs(t, controls, output_dir):
    """Control Inputs"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Control Inputs', fontsize=16, fontweight='bold')
    
    axes[0, 0].plot(t, controls[:, 0], 'b-', linewidth=2)
    axes[0, 0].set_ylabel('Thrust $u_1$ (N)', fontsize=11)
    axes[0, 0].set_xlabel('Time (s)', fontsize=11)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('Total Thrust', fontsize=12, fontweight='bold')
    
    axes[0, 1].plot(t, controls[:, 1], 'g-', linewidth=2)
    axes[0, 1].set_ylabel('Roll Torque $u_2$', fontsize=11)
    axes[0, 1].set_xlabel('Time (s)', fontsize=11)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('Roll Torque', fontsize=12, fontweight='bold')
    
    axes[1, 0].plot(t, controls[:, 2], 'r-', linewidth=2)
    axes[1, 0].set_ylabel('Pitch Torque $u_3$', fontsize=11)
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('Pitch Torque', fontsize=12, fontweight='bold')
    
    axes[1, 1].plot(t, controls[:, 3], 'm-', linewidth=2)
    axes[1, 1].set_ylabel('Yaw Torque $u_4$', fontsize=11)
    axes[1, 1].set_xlabel('Time (s)', fontsize=11)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('Yaw Torque', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fig_6_5_control_effort.png', dpi=300, bbox_inches='tight')
    plt.close()


def plot_position_tracking(t, states, desired, output_dir):
    """Position Tracking"""
    fig, axes = plt.subplots(1, 3, figsize=(16, 4))
    fig.suptitle('Position Tracking', fontsize=16, fontweight='bold')
    
    axes[0].plot(t, states[:, 4], 'b-', linewidth=2, label='Actual')
    axes[0].plot(t, desired[:, 6], 'r--', linewidth=2, label='Desired')
    axes[0].set_ylabel('Z (m)', fontsize=11)
    axes[0].set_xlabel('Time (s)', fontsize=11)
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(t, states[:, 0], 'b-', linewidth=2, label='Actual')
    axes[1].plot(t, desired[:, 0], 'r--', linewidth=2, label='Desired')
    axes[1].set_ylabel('X (m)', fontsize=11)
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    axes[2].plot(t, states[:, 2], 'b-', linewidth=2, label='Actual')
    axes[2].plot(t, desired[:, 3], 'r--', linewidth=2, label='Desired')
    axes[2].set_ylabel('Y (m)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/position_tracking.png', dpi=150, bbox_inches='tight')
    plt.close()


def plot_yaw_tracking(t, states, desired, output_dir):
    """Yaw Tracking"""
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, states[:, 10], 'b-', linewidth=2.5, label='Actual')
    ax.plot(t, desired[:, 9], 'r--', linewidth=2.5, label='Desired')
    ax.set_ylabel('Yaw (rad)', fontsize=12)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_title('Yaw Tracking', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/yaw_tracking.png', dpi=150, bbox_inches='tight')
    plt.close()


def plot_yaw_error(t, errors, output_dir):
    """Yaw Error"""
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, errors[:, 3], color='purple', linewidth=2.5)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_ylabel('Yaw Error (rad)', fontsize=12)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.set_title('Yaw Error', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/yaw_error.png', dpi=150, bbox_inches='tight')
    plt.close()


def plot_fdo_performance(t, fdo_est, fdo_true, output_dir):
    """FDO Performance"""
    fig, axes = plt.subplots(1, 3, figsize=(16, 4))
    fig.suptitle('FDO Performance', fontsize=16, fontweight='bold')
    
    labels = ['$d_2$ (Roll)', '$d_3$ (Pitch)', '$d_4$ (Yaw)']
    for i in range(3):
        axes[i].plot(t, fdo_true[:, i], 'r--', linewidth=2.5, label='True')
        axes[i].plot(t, fdo_est[:, i], 'b-', linewidth=1.5, label='Estimate')
        axes[i].set_ylabel(labels[i], fontsize=11)
        axes[i].set_xlabel('Time (s)', fontsize=11)
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fdo_performance.png', dpi=150, bbox_inches='tight')
    plt.close()


def plot_fdo_errors(t, fdo_est, fdo_true, output_dir):
    """FDO Errors"""
    fig, axes = plt.subplots(1, 3, figsize=(16, 4))
    fig.suptitle('FDO Estimation Errors', fontsize=16, fontweight='bold')
    
    errors = np.abs(fdo_est - fdo_true)
    labels = ['$|d_2$ error$|$', '$|d_3$ error$|$', '$|d_4$ error$|$']
    for i in range(3):
        axes[i].semilogy(t, errors[:, i] + 1e-10, linewidth=2)
        axes[i].set_ylabel(labels[i], fontsize=11)
        axes[i].set_xlabel('Time (s)', fontsize=11)
        axes[i].grid(True, alpha=0.3, which='both')
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/fdo_errors.png', dpi=150, bbox_inches='tight')
    plt.close()


def plot_sliding_surfaces(t, sliding, output_dir):
    """Sliding Surfaces"""
    fig, axes = plt.subplots(1, 3, figsize=(16, 4))
    fig.suptitle('NTSM Sliding Surfaces', fontsize=16, fontweight='bold')
    
    labels = ['$s_φ$', '$s_θ$', '$s_ψ$']
    colors = ['blue', 'red', 'green']
    for i in range(3):
        axes[i].plot(t[:len(sliding)], sliding[:, i], color=colors[i], linewidth=2)
        axes[i].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[i].set_ylabel(labels[i], fontsize=11)
        axes[i].set_xlabel('Time (s)', fontsize=11)
        axes[i].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/sliding_surfaces.png', dpi=150, bbox_inches='tight')
    plt.close()


def plot_3d_trajectory_static(states, desired, output_dir):
    """Static 3D Trajectory"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(states[:, 0], states[:, 2], states[:, 4], 'b-', linewidth=2.5, label='Actual', alpha=0.9)
    ax.plot(desired[:, 0], desired[:, 3], desired[:, 6], 'r--', linewidth=2.5, label='Desired', alpha=0.9)
    ax.scatter([states[0, 0]], [states[0, 2]], [states[0, 4]], color='green', s=200, marker='o', label='Start')
    ax.scatter([states[-1, 0]], [states[-1, 2]], [states[-1, 4]], color='blue', s=200, marker='s', label='End')
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.legend(fontsize=10)
    ax.set_title('3D Trajectory', fontsize=16, fontweight='bold')
    ax.view_init(elev=25, azim=45)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/3d_trajectory_static.png', dpi=150, bbox_inches='tight')
    plt.close()


def animate_3d_trajectory(t, states, desired, output_dir):
    """Animated 3D Trajectory"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    skip = 20
    t_anim = t[::skip]
    states_anim = states[::skip]
    desired_anim = desired[::skip]
    n_frames = len(t_anim)
    
    margin = 0.5
    ax.set_xlim([states[:, 0].min() - margin, states[:, 0].max() + margin])
    ax.set_ylim([states[:, 2].min() - margin, states[:, 2].max() + margin])
    ax.set_zlim([states[:, 4].min() - margin, states[:, 4].max() + margin])
    
    line_actual, = ax.plot([], [], [], 'b-', linewidth=2.5, label='Actual')
    line_desired, = ax.plot([], [], [], 'r--', linewidth=2.5, label='Desired')
    point_actual, = ax.plot([], [], [], 'bo', markersize=12)
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=14, fontweight='bold')
    
    ax.scatter([states[0, 0]], [states[0, 2]], [states[0, 4]], color='green', s=150, marker='o', label='Start')
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('3D Trajectory Animation', fontsize=16, fontweight='bold')
    ax.legend(fontsize=10)
    
    def update(frame):
        line_actual.set_data(states_anim[:frame+1, 0], states_anim[:frame+1, 2])
        line_actual.set_3d_properties(states_anim[:frame+1, 4])
        line_desired.set_data(desired_anim[:frame+1, 0], desired_anim[:frame+1, 3])
        line_desired.set_3d_properties(desired_anim[:frame+1, 6])
        point_actual.set_data([states_anim[frame, 0]], [states_anim[frame, 2]])
        point_actual.set_3d_properties([states_anim[frame, 4]])
        time_text.set_text(f'Time: {t_anim[frame]:.2f} s')
        ax.view_init(elev=25, azim=45 + frame * 0.5)
        return line_actual, line_desired, point_actual, time_text
    
    anim = FuncAnimation(fig, update, frames=n_frames, interval=50, blit=False)
    
    try:
        anim.save(f'{output_dir}/trajectory_animation.gif', writer='pillow', fps=20, dpi=100)
        print(f"Animation saved to {output_dir}/trajectory_animation.gif")
    except Exception as e:
        print(f"Could not save animation: {e}")
    
    plt.close()


# ============================================================================
# TABLE GENERATION
# ============================================================================

def generate_all_tables(t, errors, controls, fdo_est, fdo_true, output_dir):
    """Generate performance tables"""
    steady_state_idx = np.argmax(t >= (t[-1] - 5.0))
    ss_errors = errors[steady_state_idx:, :]
    
    tables = f"""
================================================================================
SIMULATION RESULTS SUMMARY
================================================================================

STEADY-STATE PERFORMANCE (Last 5 seconds):
| Metric      | Z (mm)  | X (cm)  | Y (cm)  | Yaw (mrad) |
|-------------|---------|---------|---------|------------|
| Max Error   | {np.max(np.abs(ss_errors[:, 0]))*1000:.2f}   | {np.max(np.abs(ss_errors[:, 1]))*100:.2f}   | {np.max(np.abs(ss_errors[:, 2]))*100:.2f}   | {np.max(np.abs(ss_errors[:, 3]))*1000:.2f}      |
| RMS Error   | {np.sqrt(np.mean(ss_errors[:, 0]**2))*1000:.2f}   | {np.sqrt(np.mean(ss_errors[:, 1]**2))*100:.2f}   | {np.sqrt(np.mean(ss_errors[:, 2]**2))*100:.2f}   | {np.sqrt(np.mean(ss_errors[:, 3]**2))*1000:.2f}      |

CONTROL:
| Avg Thrust: {np.mean(controls[:, 0]):.2f} N | Max Thrust: {np.max(controls[:, 0]):.2f} N |

FDO:
| RMS Error: {np.sqrt(np.mean((fdo_est - fdo_true)**2)):.6f} |
================================================================================
"""
    
    with open(f'{output_dir}/simulation_tables.txt', 'w') as f:
        f.write(tables)
    
    return tables


def print_performance_metrics(t, errors, controls, fdo_est, fdo_true):
    """Print performance metrics"""
    print("\n" + "="*60)
    print("PERFORMANCE METRICS")
    print("="*60)
    
    ss_idx = np.argmax(t >= (t[-1] - 5.0))
    ss_errors = errors[ss_idx:, :]
    
    print(f"Duration: {t[-1]:.1f} s")
    print(f"\nSteady-State RMS Errors:")
    print(f"  Z: {np.sqrt(np.mean(ss_errors[:, 0]**2))*1000:.2f} mm")
    print(f"  X: {np.sqrt(np.mean(ss_errors[:, 1]**2))*100:.2f} cm")
    print(f"  Y: {np.sqrt(np.mean(ss_errors[:, 2]**2))*100:.2f} cm")
    print(f"  Yaw: {np.sqrt(np.mean(ss_errors[:, 3]**2))*1000:.2f} mrad")
    print(f"\nAvg Thrust: {np.mean(controls[:, 0]):.2f} N")
    print("="*60)
