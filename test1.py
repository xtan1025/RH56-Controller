import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle
import scipy.optimize as optimize

# ================== 基础手指类 ==================
class Finger:
    def __init__(self, l1, l2, base_offset, is_thumb=False):
        """
        参数：
            l1, l2 - 连杆长度
            base_offset - 基准点偏移 (x, y) 世界坐标系
            is_thumb - 是否为大拇指（反向运动）
        """
        self.l1 = l1
        self.l2 = l2
        self.base_offset = np.array(base_offset)
        self.is_thumb = is_thumb
        
        # 计算可达工作空间
        self.min_reach = abs(self.l1 - self.l2)  # 最小可达距离
        self.max_reach = self.l1 + self.l2       # 最大可达距离

    def forward_kinematics(self, theta):
        """返回世界坐标系中的位置"""
        theta_rad = np.radians(theta)
        
        # 局部坐标系计算
        if self.is_thumb:
            # 大拇指：始终保持相对于负y轴的顺时针运动
            x_joint = self.l1 * np.sin(theta_rad)
            y_joint_local = -self.l1 * np.cos(theta_rad)
            # 确保第二段连杆的运动方向与第一段一致
            x_end = x_joint + self.l2 * np.sin(2*theta_rad)  # 使用相同的theta_rad
            y_end_local = y_joint_local - self.l2 * np.cos(2*theta_rad)
        else:
            # 食指：相对于正y轴的顺时针运动
            x_joint = self.l1 * np.sin(theta_rad)
            y_joint_local = self.l1 * np.cos(theta_rad)
            x_end = x_joint + self.l2 * np.sin(2*theta_rad)
            y_end_local = y_joint_local + self.l2 * np.cos(2*theta_rad)

        # 转换为世界坐标系
        joint_world = self.base_offset + np.array([x_joint, y_joint_local])
        end_world = self.base_offset + np.array([x_end, y_end_local])
        
        return joint_world, end_world

    def calculate_alpha(self, end_world):
        """计算世界坐标系中的alpha角度"""
        # 转换为局部坐标系
        end_local = end_world - self.base_offset
        x, y = end_local
        
        if self.is_thumb:
            # 大拇指：相对于负y轴顺时针
            angle_rad = np.arctan2(x, -y)
            alpha_rad = np.pi - angle_rad
        else:
            # 食指：相对于正y轴顺时针 
            angle_rad = np.arctan2(x, y)
            alpha_rad = np.pi - angle_rad
            
        return np.degrees(alpha_rad) % 360

    def is_reachable(self, target_world):
        """检查目标点是否在可达工作空间内"""
        # 转换到局部坐标系
        target_local = target_world - self.base_offset
        distance = np.linalg.norm(target_local)
        return self.min_reach <= distance <= self.max_reach

    def inverse_kinematics(self, target_world, tolerance=0.01):
        """世界坐标系目标点求解"""
        if not self.is_reachable(target_world):
            raise ValueError(f"目标点 {target_world} 超出工作空间范围 "
                           f"(最小: {self.min_reach:.2f}, 最大: {self.max_reach:.2f})")
        
        # 转换到局部坐标系
        target_local = target_world - self.base_offset
        
        # 遍历所有可能的theta值，将0-180度分成1000份
        best_theta = 0
        min_error = float('inf')
        
        for theta in np.linspace(0, 180, 1000):
            _, end_world = self.forward_kinematics(theta)
            error = np.linalg.norm(end_world - target_world)
            
            if error < min_error:
                min_error = error
                best_theta = theta
                
        return best_theta

# ================== 协同控制系统 ==================
class HandSystem:
    def __init__(self):
        # 定义手指基准点（世界坐标系）
        self.index = Finger(
            l1=3.2, 
            l2=4.6,
            base_offset=(0, 0)  # 食指基准点
        )
        
        self.thumb = Finger(
            l1=5.5,
            l2=5.0,
            base_offset=(-2.9, -7.7),  # 大拇指基准点
            is_thumb=True
        )

    def reach_common_target(self, target_world, tolerance=0.01):
        """控制双手指到达同一目标点"""
        # 检查两个手指是否都能到达目标点
        if not self.index.is_reachable(target_world):
            raise ValueError("目标点超出食指可达范围")
        if not self.thumb.is_reachable(target_world):
            raise ValueError("目标点超出大拇指可达范围")
            
        try:
            # 食指求解
            theta_index = self.index.inverse_kinematics(target_world, tolerance)
            joint_index, end_index = self.index.forward_kinematics(theta_index)
            alpha_index = self.index.calculate_alpha(end_index)
            
            # 大拇指求解
            theta_thumb = self.thumb.inverse_kinematics(target_world, tolerance)
            joint_thumb, end_thumb = self.thumb.forward_kinematics(theta_thumb)
            alpha_thumb = self.thumb.calculate_alpha(end_thumb)
            
            # 检查实际位置与目标位置的误差
            index_error = np.linalg.norm(end_index - target_world)
            thumb_error = np.linalg.norm(end_thumb - target_world)
            
            if max(index_error, thumb_error) > tolerance:
                print(f"警告：未能完全达到目标精度")
                print(f"食指误差: {index_error:.3f}")
                print(f"拇指误差: {thumb_error:.3f}")
            
            return {
                'target': target_world,
                'index': {
                    'theta': theta_index,
                    'alpha': alpha_index,
                    'position': end_index,
                    'joint': joint_index,
                    'error': index_error
                },
                'thumb': {
                    'theta': theta_thumb,
                    'alpha': alpha_thumb,
                    'position': end_thumb,
                    'joint': joint_thumb,
                    'error': thumb_error
                }
            }
            
        except ValueError as e:
            print(f"运动规划失败：{str(e)}")
            return None

    def plot_hand(self, result):
        """可视化双手协调运动"""
        fig, ax = plt.subplots(figsize=(12, 12))
        
        # 绘制基准点
        ax.plot(*self.index.base_offset, 'gs', markersize=12, label='Index Base')
        ax.plot(*self.thumb.base_offset, 'rs', markersize=12, label='Thumb Base')
        
        # 绘制食指
        index_data = result['index']
        ax.plot([self.index.base_offset[0], index_data['joint'][0]],
                [self.index.base_offset[1], index_data['joint'][1]], 
                'b-o', lw=3, label='Index Finger')
        ax.plot([index_data['joint'][0], index_data['position'][0]],
                [index_data['joint'][1], index_data['position'][1]], 
                'b-o', lw=3)
        
        # 绘制大拇指
        thumb_data = result['thumb']
        ax.plot([self.thumb.base_offset[0], thumb_data['joint'][0]],
                [self.thumb.base_offset[1], thumb_data['joint'][1]], 
                'r-o', lw=3, label='Thumb')
        ax.plot([thumb_data['joint'][0], thumb_data['position'][0]],
                [thumb_data['joint'][1], thumb_data['position'][1]], 
                'r-o', lw=3)
        
        # 标注角度
        def draw_angle(ax, center, start, end, radius, color, label):
            arc = Arc(center, 2*radius, 2*radius, theta1=start, theta2=end, 
                     color=color, lw=1.5)
            ax.add_patch(arc)
            mid_angle = np.radians((start + end)/2)
            ax.text(center[0] + radius*1.3*np.cos(np.radians(mid_angle)),
                    center[1] + radius*1.3*np.sin(np.radians(mid_angle)),
                    label, color=color, ha='center', va='center')
        
        # 食指角度
        draw_angle(ax, self.index.base_offset, 90, 90-index_data['theta'], 
                  1.5, 'blue', f'θ_i={index_data["theta"]:.1f}°')
        
        # 大拇指角度
        draw_angle(ax, self.thumb.base_offset, 270, 270+thumb_data['theta'], 
                  1.2, 'red', f'θ_t={thumb_data["theta"]:.1f}°')
        
        # 绘制目标点和实际位置
        target = result['target']
        ax.plot(*target, 'k*', markersize=20, label='Target')
        
        # 绘制误差圆
        error_circle = Circle(target, 0.1, fill=False, color='gray', linestyle='--')
        ax.add_patch(error_circle)
        
        # 设置坐标系
        ax.set_aspect('equal')
        ax.grid(True, linestyle=':')
        ax.set_xlabel("X World Coordinate")
        ax.set_ylabel("Y World Coordinate")
        ax.legend()
        
        # 添加误差信息
        plt.title(f"Hand Coordination System\n" + 
                 f"Target: ({target[0]:.2f}, {target[1]:.2f})\n" +
                 f"Index Error: {index_data['error']:.3f}, " +
                 f"Thumb Error: {thumb_data['error']:.3f}")
        plt.show()

# ================== 使用示例 ==================
if __name__ == "__main__":
    hand = HandSystem()
    
    # 指定世界坐标系目标点
    target_world = np.array([4, -4])
    #target_world = np.array([1, -4])
    
    # 协同运动规划
    result = hand.reach_common_target(target_world, tolerance=7.0)  # 大幅增加容差
    
    if result:
        print("\n=== 运动规划结果 ===")
        print(f"\n目标位置: ({target_world[0]:.2f}, {target_world[1]:.2f})")
        
        print("\n食指结果：")
        print(f"关节角度：{result['index']['theta']:.2f}°")
        print(f"末端角度α：{result['index']['alpha']:.2f}°")
        print(f"实际位置：({result['index']['position'][0]:.2f}, {result['index']['position'][1]:.2f})")
        print(f"位置误差：{result['index']['error']:.3f}")
        
        print("\n大拇指结果：")
        print(f"关节角度：{result['thumb']['theta']:.2f}°")
        print(f"末端角度α：{result['thumb']['alpha']:.2f}°")
        print(f"实际位置：({result['thumb']['position'][0]:.2f}, {result['thumb']['position'][1]:.2f})")
        print(f"位置误差：{result['thumb']['error']:.3f}")
        
        hand.plot_hand(result)
