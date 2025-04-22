import numpy as np
from controller import RH56Hand

def simulate_index_finger(l1, l2, alpha1):
    """
    Simulates the index finger and returns results, including x1, y1.
    """
    theta_values = np.linspace(0, 180, 1000)
    phi1_values = []
    d1_values = []
    alpha1_calculated_values = []
    x_joint_values = []
    y_joint_values = []
    x_end_values = []
    y_end_values = []

    for theta in theta_values:
        theta_rad = np.radians(theta)

        x_joint = l1 * np.sin(theta_rad)
        y_joint = l1 * np.cos(theta_rad)
        x_end = x_joint + l2 * np.sin(2 * theta_rad)
        y_end = y_joint + l2 * np.cos(2 * theta_rad)

        d1 = np.sqrt(x_end**2 + y_end**2)
        phi1 = 180 - theta

        # Calculate the angle of d1 with respect to the positive x-axis
        angle_d1_rad = np.arctan2(x_end, y_end)

        # Calculate alpha1 as the angle with the positive y-axis (vertical)
        alpha1_calculated_rad = np.pi - angle_d1_rad
        alpha1_calculated = np.degrees(alpha1_calculated_rad)

        phi1_values.append(phi1)
        d1_values.append(d1)
        alpha1_calculated_values.append(alpha1_calculated)
        x_joint_values.append(x_joint)
        y_joint_values.append(y_joint)
        x_end_values.append(x_end)
        y_end_values.append(y_end)

    return {
        'theta': theta_values,
        'phi1': phi1_values,
        'd1': d1_values,
        'alpha1_calculated': np.array(alpha1_calculated_values),
        'x_joint': np.array(x_joint_values),
        'y_joint': np.array(y_joint_values),
        'x_end': np.array(x_end_values),
        'y_end': np.array(y_end_values)
    }

class TorqueCalculator:
    def __init__(self, hand: RH56Hand, l1: float, l2: float):
        """
        Initialize the torque calculator
        
        Args:
            hand: RH56Hand controller instance
            l1: Length of the first phalanx
            l2: Length of the second phalanx
        """
        self.hand = hand
        self.l1 = l1
        self.l2 = l2
        
    def calculate_torque(self, finger_index: int) -> float:
        """
        Calculate the torque for a specific finger
        
        Args:
            finger_index: Index of the finger (0-5)
            
        Returns:
            float: Calculated torque in N⋅m (assuming force is in N and distance in m)
        """
        # Get force reading for the specific finger
        forces = self.hand.force_act()
        if forces is None or finger_index >= len(forces):
            raise ValueError("Failed to read force values or invalid finger index")
        
        force = forces[finger_index]
        
        # Get current angle of the finger
        angles = self.hand.angle_read()
        if angles is None:
            raise ValueError("Failed to read angle values")
        
        # Convert the angle reading (0-1000) to degrees (0-180)
        theta_deg = (angles[finger_index] / 1000.0) * 180.0
        
        # Simulate finger to get geometric parameters
        simulation_results = simulate_index_finger(self.l1, self.l2, None)
        
        # Find the closest theta value in our simulation
        theta_idx = np.abs(simulation_results['theta'] - theta_deg).argmin()
        
        # Get corresponding d1 and alpha1 values
        d1 = simulation_results['d1'][theta_idx]
        alpha1 = simulation_results['alpha1_calculated'][theta_idx]
        
        # Convert alpha1 to radians for sin calculation
        alpha1_rad = np.radians(alpha1)
        
        # Calculate torque: τ = F × d × sin(α)
        # Note: You might need to add calibration factors for force sensor readings
        force_scaling = 1.0  # Adjust this based on force sensor calibration
        torque = (force * force_scaling) * d1 * np.sin(alpha1_rad)
        
        return torque

    def get_all_torques(self) -> list:
        """
        Calculate torques for all fingers
        
        Returns:
            list: Torques for all fingers in N⋅m
        """
        return [self.calculate_torque(i) for i in range(6)]

    def calculate_index_finger_torque(self) -> float:
        """
        专门为食指（第4根手指，索引为3）计算扭矩
        
        Returns:
            float: 食指的扭矩值，单位为N⋅m
        """
        # 食指的索引是3
        index_finger_idx = 3
        
        # 获取食指的力传感器读数
        forces = self.hand.force_act()
        if forces is None or index_finger_idx >= len(forces):
            raise ValueError("无法读取力传感器值或食指索引无效")
        
        # 获取食指力传感器读数
        force = forces[index_finger_idx]
        
        # 获取食指当前角度
        angles = self.hand.angle_read()
        if angles is None:
            raise ValueError("无法读取角度值")
        
        # 将角度读数(0-1000)转换为角度(0-180)
        theta_deg = (angles[index_finger_idx] / 1000.0) * 180.0
        
        # 使用模拟函数获取几何参数
        simulation_results = simulate_index_finger(self.l1, self.l2, None)
        
        # 找到最接近当前角度的模拟值
        theta_idx = np.abs(simulation_results['theta'] - theta_deg).argmin()
        
        # 获取对应的d1(距离)和alpha1(角度)值
        d1 = simulation_results['d1'][theta_idx]
        alpha1 = simulation_results['alpha1_calculated'][theta_idx]
        
        # 将角度转为弧度以便用于sin计算
        alpha1_rad = np.radians(alpha1)
        
        # 力的缩放系数，根据传感器校准调整
        force_scaling = 1.0
        
        # 计算扭矩: τ = F × d × sin(α)
        torque = (force * force_scaling) * d1 * np.sin(alpha1_rad)
        
        return torque
        
    def calculate_torque_with_angle(self, finger_index: int, angle: float) -> float:
        """
        根据指定角度计算特定手指的扭矩
        
        Args:
            finger_index: 手指索引(0-5)
            angle: 手指角度(0-180度)
            
        Returns:
            float: 计算出的扭矩值，单位为N⋅m
        """
        # 获取指定手指的力传感器读数
        forces = self.hand.force_act()
        if forces is None or finger_index >= len(forces):
            raise ValueError("无法读取力传感器值或手指索引无效")
        
        force = forces[finger_index]
        
        # 使用指定角度而不是读取当前角度
        theta_deg = angle
        
        # 使用模拟函数获取几何参数
        simulation_results = simulate_index_finger(self.l1, self.l2, None)
        
        # 找到最接近指定角度的模拟值
        theta_idx = np.abs(simulation_results['theta'] - theta_deg).argmin()
        
        # 获取对应的d1和alpha1值
        d1 = simulation_results['d1'][theta_idx]
        alpha1 = simulation_results['alpha1_calculated'][theta_idx]
        
        # 将角度转为弧度
        alpha1_rad = np.radians(alpha1)
        
        # 力的缩放系数
        force_scaling = 1.0
        
        # 计算扭矩
        torque = (force * force_scaling) * d1 * np.sin(alpha1_rad)
        
        return torque