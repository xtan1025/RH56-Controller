import struct
import serial
from typing import List, Optional
import time

class RH56Hand:
    """
    RH56 Hand Torque Monitor - Combines force sensor readings and torque calculations
    """
    
    def __init__(self, port: str, hand_id: int = 1, baudrate: int = 115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            stopbits=1,
            parity='N',
            timeout=1
        )
        self.hand_id = hand_id

    def _calc_checksum(self, data: bytes) -> int:
        return sum(data) & 0xFF

    def _send_frame(self, command: int, address: int, data: bytes = b'') -> Optional[bytes]:
        frame = bytes([0xEB, 0x90, self.hand_id])
        data_length = len(data) + 3
        frame += bytes([data_length, command])
        frame += struct.pack('<H', address)
        frame += data
        checksum = self._calc_checksum(frame[2:])
        frame += bytes([checksum])
        self.ser.write(frame)
        response = self.ser.read(64)
        return response if response else None

    def _parse_response(self, response: bytes) -> Optional[List[int]]:
        if response is None:  # Check if response is None first
            return None
        if len(response) < 5:
            return None
        if response[0] != 0x90 or response[1] != 0xEB:
            return None
        if response[2] != self.hand_id:
            return None
        if self._calc_checksum(response[2:-1]) != response[-1]:
            return None
        return response


    def angle_set(self, angles: List[int]):
        if len(angles) != 6:
            raise ValueError("Need 6")
        
        data = b''
        for angle in angles:
            if not (-1 <= angle <= 1000):
                raise ValueError("range error")
            data += struct.pack('<h', angle)
        
        # ANGLE_SET: 0x05CE (1486)
        response = self._send_frame(
            command=0x12,
            address=0x05CE,
            data=data
        )
        return self._parse_response(response)

    def angle_read(self) -> Optional[List[int]]:
        # ANGLE_ACT：0x060A（1546），12byte
        response = self._send_frame(
            command=0x11,
            address=0x060A,
            data=bytes([0x0C])
        )
        if parsed := self._parse_response(response):
            raw_data = parsed[7:-1]
            return [struct.unpack('<h', raw_data[i:i+2])[0] for i in range(0, 12, 2)]
        return None

    def force_act(self) -> Optional[List[int]]:
        """
        Read force sensor values from each finger
        
        Returns:
            Optional[List[int]]: List of force values for 6 fingers, or None if reading fails
            Index mapping:
            0 - Pinky finger force (1582-1583)
            1 - Ring finger force (1584-1585)
            2 - Middle finger force (1586-1587)
            3 - Index finger force (1588-1589)
            4 - Thumb bend force (1590-1591)
            5 - Thumb rotation force (1592-1593)
            
            Force values typically range from 0-1000, higher values indicate greater force
            Unit: g (gram)
        """
        # FORCE_ACT: 0x062E (1582), 12 bytes
        response = self._send_frame(
            command=0x11,
            address=0x062E,
            data=bytes([0x0C])
        )
        if parsed := self._parse_response(response):
            try:
                raw_data = parsed[7:-1]
                if len(raw_data) < 12:
                    print(f"Warning: Force data incomplete, received {len(raw_data)} bytes, need 12 bytes")
                    return None
                forces = [struct.unpack('<h', raw_data[i:i+2])[0] for i in range(0, 12, 2)]
                return forces
            except Exception as e:
                print(f"Error parsing force data: {e}")
                if parsed:
                    print(f"Raw data: {[hex(b) for b in parsed]}")
                return None
        else:
            if response:
                print(f"Unable to parse force response: {[hex(b) for b in response]}")
            else:
                print("No force response received")
            return None

    def set_id(self, new_id: int):
        if not (1 <= new_id <= 254):
            raise ValueError("ID range error")
        # HAND_ID：0x05E8（1000）
        response = self._send_frame(
            command=0x12,
            address=0x05E8,
            data=bytes([new_id])
        )
        if self._parse_response(response):
            self.hand_id = new_id

    def clear_errors(self):
        # CLEAR_ERROR：0x05EC（1004）
        self._send_frame(
            command=0x12,
            address=0x05EC,
            data=bytes([1])
        )

    def save_parameters(self):
        # SAVE：0x05ED（1005）
        self._send_frame(
            command=0x12,
            address=0x05ED,
            data=bytes([1])
        )
        
    def force_set(self, thresholds: List[int]):
        """
        Set force control thresholds for each finger
        
        Args:
            thresholds: List of 6 threshold values (0-1000) for each finger
                Index mapping:
                0 - Pinky finger threshold (1498-1499)
                1 - Ring finger threshold (1500-1501)
                2 - Middle finger threshold (1502-1503)
                3 - Index finger threshold (1504-1505)
                4 - Thumb bend threshold (1506-1507)
                5 - Thumb rotation threshold (1508-1509)
        
        Returns:
            Optional[List[int]]: Response data if successful, None otherwise
        
        Note:
            When setting FORCE_SET(x) to value Y, the finger will stop at this force 
            when moving from open to closed position if FORCE_ACT(x) reaches Y.
            Unit: g (gram)
        """
        if len(thresholds) != 6:
            raise ValueError("Need 6 threshold values")
        
        data = b''
        for threshold in thresholds:
            if not (0 <= threshold <= 1000):
                raise ValueError("Threshold value must be between 0 and 1000")
            data += struct.pack('<h', threshold)
        
        # FORCE_SET: 0x05DA (1498), 12 bytes (6 fingers × 2 bytes)
        # 0x0414 (1044) default force set address
        response = self._send_frame(
            command=0x12,
            address=0x05DA,
            #address=0x0414,
            data=data
        )
        return self._parse_response(response)

    def speed_set(self, speeds: List[int]) -> Optional[List[int]]:
        """
        Set speed values for each degree of freedom.

        Args:
            speeds: List of 6 speed values (0-1000) for each finger.
                The mapping from index to finger is as follows:
                SPEED_SET(0): 小拇指速度设置值 (Pinky finger speed) - Address 1522-1523
                SPEED_SET(1): 无名指速度设置值 (Ring finger speed) - Address 1524-1525
                SPEED_SET(2): 中指速度设置值 (Middle finger speed) - Address 1526-1527
                SPEED_SET(3): 食指速度设置值 (Index finger speed) - Address 1528-1529
                SPEED_SET(4): 大拇指弯曲速度设置值 (Thumb bend speed) - Address 1530-1531
                SPEED_SET(5): 大拇指旋转速度设置值 (Thumb rotation speed) - Address 1532-1533
            
            Note: A speed value of 1000 indicates that it takes approximately 800ms 
                  for the finger to move from its maximum to minimum angle under no load. 
                  The actual speed may decrease if the load is significant.

        Returns:
            Optional[List[int]]: Response data if successful, None otherwise
        """
        if len(speeds) != 6:
            raise ValueError("Need 6 speed values")

        data = b''
        for speed_val in speeds:
            if not (0 <= speed_val <= 1000):
                raise ValueError("Speed value must be between 0 and 1000")
            data += struct.pack('<h', speed_val)  # short is 'h'

        # Base address for SPEED_SET(0) is 1522 (0x05F2).
        # The command writes 12 bytes, covering SPEED_SET(0) through SPEED_SET(5).
        response = self._send_frame(
            command=0x12,  # Write command
            address=0x05F2, # Decimal 1522
            data=data
        )
        return self._parse_response(response)

    def gesture_force_clb(self, gesture_id: int):
        """
        Calibrate force sensors for a specific gesture
        
        Args:
            gesture_id: The ID of the gesture to calibrate (1-255)
            When gesture_id=1, the hand will perform the following calibration sequence:
            1. All five fingers fully open
            2. Four fingers (pinky, ring, middle, index) bend
            3. Four fingers open again
            4. Thumb bends
            5. Thumb opens
            The entire calibration process takes about 6 seconds.
        
        Returns:
            Optional[List[int]]: Response data if successful, None otherwise
        """
        if not (1 <= gesture_id <= 255):
            raise ValueError("Gesture ID must be between 1 and 255")
        
        # GESTURE_FORCE_CLB: 0x03F1 (1009)
        response = self._send_frame(
            command=0x12,
            address=0x03F1,
            data=bytes([gesture_id])
        )
        return self._parse_response(response)

    def smooth_angle_set(self, target_angles: List[int], steps: int = 30, delay: float = 0.05):
        """
        Smoothly move fingers to target angles by dividing the movement into small steps
        
        Args:
            target_angles: List of target angles (6 values)
            steps: Number of steps to divide the movement into (default 30)
            delay: Delay time between steps in seconds (default 0.05)
        """
        if len(target_angles) != 6:
            raise ValueError("Need 6 angle values")
        
        # Read current angles
        current_angles = self.angle_read()
        if current_angles is None:
            raise RuntimeError("Failed to read current angles")
        
        # Calculate angle increment for each step
        angle_steps = []
        for curr, target in zip(current_angles, target_angles):
            diff = target - curr
            step = diff / steps
            angle_steps.append(step)
        
        # Execute step by step
        for i in range(steps):
            next_angles = []
            for j, (curr, step) in enumerate(zip(current_angles, angle_steps)):
                if i == steps - 1:  # Use target angle for the last step
                    next_angles.append(target_angles[j])
                else:
                    next_angle = int(curr + step * (i + 1))
                    next_angles.append(next_angle)
            
            self.angle_set(next_angles)
            time.sleep(delay)

    def force_set_by_newton_target(self, target_newtons: List[float], finger_lengths: dict = None):
        """
        Set force thresholds based on target Newton readings using the formula:
        torque = 0.1211 * newton + 0.005(for right hand index finger)
        
        Args:
            target_newtons: List of 6 target Newton values for each finger
            finger_lengths: Optional dictionary of finger lengths {finger_index: (l1, l2)}
                          If None, uses default lengths from TorqueCalculator
        
        Returns:
            Optional[List[int]]: Response data if successful, None otherwise
        """
        if len(target_newtons) != 6:
            raise ValueError("Need 6 target Newton values")
        
        # Import here to avoid circular import
        from torqueCalculator import TorqueCalculator, simulate_index_finger
        import numpy as np
        
        # Create torque calculator instance
        calculator = TorqueCalculator(self, finger_lengths=finger_lengths)
        
        # Calculate required force thresholds in grams
        force_thresholds = []
        finger_names = ["Pinky", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotation"]
        
        print("=== Newton to Grams Conversion Results ===")
        print("Finger         | Target(N) | Calculated(g) | Final(g)")
        print("-" * 55)
        
        for finger_idx, target_newton in enumerate(target_newtons):
            if target_newton < 0:
                raise ValueError(f"Target Newton value for finger {finger_idx} must be non-negative")
            
            # Apply the given formula: torque = 0.1211 * newton + 0.005
            target_torque = 0.1211 * target_newton + 0.005
            
            # Get current angle for this finger (or use a default angle)
            angles = self.angle_read()
            if angles is None:
                # Use a default angle if we can't read current angles
                alpha_deg = 90  # Default to 90 degrees
            else:
                # Convert angle reading (0-1000) to degrees (19-176)
                alpha_deg = 19 + (angles[finger_idx] / 1000.0) * (176 - 19)
            
            # Get finger-specific lengths
            l1, l2 = calculator.get_finger_lengths(finger_idx)
            
            # Use simulation to get geometric parameters
            simulation_results = simulate_index_finger(l1, l2, alpha_deg)
            d1 = simulation_results['d1'][0]
            
            # Calculate required force using inverse torque formula
            # torque = force_newtons * d1 * sin(alpha)
            # Rearranging: force_newtons = torque / (d1 * sin(alpha))
            alpha_rad = np.radians(90)  # Using 90 degrees as in original code
            sin_alpha = np.sin(alpha_rad)
            
            if d1 == 0 or sin_alpha == 0:
                raise ValueError(f"Invalid geometry for finger {finger_idx}: d1={d1}, sin_alpha={sin_alpha}")
            
            required_force_newtons = target_torque / (d1 * sin_alpha)
            
            # Convert from newtons to grams: N * 1000 / 9.81 = grams
            required_force_grams_raw = (required_force_newtons * 1000) / 9.81
            
            # Ensure the value is within valid range (0-1000)
            required_force_grams = max(0, min(1000, int(required_force_grams_raw)))
            force_thresholds.append(required_force_grams)
            
            # Print conversion details for this finger
            print(f"{finger_names[finger_idx]:14} | {target_newton:8.2f} | {required_force_grams_raw:12.2f} | {required_force_grams:8d}")
        
        print("-" * 55)
        print(f"Total force thresholds set: {force_thresholds}")
        print("=" * 55)
        
        # Set the calculated force thresholds
        return self.force_set(force_thresholds)

    def get_newton_readings_from_forces(self) -> Optional[List[float]]:
        """
        Convert current force readings (grams) to Newton values using the inverse formula:
        newton = (torque - 0.005) / 0.1211
        
        Returns:
            Optional[List[float]]: Newton values for each finger, or None if reading fails
        """
        # Import here to avoid circular import
        from torqueCalculator import TorqueCalculator
        
        # Create torque calculator instance
        calculator = TorqueCalculator(self)
        
        try:
            # Get current torques
            torques = calculator.get_all_torques()
            
            # Convert torques to Newton readings using inverse formula
            newton_readings = []
            for torque in torques:
                # Inverse formula: newton = (torque - 0.005) / 0.1211
                newton = (torque - 0.005) / 0.1211
                newton = max(0, newton)  # Ensure non-negative
                newton_readings.append(newton)
            
            return newton_readings
        except Exception as e:
            print(f"Error calculating Newton readings: {e}")
            return None

    def set_newton_target_with_monitoring(self, target_newtons: List[float], 
                                        finger_lengths: dict = None, 
                                        tolerance: float = 0.1,
                                        max_iterations: int = 10):
        """
        Set Newton targets and monitor if they are achieved, with iterative adjustment
        
        Args:
            target_newtons: List of 6 target Newton values for each finger
            finger_lengths: Optional dictionary of finger lengths
            tolerance: Tolerance for Newton reading accuracy
            max_iterations: Maximum number of adjustment iterations
        
        Returns:
            dict: Results with achieved Newton values and iteration count
        """
        if len(target_newtons) != 6:
            raise ValueError("Need 6 target Newton values")
        
        results = {
            'target_newtons': target_newtons,
            'achieved_newtons': None,
            'force_thresholds_used': None,
            'iterations': 0,
            'success': False
        }
        
        for iteration in range(max_iterations):
            # Set force thresholds based on current target
            response = self.force_set_by_newton_target(target_newtons, finger_lengths)
            
            if response is None:
                print(f"Iteration {iteration + 1}: Failed to set force thresholds")
                continue
            
            # Wait a moment for the hand to respond
            time.sleep(0.5)
            
            # Read current Newton values
            current_newtons = self.get_newton_readings_from_forces()
            
            if current_newtons is None:
                print(f"Iteration {iteration + 1}: Failed to read Newton values")
                continue
            
            results['achieved_newtons'] = current_newtons
            results['iterations'] = iteration + 1
            
            # Check if targets are achieved within tolerance
            all_within_tolerance = True
            for i, (target, achieved) in enumerate(zip(target_newtons, current_newtons)):
                if abs(target - achieved) > tolerance:
                    all_within_tolerance = False
                    break
            
            if all_within_tolerance:
                results['success'] = True
                print(f"Newton targets achieved in {iteration + 1} iterations")
                break
            
            # Adjust targets for next iteration (simple proportional adjustment)
            adjustment_factor = 1.1  # 10% adjustment
            for i in range(len(target_newtons)):
                if current_newtons[i] < target_newtons[i] - tolerance:
                    target_newtons[i] *= adjustment_factor
                elif current_newtons[i] > target_newtons[i] + tolerance:
                    target_newtons[i] /= adjustment_factor
            
            print(f"Iteration {iteration + 1}: Adjusting targets. Current: {['%.3f' % x for x in current_newtons]}")
        
        if not results['success']:
            print(f"Failed to achieve Newton targets after {max_iterations} iterations")
        
        return results

    def adaptive_force_control(self, target_forces: List[int], target_angles: List[int], 
                             step_size: int = 50, max_iterations: int = 20):
        """
        Advanced force control with adaptive adjustment and gradual positioning
        
        Args:
            target_forces: List of 6 target force values in grams
            target_angles: List of 6 target angles (0-1000)
            step_size: Angle reduction step size (default 50)
            max_iterations: Maximum number of iterations
        
        Returns:
            dict: Results with final readings and adjustment history
        """
        if len(target_forces) != 6 or len(target_angles) != 6:
            raise ValueError("Need 6 values for both forces and angles")
        
        finger_names = ["Pinky", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotation"]
        
        # Record original target forces for comparison
        original_targets = target_forces.copy()
        current_targets = target_forces.copy()
        
        # Get current angles as starting position
        current_angles = self.angle_read()
        if current_angles is None:
            print("Failed to read initial angles, using default [1000, 1000, 1000, 1000, 1000, 1000]")
            current_angles = [1000, 1000, 1000, 1000, 1000, 1000]
        
        print("=== Adaptive Force Control Started ===")
        print(f"Original target forces: {original_targets}")
        print(f"Target angles: {target_angles}")
        print(f"Starting angles: {current_angles}")
        print(f"Step size: {step_size}")
        
        results = {
            'original_targets': original_targets,
            'final_forces': None,
            'final_angles': None,
            'iterations': 0,
            'adjustment_history': []
        }
        
        for iteration in range(max_iterations):
            print(f"\n--- Iteration {iteration + 1} ---")
            
            # Set current force thresholds
            print(f"Setting force thresholds: {current_targets}")
            response = self.force_set(current_targets)
            if not response:
                print("Failed to set force thresholds")
                continue
            
            # Calculate next angle step
            next_angles = []
            angles_at_target = True
            
            for i in range(6):
                if current_angles[i] > target_angles[i]:
                    # Move towards target by step_size
                    next_angle = max(target_angles[i], current_angles[i] - step_size)
                    next_angles.append(next_angle)
                    if next_angle != target_angles[i]:
                        angles_at_target = False
                elif current_angles[i] < target_angles[i]:
                    # Move towards target by step_size
                    next_angle = min(target_angles[i], current_angles[i] + step_size)
                    next_angles.append(next_angle)
                    if next_angle != target_angles[i]:
                        angles_at_target = False
                else:
                    # Already at target
                    next_angles.append(target_angles[i])
            
            print(f"Moving to angles: {next_angles}")
            self.angle_set(next_angles)
            current_angles = next_angles.copy()
            
            # Wait for movement to complete
            time.sleep(2)
            
            # Read current forces
            current_forces = self.force_act()
            if not current_forces:
                print("Failed to read current forces")
                continue
            
            print("Current readings vs Original targets vs Current thresholds:")
            print("Finger      | Current(g) | Original(g) | Threshold(g) | Diff | Action")
            print("-" * 75)
            
            adjustment_made = False
            iteration_adjustments = []
            
            for i, name in enumerate(finger_names):
                current_force = current_forces[i]
                original_target = original_targets[i]
                current_threshold = current_targets[i]
                diff = abs(current_force - original_target)
                
                action = "No change"
                
                # Check if current angle is close to target angle (within ±30)
                angle_close_to_target = abs(current_angles[i] - target_angles[i]) <= 30
                
                # Apply adjustment logic
                if angle_close_to_target:
                    # Stop force adjustments when angle is close to target
                    action = f"Angle close to target ({current_angles[i]} vs {target_angles[i]}), force adjustment stopped"
                elif diff > 100:
                    # Large difference: don't adjust force until angles reach target
                    if angles_at_target:
                        action = "Large diff, angles at target - consider manual adjustment"
                    else:
                        action = "Large diff, waiting for angle target"
                elif 50 < diff <= 100:
                    # Medium difference: increase threshold by 50g
                    if current_threshold < 1000:  # Don't exceed maximum
                        current_targets[i] = min(1000, current_threshold + 50)
                        action = f"Increased threshold by 50g (was {current_threshold})"
                        adjustment_made = True
                    else:
                        action = "At max threshold (1000g)"
                else:
                    # Small difference: no adjustment needed
                    action = "Within tolerance"
                
                print(f"{name:11} | {current_force:10d} | {original_target:11d} | {current_targets[i]:11d} | {diff:4.0f} | {action}")
                
                iteration_adjustments.append({
                    'finger': name,
                    'current_force': current_force,
                    'original_target': original_target,
                    'threshold': current_targets[i],
                    'diff': diff,
                    'action': action
                })
            
            results['adjustment_history'].append({
                'iteration': iteration + 1,
                'angles': current_angles.copy(),
                'forces': current_forces.copy(),
                'thresholds': current_targets.copy(),
                'adjustments': iteration_adjustments,
                'angles_at_target': angles_at_target
            })
            
            # Check if we should continue
            if angles_at_target and not adjustment_made:
                print(f"\nTarget angles reached and no more adjustments needed")
                break
            
            results['iterations'] = iteration + 1
        
        # Final readings
        final_forces = self.force_act()
        final_angles = self.angle_read()
        
        results['final_forces'] = final_forces
        results['final_angles'] = final_angles
        
        print(f"\n=== Adaptive Force Control Complete ===")
        print(f"Completed in {results['iterations']} iterations")
        print(f"Final angles: {final_angles}")
        print(f"Final forces: {final_forces}")
        print(f"Final thresholds: {current_targets}")
        
        return results

def demonstrate_force_calibration(port: str, hand_id: int = 1):
    """
    Demonstrate force sensor calibration process
    
    Args:
        port: Serial port device path
        hand_id: Hand ID, default is 1
    """
    print("Starting force sensor calibration demonstration...")
    hand = RH56Hand(port=port, hand_id=hand_id)
    
    # Read current angles and forces
    print("Pre-calibration angles:", hand.angle_read())
    forces = hand.force_act()
    if forces:
        print("Pre-calibration finger forces:", forces)
    
    input("Press Enter to start calibration...")
    
    # Start calibration
    print("Starting calibration, process will take about 15 seconds...")
    hand.gesture_force_clb(1)
    
    # Wait for calibration process
    print("Calibrating: All fingers opening...")
    time.sleep(3)
    print("Calibrating: Four fingers bending...")
    time.sleep(3)
    print("Calibrating: Four fingers opening...")
    time.sleep(3)
    print("Calibrating: Thumb bending...")
    time.sleep(3)
    print("Calibrating: Thumb opening...")
    time.sleep(3)
    
    # Wait for system stabilization
    print("Calibration complete, waiting for system stabilization...")
    time.sleep(2)
    
    # Read post-calibration forces
    print("Reading post-calibration forces...")
    attempts = 0
    forces = None
    finger_names = ["Pinky", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotation"]
    while attempts < 3:
        forces = hand.force_act()
        if forces:
            print(f"Post-calibration finger forces (Attempt {attempts+1}):")
            for i, force in enumerate(forces):
                print(f"{finger_names[i]}: {force:4d} g")
            break
        print(f"Attempt {attempts+1} failed to read forces, retrying...")
        attempts += 1
        time.sleep(1)
    
    if not forces:
        print("Warning: Unable to read post-calibration forces, check device connection")
    
    print("Calibration process complete, recommend saving parameters")
    save = input("Save parameters? (y/n): ")
    if save.lower() == 'y':
        hand.save_parameters()
        print("Parameters saved")
    
    # Add verification test
    test = input("Test post-calibration force sensor response? (y/n): ")
    if test.lower() == 'y':
        print("Press each finger in sequence to observe force changes (Press Ctrl+C to exit)...")
        try:
            while True:
                forces = hand.force_act()
                if forces:
                    print("Current forces (unit: g):")
                    for i, force in enumerate(forces):
                        print(f"{finger_names[i]}: {force:4d}", end="  ")
                    print()  # New line
                else:
                    print("Unable to read forces")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nTest complete")

if __name__ == "__main__":
    # Basic force control test using grams directly
    hand = RH56Hand(port="/dev/tty.usbserial-1130", hand_id=1) # 1 for right hand, 2 for left hand
    #demonstrate_force_calibration(port="/dev/tty.usbserial-1130", hand_id=1)
    
    # Setup custom finger lengths (optional)
    finger_lengths = {
        0: (0.032, 0.041),  # Pinky finger
        1: (0.032, 0.046),  # Ring finger
        2: (0.032, 0.051),  # Middle finger
        3: (0.032, 0.046),  # Index finger
        4: (0.05, 0.05),    # Thumb bend
        5: (0.05, 0.05),    # Thumb rotation
    }
    hand.speed_set([1000, 1000, 1000, 1000, 1000, 1000])
    hand.angle_set([1000, 1000, 1000, 500, 1000, 1000])
    time.sleep(1)

    # Set speeds
    hand.speed_set([10, 10, 10, 10, 10, 10])
    
    # Use new adaptive force control system
    target_forces = [100, 100, 100, 500, 100, 100]  # Target force readings in grams
    target_angles = [1000, 1000, 1000, 0, 1000, 1000]    # Target angles (0-1000)
    
    print(f"Starting adaptive force control...")
    print(f"Target forces: {target_forces}")
    print(f"Target angles: {target_angles}")
    
    # Run the adaptive force control
    results = hand.adaptive_force_control(
        target_forces=target_forces,
        target_angles=target_angles,
        step_size=50,  # Reduce angle by 50 each step
        max_iterations=20
    )
    
    # Display summary of results
    print(f"\n=== Final Summary ===")
    print(f"Process completed in {results['iterations']} iterations")
    if results['final_forces'] and results['final_angles']:
        finger_names = ["Pinky", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotation"]
        print("\nFinal Results:")
        print("Finger      | Final Angle | Final Force | Original Target | Difference")
        print("-" * 70)
        for i, name in enumerate(finger_names):
            final_force = results['final_forces'][i]
            final_angle = results['final_angles'][i]
            original_target = results['original_targets'][i]
            diff = abs(final_force - original_target)
            print(f"{name:11} | {final_angle:11d} | {final_force:11d} | {original_target:15d} | {diff:10.0f}")
    
    print("\nAdaptive force control demo complete!")
    
    # Commented out Newton-based control for now
    """
    # Set target Newton values for each finger
    target_newtons = [2, 2, 2, 4, 2, 2]  # Newton values
    print(f"Setting Newton targets: {target_newtons}")
    
    try:
        response = hand.force_set_by_newton_target(target_newtons, finger_lengths)
        if response:
            print("Successfully set Newton-based force thresholds")
            # Close hand to activate force control
            hand.angle_set([1000, 1000, 1000, 0, 1000, 1000])
            time.sleep(5)
        else:
            print("Failed to set Newton-based force thresholds")
    except Exception as e:
        print(f"Error in Newton control: {e}")
    
    # Monitor both traditional and Newton readings
    print("\n=== Monitoring Readings ===")
    finger_names = ["Pinky", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotation"]
    
    for i in range(10):  # Monitor for 10 iterations
        print(f"\nIteration {i+1}:")
        
        # Read angles and forces
        angles = hand.angle_read()
        forces = hand.force_act()
        newton_readings = hand.get_newton_readings_from_forces()
        
        if angles and forces and newton_readings:
            print("Finger      | Angle | Force(g) | Newton(N) | Target(N)")
            print("-" * 55)
            for j, name in enumerate(finger_names):
                target_n = target_newtons[j] if j < len(target_newtons) else 0
                print(f"{name:11} | {angles[j]:5d} | {forces[j]:8d} | {newton_readings[j]:9.3f} | {target_n:8.1f}")
        else:
            print("Failed to read sensor values")
        
        time.sleep(1)
    
    print("\nDemo complete!")
    """
    
    # Uncomment below for additional functionality:
    #hand.set_id(2)
    #hand.save_parameters()
    #hand.gesture_force_clb(1)  # Calibrate force sensors for gesture ID 1
    #demonstrate_force_calibration(port="/dev/tty.usbserial-2130", hand_id=2)
