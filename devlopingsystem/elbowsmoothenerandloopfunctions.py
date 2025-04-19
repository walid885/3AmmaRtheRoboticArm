def smooth_elbow_movement(self, current, target, velocity):
    """Special handling for elbow servo to compensate for gravity effects"""
    # Constants for elbow-specific gravity compensation
    GRAVITY_FACTOR = 0.3  # Strength of gravity compensation
    EXTRA_SMOOTHING = 0.4  # Additional smoothing factor for elbow
    
    # Calculate standard movement parameters
    diff = target - current
    
    # Apply gravity compensation based on current position
    # More compensation when elbow is extended (higher pulse width)
    min_pw, max_pw = self.SERVO_LIMITS['ELBOW']
    position_ratio = (current - min_pw) / (max_pw - min_pw)  # 0 = retracted, 1 = extended
    
    # Apply stronger gravity assistance when moving up (reducing pulse width)
    # and less assistance when moving down (increasing pulse width)
    if diff < 0:  # Moving up against gravity
        gravity_assist = -GRAVITY_FACTOR * position_ratio * self.settings['STEP_SIZE']
    else:  # Moving down with gravity
        gravity_assist = GRAVITY_FACTOR * (1 - position_ratio) * self.settings['STEP_SIZE'] * 0.5
    
    # Adjust velocity with gravity compensation
    adjusted_velocity = velocity + gravity_assist
    
    # Apply extra smoothing for elbow movements
    new_position = current + adjusted_velocity
    smoothed_position = (current * (self.settings['SMOOTHING'] + EXTRA_SMOOTHING) + 
                         new_position * (1 - (self.settings['SMOOTHING'] + EXTRA_SMOOTHING)))
    
    # Ensure smoothing factor doesn't exceed 0.95
    effective_smoothing = min(0.95, self.settings['SMOOTHING'] + EXTRA_SMOOTHING)
    smoothed_position = current * effective_smoothing + new_position * (1 - effective_smoothing)
    
    return smoothed_position 


def smooth_movement_loop(self):
    """Background thread that handles smooth movement of all servos"""
    while True:
        # Process each servo that has a target different from current position
        active_movement = False
        for name in self.SERVO_PINS.keys():
            # Skip inactive servos to save power
            if name not in self.active_servos and abs(self.target_pw[name] - self.current_pw[name]) < 1:
                continue
                
            # Calculate ideal movement with acceleration/deceleration
            target = self.target_pw[name]
            current = self.current_pw[name]
            diff = target - current
            
            # Apply acceleration/deceleration logic
            if abs(diff) > 1:  # Only move if difference is significant
                active_movement = True
                
                # Calculate velocity target based on distance to target
                distance_factor = min(1.0, abs(diff) / 100)  # Normalize distance effect
                max_velocity = self.settings['STEP_SIZE'] * self.speed_factor.get() * distance_factor
                
                # Apply acceleration if moving toward target
                if abs(self.velocities[name]) < max_velocity:
                    # Accelerate
                    self.velocities[name] += math.copysign(
                        self.settings['ACCELERATION'] * max_velocity,
                        diff
                    )
                    # Cap velocity
                    self.velocities[name] = math.copysign(
                        min(abs(self.velocities[name]), max_velocity),
                        self.velocities[name]
                    )
                elif abs(diff) < abs(self.velocities[name]) * 2:
                    # Decelerate as we approach target
                    self.velocities[name] *= (1.0 - self.settings['DECELERATION'])
                
                # Ensure velocity direction matches target direction
                if (self.velocities[name] * diff) < 0:
                    # If velocity direction opposes target direction, decelerate faster
                    self.velocities[name] *= (1.0 - self.settings['DECELERATION'] * 2)
                
                # Apply smoothing to current position
                if name == 'ELBOW':
                    # Use special elbow smoothing function
                    new_position = self.smooth_elbow_movement(
                        current, target, self.velocities[name]
                    )
                else:
                    # Standard movement for other servos
                    new_position = current + self.velocities[name]
                    # Apply standard smoothing
                    new_position = (current * self.settings['SMOOTHING'] + 
                                   new_position * (1 - self.settings['SMOOTHING']))
                
                # Ensure within limits
                min_pw, max_pw = self.SERVO_LIMITS[name]
                new_position = max(min_pw, min(new_position, max_pw))
                
                # Update current position
                self.current_pw[name] = new_position
                
                # Update servo position
                pin = self.SERVO_PINS[name]
                self.pi.set_servo_pulsewidth(pin, int(self.current_pw[name]))
                
                # Update position label and slider if available
                if hasattr(self, 'position_labels') and name in self.position_labels:
                    self.root.after_idle(lambda n=name, p=int(self.current_pw[name]): 
                                        self.position_labels[n].config(text=f"{p}μs"))
            else:
                # Reached target, stop velocity
                self.velocities[name] = 0
                
                # If servo is active but no movement needed, update final position
                if name in self.active_servos:
                    self.current_pw[name] = target
                    pin = self.SERVO_PINS[name]
                    self.pi.set_servo_pulsewidth(pin, int(target))
                
        # Pause for movement interval
        time.sleep(self.settings['SPEED_INTERVAL'] / 1000.0)