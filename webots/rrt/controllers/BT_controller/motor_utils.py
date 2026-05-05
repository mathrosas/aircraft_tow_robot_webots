def configure_speeds(name):
        # Placeholder for the method that sets MAX_SPEED, p1, and p2 based on the task
        if name == "Move around the aerodrome":
            MAX_SPEED = 10  # Simulation velocity for mapping
        else:
            MAX_SPEED = 6.64  # Real velocity for other tasks
        p1 = MAX_SPEED * 2
        p2 = MAX_SPEED * 0.3
        return p1, p2, MAX_SPEED

def set_motor_positions(leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor, position):
    motors = [leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor]
    for motor in motors:
        motor.setPosition(position)

def set_motor_velocities(leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor, left_velocity, right_velocity=None):
    if right_velocity is None:
        right_velocity = left_velocity
    leftBackMotor.setVelocity(left_velocity)
    leftFrontMotor.setVelocity(left_velocity)
    rightBackMotor.setVelocity(right_velocity)
    rightFrontMotor.setVelocity(right_velocity)