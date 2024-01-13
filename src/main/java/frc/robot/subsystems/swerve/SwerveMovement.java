package frc.robot.subsystems.swerve;

import math.vec2;

/**
 * A struct containing a desired translation and a rotation of the swerve drive.
 * Positive strideMovement is to the left of the robot
 * rotation should be in degrees
 */
public record SwerveMovement(double forwardMovement, double strideMovement, double rotation){
    public SwerveMovement(vec2 movement, double rotation){
        this(movement.x(), movement.y(), rotation);
    }

    /**
     * @return The movement represented by this swerve movement. +X is forward
     */
    public vec2 movement(){
        return new vec2(forwardMovement(), strideMovement());
    }

    public String toString(){
        return "SwerveMovement{mvmt=" + movement() + ", rotate=" + rotation + "}";
    }
}