package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import math.math;
import math.vec2;

public class SwerveUtil {
    private SwerveUtil() {}

    /**
     * Converts a field-centric {@link SwerveMovement} to robot-centric.
     * That is, this method converts a swerve movement measured relative to the field
     * to a swerve movement relative to the robot
     * @param m The field-centric swerve movement
     * @param robotHeading The direction that the robot is pointing, measured counterclockwise in degrees
     * @return The robot-centric swerve movement
     */
    public static SwerveMovement toRobotCentric(SwerveMovement m, double robotHeading) {
        vec2 rawMovement = m.movement();
        double rawRotation = m.rotation();

        vec2 movement = math.rotateCW(rawMovement, robotHeading);

        return new SwerveMovement(movement, rawRotation);
    }

    /**
     * Converts a {@link SwerveModuleState} to an equivalent state that minimizes the angle that the swerve module needs to rotate.
     * @param state The swerve module state to optimize
     * @param currRotation The direction that the swerve module is currently pointing in
     * @return An optimized swerve module state
     */
    public static  SwerveModuleState optimize(SwerveModuleState state, double currRotation){
        double degrees = state.angle.getDegrees();
        double speed = state.speedMetersPerSecond;

        double rawAngleDifference = currRotation - degrees;

        double angleDifference = math.mod(rawAngleDifference, -90, 90);
        boolean shouldReverse = Math.abs(math.mod(rawAngleDifference, -180, 180)) >= 90;

        double finalAngle = currRotation - angleDifference;
        double finalSpeed = shouldReverse ? -speed : speed;

        return new SwerveModuleState(finalSpeed, Rotation2d.fromDegrees(finalAngle));
    }
}