package frc.robot.constants;


import edu.wpi.first.math.controller.PIDController;

public final class RobotInfo {
    public static final class SwerveInfo {
        public static final double centerToWheel = 0.245;

        // Change MOVEMENT_SPEED to 1.0 for max speed
        public static final double CURRENT_MAX_ROBOT_MPS = 4.6;
        public static final double TELOP_ROTATION_SPEED = 12;


        public static final PIDController choreoX = new PIDController(7, 0, 0.1);
        public static final PIDController choreoY = new PIDController(7, 0, 0.1);
        public static final PIDController choreoRotation = new PIDController(1.5, 0, 0);
        public static final double TIME_BEFORE_INTAKE_START = 1;
    }

    public static final class IntakeInfo {

        public static final double INTAKE_SPEED = -0.95;
        public static final double AMP_OUTTAKE_SPEED = 0.6;
        public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.04;
        public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.732;
        public static final double INTAKE_PIVOT_AMP_SETPOINT = 0.179;
        public static final PIDTemplate INTAKE_PIVOT_PID_CONSTANTS = new PIDTemplate(1.2, 0, 0);
    }

    public static final class HopperInfo {

    }

    public static final class ClimberInfo {
        public static final double CLIMBER_SPEED = 0.9;
    }

    public static final class AimInfo {

        public static final double AIM_TOLERANCE = Math.toRadians(1);
        public static final PIDTemplate LIMELIGHT_AIM_PID_CONSTANTS = new PIDTemplate(2, 0.01, 0);
        public static final double AIM_ROT_POW = 1.2;
        public static final double AIM_UPWARDS_TILT = Math.PI / 6;
        public static final double AIM_TIME = 0.5;
    }

    public static class ShooterInfo {
        public static double LimelightTxOffset = -8;
        public static double LimelightOffsetZ = -0.1651;
        public static double LimelightOffsetX = -0.1267;


    }
}
