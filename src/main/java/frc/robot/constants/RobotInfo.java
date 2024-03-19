package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class RobotInfo {
    public static final class SwerveInfo {
        public static final double centerToWheel = 0.245;

        public static final int K_P_MULTIPLIER = 1;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(new Translation2d(centerToWheel, centerToWheel), new Translation2d(centerToWheel, -centerToWheel), new Translation2d(-centerToWheel, centerToWheel), new Translation2d(-centerToWheel, -centerToWheel));

        // Change MOVEMENT_SPEED to 1.0 for max speed
        public static final double CURRENT_MAX_ROBOT_MPS = 3;
        public static final double MOVEMENT_SPEED = .5;
        public static final double AIM_ROTATION_SPEED = 2.5;
        public static final PIDTemplate SWERVE_ROTATOR_PID_CONSTANTS = new PIDTemplate(0.0085);
        public static final PIDTemplate SWERVE_DRIVER_PID_CONSTANTS = new PIDTemplate(3.25);
        public static final double DERIVATIVE_MULTIPLIER = 0.1;
        public static final double TELOP_ROTATION_SPEED = 4;
    }

    public static final class TrapInfo {
        // TODO: make trap subsystem and put constants here
    }

    public static final class IntakeInfo {

        public static final double INTAKE_SPEED = -0.6 * 1.6;
        public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.45;
        public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.69;
        public static final PIDTemplate INTAKE_PIVOT_PID_CONSTANTS = new PIDTemplate(.8, 0, 0);
    }

    public static final class HopperInfo {

        public static final double HOPPER_SPEED = 0.7;
    }

    public static final class ClimberInfo {

        public static final double CLIMBER_SPEED = 0.9;
    }

    public static final class AimInfo {

        public static final double AIM_TOLERANCE = Math.toRadians(1);
        public static final PIDTemplate LIMELIGHT_AIM_PID_CONSTANTS = new PIDTemplate(0.085, 0.001, 0.015);
        public static final double AIM_ROT_POW = 0.7;

    }

    public static class ShooterInfo {
        public static double LimelightTxOffset = -5;
        public static double LimelightOffsetY = -0.1651;
        public static double LimelightOffsetX = -0.1524;
        public record ShooterSetpoint(double speed, double angle) {
        }

        public record ShooterSetpointMeasurement(double distance, ShooterInfo.ShooterSetpoint setpoint) {
        }

        public enum ShootingMode {
            AUTO_SPEAKER, SPEAKER, AMP, TRAP, IDLE, LAUNCH, INTAKE
        }

        public static final ShooterInfo.ShooterSetpointMeasurement[] measurements = {
                new ShooterInfo.ShooterSetpointMeasurement(1.15, new ShooterInfo.ShooterSetpoint(28, .808)),
                new ShooterInfo.ShooterSetpointMeasurement(1.2, new ShooterInfo.ShooterSetpoint(28, .809)),
                new ShooterInfo.ShooterSetpointMeasurement(1.3, new ShooterInfo.ShooterSetpoint(27.875, .811)),
                new ShooterInfo.ShooterSetpointMeasurement(1.4, new ShooterInfo.ShooterSetpoint(27.75, .814)),
                new ShooterInfo.ShooterSetpointMeasurement(1.5, new ShooterInfo.ShooterSetpoint(27.75, .82)),
                new ShooterInfo.ShooterSetpointMeasurement(1.6, new ShooterInfo.ShooterSetpoint(27.75, .825)),
                new ShooterInfo.ShooterSetpointMeasurement(1.7, new ShooterInfo.ShooterSetpoint(27.75, .83)),
                new ShooterInfo.ShooterSetpointMeasurement(1.8, new ShooterInfo.ShooterSetpoint(27.75, .834)),
                new ShooterInfo.ShooterSetpointMeasurement(1.9, new ShooterInfo.ShooterSetpoint(26.75, .842)),
                new ShooterInfo.ShooterSetpointMeasurement(2.0, new ShooterInfo.ShooterSetpoint(27.25, .84325)),    
                new ShooterInfo.ShooterSetpointMeasurement(2.1, new ShooterInfo.ShooterSetpoint(27.25, .84375)),    
                new ShooterInfo.ShooterSetpointMeasurement(2.2, new ShooterInfo.ShooterSetpoint(28, .8445)),
                new ShooterInfo.ShooterSetpointMeasurement(2.3, new ShooterInfo.ShooterSetpoint(28.5, .855)),    
                new ShooterInfo.ShooterSetpointMeasurement(2.4, new ShooterInfo.ShooterSetpoint(30, .8605)),
                new ShooterInfo.ShooterSetpointMeasurement(2.5, new ShooterInfo.ShooterSetpoint(32, .86775)),
                new ShooterInfo.ShooterSetpointMeasurement(2.6, new ShooterInfo.ShooterSetpoint(33.5, .873)),
                new ShooterInfo.ShooterSetpointMeasurement(2.7, new ShooterInfo.ShooterSetpoint(36, .878)),
                new ShooterInfo.ShooterSetpointMeasurement(2.8, new ShooterInfo.ShooterSetpoint(39, .88325)),
                new ShooterInfo.ShooterSetpointMeasurement(2.9, new ShooterInfo.ShooterSetpoint(41, .887)),
                new ShooterInfo.ShooterSetpointMeasurement(3.0, new ShooterInfo.ShooterSetpoint(44, .889)),
                new ShooterInfo.ShooterSetpointMeasurement(3.1, new ShooterInfo.ShooterSetpoint(46, .889)),
                new ShooterInfo.ShooterSetpointMeasurement(3.2, new ShooterInfo.ShooterSetpoint(48, .8905)),
                new ShooterInfo.ShooterSetpointMeasurement(3.3, new ShooterInfo.ShooterSetpoint(50, .892)),
                new ShooterInfo.ShooterSetpointMeasurement(3.4, new ShooterInfo.ShooterSetpoint(52, .8935)),
                new ShooterInfo.ShooterSetpointMeasurement(3.5, new ShooterInfo.ShooterSetpoint(54.5, .895)),
                new ShooterInfo.ShooterSetpointMeasurement(3.6, new ShooterInfo.ShooterSetpoint(57.5, .8965)),
                new ShooterInfo.ShooterSetpointMeasurement(3.7, new ShooterInfo.ShooterSetpoint(57.5, .89675)),
                // Max speed is 57.5
                // Min angle is .9
                // Max angle is .84375
        };

        public static final double SHOOTER_IDLE_SPEED = 0.2;
        public static final double SHOOTER_PIVOT_BOTTOM_SETPOINT = .90;

        public static final double SHOOTER_PIVOT_ERROR = 0.01;

        public static final ShooterInfo.ShooterSetpoint SHOOTER_AMP_SETPOINT = new ShooterInfo.ShooterSetpoint(
                8.5,
                .79
        );
        public static final ShooterInfo.ShooterSetpoint SHOOTER_SPEAKER_SETPOINT = new ShooterInfo.ShooterSetpoint(
                44,
                .889
        );
        public static final ShooterInfo.ShooterSetpoint SHOOTER_TRAP_SETPOINT = new ShooterInfo.ShooterSetpoint(
                20,
                .82
        );
        public static final ShooterInfo.ShooterSetpoint SHOOTER_LAUNCH_SETPOINT = new ShooterInfo.ShooterSetpoint(
            30, 
            .85
        );

        public static final ShooterInfo.ShooterSetpoint SHOOTER_INTAKE_SETPOINT = new ShooterInfo.ShooterSetpoint(
                -15,
                .85
        );

        public static final double SHOOTER_VOLTAGE = 10;

        public static final PIDTemplate SHOOTER_AIM_PID_CONSTANTS = new PIDTemplate(4, 0, 0);
    }
}
