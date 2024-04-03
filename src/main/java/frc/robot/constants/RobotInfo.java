package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class RobotInfo {
    public static final class SwerveInfo {
        public static final double centerToWheel = 0.245;

        public static final int K_P_MULTIPLIER = 1;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(new Translation2d(centerToWheel, centerToWheel), new Translation2d(centerToWheel, -centerToWheel), new Translation2d(-centerToWheel, centerToWheel), new Translation2d(-centerToWheel, -centerToWheel));

        // Change MOVEMENT_SPEED to 1.0 for max speed
        public static final double CURRENT_MAX_ROBOT_MPS = 6;
        public static final double MOVEMENT_SPEED = .5;
        public static final double AIM_ROTATION_SPEED = 2.5;
        public static final PIDTemplate SWERVE_ROTATOR_PID_CONSTANTS = new PIDTemplate(0.0085);
        public static final PIDTemplate SWERVE_DRIVER_PID_CONSTANTS = new PIDTemplate(3.25);
        public static final double DERIVATIVE_MULTIPLIER = 0.1;
        public static final double TELOP_ROTATION_SPEED = 4;

        public static final PIDConstants TranslationPID = new PIDConstants(56, 0, 0.0);
        public static final PIDConstants RotationPID = new PIDConstants(64, 0, 0.0);
    }

    public static final class TrapInfo {
        // TODO: make trap subsystem and put constants here
    }

    public static final class IntakeInfo {

        public static final double INTAKE_SPEED = -0.8;
        public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.3;
        public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.52;
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
        public static final PIDTemplate LIMELIGHT_AIM_PID_CONSTANTS = new PIDTemplate(0.12, 0.01, 0);
        public static final double AIM_ROT_POW = 0.7;
        public static final double AIM_UPWARDS_TILT = Math.PI/6;

    }

    public static class ShooterInfo {
        public static double LimelightTxOffset = -8;
        public static double LimelightOffsetZ = -0.1651;
        public static double LimelightOffsetX = -0.1267;
        public record ShooterSetpoint(double speed, double angle) {
        }

        public record ShooterSetpointMeasurement(double distance, ShooterInfo.ShooterSetpoint setpoint) {
        }

        public enum ShootingMode {
            AUTO_SPEAKER, SPEAKER, AMP, TRAP, IDLE, LAUNCH, INTAKE
        }

        public static final ShooterInfo.ShooterSetpointMeasurement[] measurements = {
                new ShooterInfo.ShooterSetpointMeasurement(1.1, new ShooterInfo.ShooterSetpoint(24, .812)),
                new ShooterInfo.ShooterSetpointMeasurement(1.2, new ShooterInfo.ShooterSetpoint(24, .815)),
                new ShooterInfo.ShooterSetpointMeasurement(1.3, new ShooterInfo.ShooterSetpoint(24.5, .817)),
                new ShooterInfo.ShooterSetpointMeasurement(1.4, new ShooterInfo.ShooterSetpoint(24.5, .8215)),
                new ShooterInfo.ShooterSetpointMeasurement(1.5, new ShooterInfo.ShooterSetpoint(25, .8255)),
                new ShooterInfo.ShooterSetpointMeasurement(1.6, new ShooterInfo.ShooterSetpoint(25, .829)),
                new ShooterInfo.ShooterSetpointMeasurement(1.7, new ShooterInfo.ShooterSetpoint(25, .834)),
                new ShooterInfo.ShooterSetpointMeasurement(1.8, new ShooterInfo.ShooterSetpoint(25.5, .8365)),
                new ShooterInfo.ShooterSetpointMeasurement(1.9, new ShooterInfo.ShooterSetpoint(26, .8395)),
                new ShooterInfo.ShooterSetpointMeasurement(2.0, new ShooterInfo.ShooterSetpoint(26, .842)),    
                new ShooterInfo.ShooterSetpointMeasurement(2.1, new ShooterInfo.ShooterSetpoint(26.5, .845)),    
                new ShooterInfo.ShooterSetpointMeasurement(2.2, new ShooterInfo.ShooterSetpoint(26.5, .8495)),
                new ShooterInfo.ShooterSetpointMeasurement(2.3, new ShooterInfo.ShooterSetpoint(26.75, .8575)),
                new ShooterInfo.ShooterSetpointMeasurement(2.4, new ShooterInfo.ShooterSetpoint(27, .8605)),
                new ShooterInfo.ShooterSetpointMeasurement(2.5, new ShooterInfo.ShooterSetpoint(27.25, .863)),
                new ShooterInfo.ShooterSetpointMeasurement(2.6, new ShooterInfo.ShooterSetpoint(27.65, .863)),
                new ShooterInfo.ShooterSetpointMeasurement(2.7, new ShooterInfo.ShooterSetpoint(28.25, .865)),
                new ShooterInfo.ShooterSetpointMeasurement(2.8, new ShooterInfo.ShooterSetpoint(29.25, .8685)),
                new ShooterInfo.ShooterSetpointMeasurement(2.9, new ShooterInfo.ShooterSetpoint(30.6, .871)),
                new ShooterInfo.ShooterSetpointMeasurement(3.0, new ShooterInfo.ShooterSetpoint(32.25, .8735)),
                new ShooterInfo.ShooterSetpointMeasurement(3.1, new ShooterInfo.ShooterSetpoint(34.5, .876)),
                new ShooterInfo.ShooterSetpointMeasurement(3.2, new ShooterInfo.ShooterSetpoint(37, .8815)),
                new ShooterInfo.ShooterSetpointMeasurement(3.3, new ShooterInfo.ShooterSetpoint(39.5, .884)),
                new ShooterInfo.ShooterSetpointMeasurement(3.4, new ShooterInfo.ShooterSetpoint(42, .886)),
                new ShooterInfo.ShooterSetpointMeasurement(3.5, new ShooterInfo.ShooterSetpoint(45, .886)),
                new ShooterInfo.ShooterSetpointMeasurement(3.6, new ShooterInfo.ShooterSetpoint(47.5, .89)),
                new ShooterInfo.ShooterSetpointMeasurement(3.7, new ShooterInfo.ShooterSetpoint(50, .892)),
                new ShooterInfo.ShooterSetpointMeasurement(3.8, new ShooterInfo.ShooterSetpoint(52.5, .892)),
                new ShooterInfo.ShooterSetpointMeasurement(3.9, new ShooterInfo.ShooterSetpoint(54.5, .893)),
                new ShooterInfo.ShooterSetpointMeasurement(4.0, new ShooterInfo.ShooterSetpoint(55.75, .8945)),
                new ShooterInfo.ShooterSetpointMeasurement(4.1, new ShooterInfo.ShooterSetpoint(56.5, .896)),
                new ShooterInfo.ShooterSetpointMeasurement(4.2, new ShooterInfo.ShooterSetpoint(57.25, .897)),
                new ShooterInfo.ShooterSetpointMeasurement(4.3, new ShooterInfo.ShooterSetpoint(58.5, .898))
                // Max speed is 58.5
                // Min angle is .9
            
        };

        public static final double SHOOTER_IDLE_SPEED = 0.2;
        public static final double SHOOTER_INTAKE_SPEED = -0.35;
        public static final double SHOOTER_PIVOT_BOTTOM_SETPOINT = .90;

        public static final double SHOOTER_PIVOT_ERROR = 0.01;

        public static final ShooterInfo.ShooterSetpoint SHOOTER_AMP_SETPOINT = new ShooterInfo.ShooterSetpoint(
                6.9,
                .811
        );
        public static final ShooterInfo.ShooterSetpoint SHOOTER_SPEAKER_SETPOINT = measurements[2].setpoint;
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
