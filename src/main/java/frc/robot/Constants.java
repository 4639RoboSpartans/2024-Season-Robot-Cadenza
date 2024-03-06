package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.oi.OI;

public final class Constants {
    public static final double DEADZONE_VALUE = 0.05;
    public static final int POSE_WINDOW_LENGTH = 5;
    public static final double INTAKE_PIVOT_UP_MULTIPLIER = 2;

    public enum CurrentRobot {
        ZEUS, SIREN
    }

    public static final CurrentRobot currentRobot = CurrentRobot.SIREN;

    private static final double[] oldOffsets = {
        -54.98, -122.4, 74.44, 121.92
    }, newOffsets = {
        135.36, 155.23, -94.98, -88.23
    };

    private static final double[] offsets = switch (currentRobot) {
        case ZEUS -> oldOffsets;
        case SIREN -> newOffsets;
    };

    public static final class IDs {
        // Swerve uses up motor ids 1-12
        public static final SwerveModuleConfig MODULE_FRONT_LEFT = new SwerveModuleConfig(1, 2, 9, offsets[0]);//124.77
        public static final SwerveModuleConfig MODULE_FRONT_RIGHT = new SwerveModuleConfig(3, 4, 10, offsets[1]);//233.877
        public static final SwerveModuleConfig MODULE_BACK_LEFT = new SwerveModuleConfig(5, 6, 11, offsets[2]);//9.668
        public static final SwerveModuleConfig MODULE_BACK_RIGHT = new SwerveModuleConfig(7, 8, 12, offsets[3]);//50.400

        public static final int SHOOTER_SHOOTER_MOTOR = 13;
        public static final int SHOOTER_PIVOT_MOTOR = 14;

        public static final int CLIMBER_LEFT = 15;
        public static final int CLIMBER_RIGHT = 16;

        public static final int HOPPER_MOTOR = 18;

        public static final int INTAKE_PIVOT_MOTOR_LEFT = 22;
        public static final int INTAKE_PIVOT_MOTOR_RIGHT = 23;
        public static final int INTAKE_MOTOR = 24;


        public static final int SHOOTER_PIVOT_ENCODER_CHANNEL = 0;
        public static final int INTAKE_ENCODER_CHANNEL = 1;
        public static final int IR_SENSOR = 3;
    }

    public static final class RobotInfo {
        public static final class SwerveInfo {
            public static final double centerToWheel = 0.245;

            public static final int K_P_MULTIPLIER = 5;
            public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(new Translation2d(centerToWheel, centerToWheel), new Translation2d(centerToWheel, -centerToWheel), new Translation2d(-centerToWheel, centerToWheel), new Translation2d(-centerToWheel, -centerToWheel));

            // Change MOVEMENT_SPEED to 1.0 for max speed
            public static final double CURRENT_MAX_ROBOT_MPS = 3;
            public static final double MAX_ROBOT_MPS = 3;
            public static final double MAX_ROBOT_MPS_SHOOTING = .5;
            public static final double MOVEMENT_SPEED = .5;
            public static final double AIM_ROTATION_SPEED = 2.5;
            public static final PID SWERVE_ROTATOR_PID = new PID(0.0085);
            public static final PID SWERVE_DRIVER_PID = new PID(0.65);
        }

        public static final class TrapInfo {
            // TODO: make trap subsystem and put constants here
        }

        public static final class IntakeInfo {

            public static final double INTAKE_SPEED = -0.6 * 1.6;
            public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.45;
            public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.69;
            public static final PID INTAKE_PIVOT_PID = new PID(.8, 0, 0);
        }

        public static final class HopperInfo {

            public static final double HOPPER_SPEED = 0.7;
            public static boolean usingIRSensor = true;
        }

        public static final class ClimberInfo {

            public static final double CLIMBER_SPEED = 0.9;
        }

        public static final class AimInfo {

            public static final double AIM_TOLERANCE = Math.toRadians(1);
            public static final PID LIMELIGHT_AIM_PID = new PID(0.1, 0.001, 0.015);
            public static final double AIM_ROT_POW = 0.7;

        }

        public static class ShooterInfo {
            public record ShooterSetpoint(double speed, double angle) {}

            public record ShooterSetpointMeasurement(double distance, ShooterSetpoint setpoint) {}

            public enum ShootingMode {
                AUTO_SPEAKER, SPEAKER, AMP, TRAP, IDLE
            }

            public static final ShooterSetpointMeasurement[] measurements = {
                    new ShooterSetpointMeasurement(2.4, new ShooterSetpoint(30, .845)),
                    new ShooterSetpointMeasurement(2.6, new ShooterSetpoint(35, .862)),
                    new ShooterSetpointMeasurement(2.84, new ShooterSetpoint(40, .875)),
                    new ShooterSetpointMeasurement(3.2, new ShooterSetpoint(40, .885)),
                    new ShooterSetpointMeasurement(3.6, new ShooterSetpoint(45, .892)),
                    new ShooterSetpointMeasurement(3.8, new ShooterSetpoint(42, .9)),
            };

            public static final double SHOOTER_IDLE_SPEED = 0.2;
            public static final double SHOOTER_PIVOT_BOTTOM_SETPOINT = .90;

            public static final double SHOOTER_PIVOT_ERROR = 0.01;

            public static final ShooterSetpoint SHOOTER_AMP_SETPOINT = new ShooterSetpoint(
                    13.55,
                    .85
            );
            public static final ShooterSetpoint SHOOTER_SPEAKER_SETPOINT = new ShooterSetpoint(
                    40,
                    .835
            );
            public static final ShooterSetpoint SHOOTER_TRAP_SETPOINT = new ShooterSetpoint(
                    20,
                    .82
            );

            public static final double SHOOTER_VOLTAGE = 10;

            public static final PID SHOOTER_AIM_PID = new PID(3.5, 0, 0);
        }
    }

    public record SwerveModuleConfig(int driveMotorID, int rotatorMotorID, int encoderID, double rotationOffset,
                                     double rotatorPIDkPMultiplier) {
        public SwerveModuleConfig(int driveMotorID, int rotatorMotorID, int encoderID, double rotationOffset) {
            this(driveMotorID, rotatorMotorID, encoderID, rotationOffset, 1);
        }
    }

    public record PID(double kp, double ki, double kd) {
        public PID(double kp) {
            this(kp, 0);
        }

        public PID(double kp, double ki) {
            this(kp, ki, 0);
        }

        public PIDController create() {
            return create(1);
        }

        public PIDController create(double kPMultiplier) {
            return new PIDController(kp * kPMultiplier, ki, kd);
        }
    }

    public static final class Controls {

        public static final class DriverControls {
            public static final OI.Axes SwerveForwardAxis = OI.Axes.LEFT_STICK_Y;
            public static final OI.Axes SwerveStrafeAxis = OI.Axes.LEFT_STICK_X;
            public static final OI.Axes SwerveRotationAxis = OI.Axes.RIGHT_STICK_X;
            public static final OI.Buttons AimButton = OI.Buttons.LEFT_TRIGGER;

            public static final OI.Buttons ClimberExtendButton = OI.Buttons.LEFT_BUMPER;
            public static final OI.Buttons ClimberRetractButton = OI.Buttons.RIGHT_BUMPER;
            public static final OI.Buttons ClimberSwap1Button = OI.Buttons.POV_LEFT;
            public static final OI.Buttons ClimberSwap2Button = OI.Buttons.POV_RIGHT;

            public static final OI.Buttons AmpAlignButton = OI.Buttons.X_BUTTON;
        }

        public static final class OperatorControls {
            public static final OI.Buttons RunSpeakerShooterButton = OI.Buttons.RIGHT_TRIGGER;
            public static final OI.Buttons RunAmpShooterButton = OI.Buttons.LEFT_BUMPER;
            public static final OI.Buttons ManualShooterButton = OI.Buttons.LEFT_TRIGGER;

            public static final OI.Buttons RunTrapShooterButton = OI.Buttons.RIGHT_BUMPER;

            public static final OI.Buttons IntakeButton = OI.Buttons.X_BUTTON;
            public static final OI.Buttons OuttakeButton = OI.Buttons.Y_BUTTON;
            public static final OI.Buttons IntakeExtendButton = OI.Buttons.POV_DOWN;
            public static final OI.Buttons IntakeRetractButton = OI.Buttons.POV_UP;

            public static final OI.Buttons ToggleIR = OI.Buttons.A_BUTTON;

        }
    }
}
