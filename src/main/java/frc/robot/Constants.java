package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.oi.OI;


public final class Constants {
    public static final double DEADZONE_VALUE = 0.05;

    private static final double[] oldOffsets = {
        -54.98,
        -122.4,
        74.44,
        121.92
    }, newOffsets = {
        -45.44,
        -16.52,
        84.2,
        -99.45
    };
    private static final double[] offsets = oldOffsets;

    public static final class IDs {
        // Swerve uses up motor ids 1-12
        public static final SwerveModuleConfig MODULE_FRONT_LEFT = new SwerveModuleConfig(
                1, 2, 9, offsets[0]
        );//124.77
        public static final SwerveModuleConfig MODULE_FRONT_RIGHT = new SwerveModuleConfig(
                3, 4, 10, offsets[1]
        );//233.877
        public static final SwerveModuleConfig MODULE_BACK_LEFT = new SwerveModuleConfig(
                5, 6, 11, offsets[2]
        );//9.668
        public static final SwerveModuleConfig MODULE_BACK_RIGHT = new SwerveModuleConfig(
                7, 8, 12, offsets[3]
        );//50.400

        public static final int SHOOTER_MOTOR_LEFT = 13;
        public static final int SHOOTER_MOTOR_RIGHT = 14;

        public static final int CLIMBER_LEFT = 15;
        public static final int CLIMBER_RIGHT = 16;
        public static final int AIM_MOTOR = 17;

        public static final int LEFT_ROTATOR_MOTOR = 18;
        public static final int RIGHT_ROTATOR_MOTOR = 19;
        public static final int ROLLER_MOTOR = 21;
    }

    public static final class RobotInfo {
        public static final double centerToWheel = 0.245;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(centerToWheel, centerToWheel),
                new Translation2d(centerToWheel, -centerToWheel),
                new Translation2d(-centerToWheel, centerToWheel),
                new Translation2d(-centerToWheel, -centerToWheel)
        );

        public static final double MOVEMENT_SPEED = 0.5;
        public static final double TRAP_ROTATOR_SPEED = 0.5;
        public static final double TRAP_ROLLER_RELEASE_SPEED = 0.5;
        public static final double TRAP_ROLLER_INTAKE_SPEED = 0.5;
        public static final double TRAP_BACK_ROTATOR_DEGREES = -30;
        public static final double TRAP_FRONT_ROTATOR_DEGREES = 90;
        public static final double TRAP_EXTEND_TIME = 15;
        public static final double TRAP_RELEASE_TIME = 20;
        public static final double TRAP_FINAL_TIME = 25;
        public static final double INTAKE_SPEED = 0.5;
        public static final double CLIMBER_SPEED = 0.5;

        public static final PID SWERVE_ROTATOR_PID = new PID(
            0.0085
        );

        // TODO: find actual values
        public static final PID SHOOTER_AIM_PID = new PID(
                0, 0, 0
        );

        public static final PID TRAP_ROTATOR_PID = new PID(
                0, 0, 0
        );

    }

    public record SwerveModuleConfig(
            int driveMotorID,
            int rotatorMotorID,
            int encoderID,
            double rotationOffset,
            double rotatorPIDkPMultiplier
    ) {
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
        public static final OI.Axes ShooterPivotAxis = OI.Axes.LEFT_STICK_Y;
        public static final OI.Buttons TrapReleaseButton = OI.Buttons.RIGHT_BUMPER;
        public static final OI.Buttons ShooterButton = OI.Buttons.LEFT_BUMPER;
        public static final OI.Buttons ClimberExtendButton = OI.Buttons.Y_BUTTON;
        public static final OI.Buttons ClimberRetractButton = OI.Buttons.X_BUTTON;

        public static final OI.Axes SwerveForwardAxis = OI.Axes.LEFT_STICK_Y;
        public static final OI.Axes SwerveStrafeAxis = OI.Axes.LEFT_STICK_X;
        public static final OI.Axes SwerveRotationAxis = OI.Axes.RIGHT_STICK_X;
    }
}