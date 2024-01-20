package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final class IDs {
        // Swerve uses up motor ids 1-12
        public static final SwerveModuleConfig MODULE_FRONT_LEFT = new SwerveModuleConfig(
                1, 2, 9, 125.02
        );//124.77
        public static final SwerveModuleConfig MODULE_FRONT_RIGHT = new SwerveModuleConfig(
                3, 4, 10, 57.6
        );//233.877
        public static final SwerveModuleConfig MODULE_BACK_LEFT = new SwerveModuleConfig(
                5, 6, 11, -105.56
        );//9.668
        public static final SwerveModuleConfig MODULE_BACK_RIGHT = new SwerveModuleConfig(
                7, 8, 12, -58.08
        );//50.400

        public static final int SHOOTER_MOTOR_LEFT = 13;
        public static final int SHOOTER_MOTOR_RIGHT = 14;

        public static final int CLIMBER_LEFT = 15;
        public static final int CLIMBER_RIGHT = 16;
        public static final int AIM_MOTOR = 17;

        public static final int LEFT_ROTATOR_MOTOR = 18;
        public static final int RIGHT_ROTATOR_MOTOR = 19;
        public static final int HOOK_MOTOR = 20;
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

        public static final double MOVEMENT_SPEED = 0.75;
        public static final double CLIMBER_LEFT_SPEED = 0.5;
        public static final double CLIMBER_RIGHT_SPEED = 0.5;
        public static final double TRAMP_ROTATOR_SPEED = 0.5;
        public static final double TRAMP_HOOK_RELEASE_SPEED = 0.5;
        public static final double TRAMP_ROLLER_RELEASE_SPEED = 0.5;
        public static final double TRAMP_BACK_ROTATOR_DEGREES = -30;
        public static final double TRAMP_FRONT_ROTATOR_DEGREES = 90;
        public static final double TRAMP_HOOK_RETRACT_DEGREES = 0;
        public static final double TRAMP_HOOK_RELEASE_DEGREES = 90;
        public static final double TRAMP_RETRACT_TIME = 5;
        public static final double TRAMP_EXTEND_TIME = 15;
        public static final double TRAMP_RELEASE_TIME = 20;
        public static final double TRAMP_FINAL_TIME = 25;
        public static final double INTAKE_SPEED = 0.5;

        public static final PID SWERVE_ROTATOR_PID = new PID(
                0.0085
        );

        // TODO: find actual values
        public static final PID SHOOTER_AIM_PID = new PID(
            0, 0, 0
        );

        public static final PID TRAMP_ROTATOR_PID = new PID(
            0, 0, 0
        );

        public static final PID HOOK_ROTATOR_PID = new PID(
            0, 0, 0
        );
    }

    public static final double DEADZONE_VALUE = 0.05;

    public record SwerveModuleConfig(
            int driveMotorID,
            int rotatorMotorID,
            int encoderID,
            double rotationOffset,
            double rotatorPIDkPMultiplier
    ) {
        public SwerveModuleConfig(int driveMotorID, int rotatorMotorID, int encoderID, double rotationOffset){
            this(driveMotorID, rotatorMotorID, encoderID, rotationOffset, 1);
        }
    }

    public record PID(double kp, double ki, double kd) {
        public PID(double kp) { this(kp, 0); }
        public PID(double kp, double ki) { this(kp, ki, 0); }

        public PIDController create() {
            return create(1);
        }

        public PIDController create(double kPMultipler) {
            return new PIDController(kp * kPMultipler, ki, kd);
        }
    }
}