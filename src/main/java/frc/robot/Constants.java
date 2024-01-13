package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final class IDs {
        // Swerve uses up motor ids 1-12
        public static final SwerveModuleConfig MODULE_FRONT_LEFT = new SwerveModuleConfig(1, 2, 9, 126.1231);//124.77
        public static final SwerveModuleConfig MODULE_FRONT_RIGHT = new SwerveModuleConfig(3, 4, 10, -124.1016);//233.877
        public static final SwerveModuleConfig MODULE_BACK_LEFT = new SwerveModuleConfig(5, 6, 11, 10.6348);//9.668
        public static final SwerveModuleConfig MODULE_BACK_RIGHT = new SwerveModuleConfig(7, 8, 12, 48.7793);//50.400

        public static final int SHOOTER_MOTOR_LEFT = 13;
        public static final int SHOOTER_MOTOR_RIGHT = 14;

        public static final int CLIMBER_LEFT = 15;
        public static final int CLIMBER_RIGHT = 16;
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

        public static final double ROTATOR_MOTOR_KP = 0.05;
        public static final double ROTATOR_MOTOR_KI = 0.07;
    }

    public static final double DEADZONE_VALUE = 0.01;

    public record SwerveModuleConfig(
            int driveMotorID,
            int rotatorMotorID,
            int encoderID,
            double rotationOffset
    ) {}
}