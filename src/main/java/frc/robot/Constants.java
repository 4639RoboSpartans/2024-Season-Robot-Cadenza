package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    public static class LEDInfo {
        public static int LEDStatus = 0;
        // 0 = Blue and orange stirps, disabled status
        // 1 = Green, has object
        // 2 = Purple, need Cube
        // 3 = Yellow, need Cone
        // 4 = Red, does not have object
        public static final int BlueR = 3;
        public static final int BlueG = 23;
        public static final int BlueB = 252;

        public static final int OrangeR = 252;
        public static final int OrangeG = 61;
        public static final int OrangeB = 3;

        public static final int PurpleR = 252;
        public static final int PurpleG = 6;
        public static final int PurpleB = 244;

        public static final int YellowR = 255;
        public static final int YellowG = 255;
        public static final int YellowB = 0;

        public static final int GreenR = 19;
        public static final int GreenG = 252;
        public static final int GreenB = 3;

        public static final int RedR = 252;
        public static final int RedG = 3;
        public static final int RedB = 3;
    }

    public static final class RobotInfo {
        public static final double robotBaseLength = 0.44;
        public static final double centerToWheel = 0.245;
        public static final double fullRobotBaseLength = 0.84;
        public static final double SWERVE_KP = 0.1;
        public static final double SWERVE_KI = 0.1;

        public static final double MOVEMENT_SPEED = 0.75; // 0 - 1
        public static final double MAX_VELOCITY = 4;
        public static final double MAX_ACCELERATION = 3;
        public static final double CLIMBER_LEFT_SPEED = 0.5;
        public static final double CLIMBER_RIGHT_SPEED = 0.5;

        //https://github.com/SeanSun6814/FRC0ToAutonomous/blob/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot/Constants.java#L57


        public static class DriveConstants {
            public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    new Translation2d(centerToWheel, centerToWheel),
                    new Translation2d(centerToWheel, -centerToWheel),
                    new Translation2d(-centerToWheel, centerToWheel),
                    new Translation2d(-centerToWheel, -centerToWheel)
            );
        }

        public static final double ROTATOR_MOTOR_KP = 0.05;
        public static final double ROTATOR_MOTOR_KI = 0.07;

        public static class Auton {
            public static final double POSITION_KP = 5.0;
            public static final double POSITION_KI = 0.0;

            public static final double ROTATION_KP = 5.0;
            public static final double ROTATION_KI = 0.0;

            public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
            public static final double kMaxAngularSpeedRadiansPerSecond = //
                    DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
            public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
            public static final double kPXController = 1.5;
            public static final double kPYController = 1.5;
            public static final double kPThetaController = 3;

            public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                    new TrapezoidProfile.Constraints(
                            kMaxAngularSpeedRadiansPerSecond,
                            kMaxAngularAccelerationRadiansPerSecondSquared);
        }
    }

    public static final class Timing {
        public static final double CLAW_DELAY_AFTER_OPEN = 0.5;
    }

    public static final double DEADZONE_VALUE = 0.01;
    public static final int NUMBER_OF_CONTROLLERS = 2;

    public record SwerveModuleConfig(
            int driveMotorID,
            int rotaterMotorID,
            int encoderID,
            double rotationOffset
    ) {
    }

    public enum Axes {
        LEFT_STICK_X(0), LEFT_STICK_Y(1),
        LEFT_TRIGGER(2), RIGHT_TRIGGER(3),
        RIGHT_STICK_X(4), RIGHT_STICK_Y(5);

        private final int value;

        Axes(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum Buttons {
        A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
                7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

        private final int value;

        private Buttons(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
}