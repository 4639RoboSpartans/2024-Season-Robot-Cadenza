package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public enum ShootingMode {
        AUTO_SPEAKER,
        MANUAL_SPEAKER,
        IDLE,
        LAUNCH
    }

    public record ShooterSetpoint(double speed, double angle) {}

    public static final double SHOOTER_LOWER_OFFSET = 0.92;

    public static final double SHOOTER_PIVOT_MECHANISM_OFFSET_DEGREES = 0;

    public static final ShooterSetpoint SHOOTER_IDLE = new ShooterSetpoint(15, SHOOTER_LOWER_OFFSET - 0.07);

    public static final ShooterSetpoint SHOOTER_LAUNCH = new ShooterSetpoint(30, SHOOTER_LOWER_OFFSET - 0.045);

    public static final ShooterSetpoint SHOOTER_MANUAL_SPEAKER = new ShooterSetpoint(28, SHOOTER_LOWER_OFFSET - 0.095);

    public static final double SHOOTER_VOLTAGE_MULTIPLIER = 10;

    public static final double SHOOTER_PIVOT_TOLERANCE = 0.01;
    public static final double SHOOTER_SPEED_TOLERANCE = 0.001;

    public static final double SHOOTER_PID_kp = 640;
    public static final double SHOOTER_PID_ki = 0;
    public static final double SHOOTER_PID_kd = 0;
    public static final double SHOOTER_PID_kv = 1.22;
    public static final double SHOOTER_PID_ka = 10000;

    public static final double SHOOTER_PIVOT_kp = 4;
    public static final double SHOOTER_PIVOT_ki = 0;
    public static final double SHOOTER_PIVOT_kd = 0;
    public static final double SHOOTER_PIVOT_VELOCITY = 2;
    public static final double SHOOTER_PIVOT_ACCELERATION = 2;

    public class IDs {
        public static final int SHOOTER_PIVOT_MOTOR_LEFT = 14;
        public static final int SHOOTER_PIVOT_MOTOR_RIGHT = 19;

        public static final int SHOOTER_SHOOTER_LEFT_MOTOR = 13;
        public static final int SHOOTER_SHOOTER_RIGHT_MOTOR = 17;

        public static final int SHOOTER_PIVOT_ENCODER_DIO_PORT = 0;
    }
}