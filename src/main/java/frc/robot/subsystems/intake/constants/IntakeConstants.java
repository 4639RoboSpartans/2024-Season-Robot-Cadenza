package frc.robot.subsystems.intake.constants;


public class IntakeConstants {
    public static final double INTAKE_SPEED = -0.95;
    public static final double AMP_OUTTAKE_SPEED = 0.6;
    public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.154;
    public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.83;
    public static final double INTAKE_PIVOT_AMP_SETPOINT = 0.254;
    public static final double INTAKE_PIVOT_kp = 2.3, INTAKE_PIVOT_ki = 0, INTAKE_PIVOT_kd = 0;
    public static final double INTAKE_PIVOT_VELOCITY = 2.5, INTAKE_PIVOT_ACCELERATION = 1.5;

    public class IDs {
        public static final int INTAKE_PIVOT_MOTOR_LEFT = 22;
        public static final int INTAKE_PIVOT_MOTOR_RIGHT = 23;
        public static final int INTAKE_MOTOR = 24;

        public static final int INTAKE_ENCODER_DIO_PORT = 1;
    }
}
