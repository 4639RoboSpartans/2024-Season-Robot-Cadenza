package frc.robot.constants;

public final class IDs {
    // Swerve uses up motor ids 1-12
    public static final SwerveModuleConfiguration MODULE_FRONT_LEFT = new SwerveModuleConfiguration(
        1, 2, 9, Constants.swerveOffsets[0]//, 0.8
    );
    public static final SwerveModuleConfiguration MODULE_FRONT_RIGHT = new SwerveModuleConfiguration(
        3, 4, 10, Constants.swerveOffsets[1]//, 0.5
    );
    public static final SwerveModuleConfiguration MODULE_BACK_LEFT = new SwerveModuleConfiguration(
        5, 6, 11, Constants.swerveOffsets[2]//, 0.8
    );
    public static final SwerveModuleConfiguration MODULE_BACK_RIGHT = new SwerveModuleConfiguration(
        7, 8, 12, Constants.swerveOffsets[3]//, 0.8
    );

    public static final int SHOOTER_SHOOTER_LEFT_MOTOR = 13;
    public static final int SHOOTER_PIVOT_MOTOR_LEFT = 14;
    public static final int SHOOTER_PIVOT_MOTOR_RIGHT = 19;

    public static final int CLIMBER_LEFT = 15;
    public static final int CLIMBER_RIGHT = 16;

    public static final int HOPPER_MOTOR = 18;

    public static final int SHOOTER_SHOOTER_RIGHT_MOTOR = 17;

    public static final int INTAKE_PIVOT_MOTOR_LEFT = 22;
    public static final int INTAKE_PIVOT_MOTOR_RIGHT = 23;
    public static final int INTAKE_MOTOR = 24;

    public static final int SHOOTER_PIVOT_ENCODER_DIO_PORT = 0;
    public static final int INTAKE_ENCODER_DIO_PORT = 1;
    public static final int IR_SENSOR_1_DIO_PORT = 3;
    public static final int IR_SENSOR_2_DIO_PORT = 6;
}
