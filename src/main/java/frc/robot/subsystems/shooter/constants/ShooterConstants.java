package frc.robot.subsystems.shooter.constants;

import frc.robot.constants.PIDTemplate;
import frc.robot.constants.RobotInfo;

public class ShooterConstants {public static double ShooterKv = 0.0000001;

    public static double ShooterKp = 0;

    public static double ShooterKi = 0;

    public static double ShooterKd = 0;

    public record ShooterSetpoint(double speed, double angle) {
    }

    public record ShooterSetpointMeasurement(double distance, ShooterSetpoint setpoint) {
    }

    public enum ShootingMode {
        AUTO_SPEAKER, SPINUP, MANUAL_SPEAKER, AMP, IDLE, LAUNCH, INTAKE
    }

    public static final double SHOOTER_IDLE_SPEED = 0.2;
    public static final double SHOOTER_AUTON_IDLE_SPEED = 0.35;
    public static final double SHOOTER_INTAKE_SPEED = -0.35;
    public static final double SHOOTER_IDLE_SETPOINT = .85;
    public static final double AngleOffset = -0.005;

    public static double ShooterLowerOffset = 0.92;

    public static final double SHOOTER_PIVOT_ERROR = 0.01;

    public static final ShooterSetpoint SHOOTER_AMP_SETPOINT = new ShooterSetpoint(
            6.8,
            .8
    );
    public static final ShooterSetpoint SHOOTER_SPEAKER_SETPOINT =
            new ShooterSetpointMeasurement(1.2, new ShooterSetpoint(24.25, ShooterLowerOffset - 0.09)).setpoint;
    public static final ShooterSetpoint SHOOTER_LAUNCH_SETPOINT = new ShooterSetpoint(
            30,
            .875
    );

    public static final ShooterSetpoint SHOOTER_INTAKE_SETPOINT = new ShooterSetpoint(
            -15,
            .8
    );

    public static final double SHOOTER_VOLTAGE = 10;

    public static final PIDTemplate SHOOTER_AIM_PID_CONSTANTS = new PIDTemplate(4, 0, 0);


    public static final int SHOOTER_PIVOT_ENCODER_DIO_PORT = 0;
}
