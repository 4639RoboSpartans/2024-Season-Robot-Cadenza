package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.lib.util.Helpers;

public class FalconShooterSubsystem extends ShooterSubsystem {
    private final TalonFX shooterLeft, shooterRight;
    private double targetVelocity;
    private final BangBangController controller;

    public FalconShooterSubsystem() {
        shooterLeft = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_LEFT_MOTOR, "Canivore1");
        shooterRight = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_RIGHT_MOTOR, "Canivore1");
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ShooterConstants.SHOOTER_PID_kp)
                                .withKI(ShooterConstants.SHOOTER_PID_ki)
                                .withKD(ShooterConstants.SHOOTER_PID_kd)
                                .withKV(ShooterConstants.SHOOTER_PID_kv))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(4)
                                .withStatorCurrentLimitEnable(true));
        shooterLeft.getConfigurator().apply(shooterConfig);
        shooterRight.getConfigurator().apply(shooterConfig);
        shooterRight.setControl(new Follower(shooterLeft.getDeviceID(), true));
        targetVelocity = ShooterConstants.SHOOTER_IDLE.speed();
        controller = new BangBangController(ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    }

    @Override
    protected boolean atSetPointSupplier() {
        return Helpers.withinTolerance(
                getCurrentSpeed(),
                targetVelocity,
                ShooterConstants.SHOOTER_SPEED_TOLERANCE
        );
    }

    @Override
    protected void runShootingSpeed(double speed) {
        targetVelocity = speed;
        double controllerOutput = controller.calculate(
                getCurrentSpeed(), speed);
        shooterLeft.setVoltage(controllerOutput * ShooterConstants.SHOOTER_VOLTAGE_MULTIPLIER);
    }

    @Override
    protected double getCurrentSpeed() {
        return shooterLeft.getVelocity().getValueAsDouble();
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        builder.addDoubleProperty("Speed",
                this::getCurrentSpeed,
                null);
    }

    @Override
    public Command getSysIDQuasistaticCommand() {
        return null;
    }

    @Override
    public Command getSysIDDynamicCommand() {
        return null;
    }
}