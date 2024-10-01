package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingMode;
import frc.robot.util.AimUtil;

import java.util.Objects;

public abstract class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;
    private final MedianFilter filter = new MedianFilter(20);

    public static ShooterSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                Robot.isReal() ? FalconShooterSubsystem::new
                        : SimShooterSubsystem::new);
    }

    public Command runShootingMode(ShootingMode mode) {
        return run(() -> runShootingSpeed(getTargetSpeed(mode)));
    }

    public Trigger atSetPoint() {
        return new Trigger(this::atSetPointSupplier);
    }

    protected abstract boolean atSetPointSupplier();

    protected abstract void runShootingSpeed(double speed);

    protected abstract double getCurrentSpeed();

    private double getRawTargetSpeed(ShootingMode mode) {
        return switch (mode) {
            case IDLE -> ShooterConstants.SHOOTER_IDLE.speed();
            case MANUAL_SPEAKER -> ShooterConstants.SHOOTER_MANUAL_SPEAKER.speed();
            case AUTO_SPEAKER -> AimUtil.getShooterSetpoint().speed();
            case LAUNCH -> ShooterConstants.SHOOTER_LAUNCH.speed();
        };
    }

    private double getTargetSpeed(ShootingMode mode) {
        return filter.calculate(getRawTargetSpeed(mode));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
    }

    protected abstract void buildSendable(SendableBuilder builder);

    public abstract Command getSysIDQuasistaticCommand();

    public abstract Command getSysIDDynamicCommand();
}