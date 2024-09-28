package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingMode;
import frc.robot.util.AimUtil;

import java.util.Objects;

public abstract class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    public static ShooterSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, FalconShooterFeedforward::new);
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

    private double getTargetSpeed(ShootingMode mode) {
        return switch (mode) {
            case IDLE -> ShooterConstants.SHOOTER_IDLE.speed();
            case MANUAL_SPEAKER -> ShooterConstants.SHOOTER_MANUAL_SPEAKER.speed();
            case AUTO_SPEAKER -> AimUtil.getShooterSetpoint().speed();
            case LAUNCH -> ShooterConstants.SHOOTER_LAUNCH.speed();
        };
    }
}