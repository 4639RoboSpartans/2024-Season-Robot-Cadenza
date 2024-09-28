package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingMode;
import frc.robot.util.AimUtil;

import java.util.Objects;

public abstract class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    public static PivotSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcretePivotSubsystem::new);
    }

    public Command runShootingMode(ShooterConstants.ShootingMode mode) {
        return run(() -> runShootingAngle(getTargetAngle(mode)));
    }

    public Trigger atSetPoint() {
        return new Trigger(this::atSetPointSupplier);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
    }

    protected abstract void buildSendable(SendableBuilder builder);

    public abstract double getRotations();

    public abstract void instantiateMech(Mechanism2d mech);

    public abstract void runShootingAngle(double angle);

    protected abstract boolean atSetPointSupplier();

    private double getTargetAngle(ShootingMode mode) {
        return switch (mode) {
            case IDLE -> ShooterConstants.SHOOTER_IDLE.angle();
            case AUTO_SPEAKER -> AimUtil.getShooterSetpoint().angle();
            case MANUAL_SPEAKER -> ShooterConstants.SHOOTER_MANUAL_SPEAKER.angle();
            case LAUNCH -> ShooterConstants.SHOOTER_LAUNCH.angle();
        };
    }
}
