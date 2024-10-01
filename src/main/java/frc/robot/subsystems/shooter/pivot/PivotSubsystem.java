package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingMode;
import frc.robot.util.AimUtil;

import java.util.Objects;

public abstract class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;
    private ShootingMode mode = ShootingMode.IDLE;

    public static PivotSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                Robot.isReal()? ConcretePivotSubsystem::new
                : SimPivotSubsystem::new);
    }

    public Command setShootingMode(ShooterConstants.ShootingMode mode) {
        return runOnce(() -> this.mode = mode);
    }

    public Trigger atSetPoint() {
        return new Trigger(this::atSetPointSupplier);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
        builder.addStringProperty("Mode",
                () -> mode.name(),
                null);
    }

    protected abstract void buildSendable(SendableBuilder builder);

    public abstract double getRotations();

    public abstract void initMech(Mechanism2d mech);

    public abstract void runShootingAngle(double angle);

    protected abstract boolean atSetPointSupplier();

    protected double getTargetAngle() {
        return switch (mode) {
            case IDLE -> ShooterConstants.SHOOTER_IDLE.angle();
            case AUTO_SPEAKER -> AimUtil.getShooterSetpoint().angle();
            case MANUAL_SPEAKER -> ShooterConstants.SHOOTER_MANUAL_SPEAKER.angle();
            case LAUNCH -> ShooterConstants.SHOOTER_LAUNCH.angle();
        };
    }
}
