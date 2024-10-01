package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

import java.util.Objects;

public abstract class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                Robot.isReal()? ConcreteIntakeSubsystem::new
                : SimIntakeSubsystem::new);
    }

    public abstract double getRotations();

    public abstract void initMech(Mechanism2d mech);

    public enum ExtensionState {
        EXTENDED, RETRACTED, AMP
    }

    protected abstract void setExtendedState(ExtensionState extended);

    protected abstract void outtakeRun();

    protected abstract void ampRun();

    protected abstract void stopIntakeRun();

    protected abstract void stopRun();

    protected abstract void intakeRun();

    public Command setExtended(ExtensionState state) {
        return runOnce(() -> setExtendedState(state))
                .andThen(
                        Commands.waitUntil(
                                atSetPoint()
                        )
                );
    }

    public Command intake() {
        return runOnce(this::intakeRun);
    }

    public Command outtake() {
        return runOnce(this::outtakeRun);
    }

    public Command amp() {
        return runOnce(this::ampRun);
    }

    public Command stopIntake() {
        return runOnce(this::stopIntakeRun);
    }

    public Command stop() {
        return runOnce(this::stopRun);
    }

    protected abstract Trigger atSetPoint();

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
        builder.setSmartDashboardType("Intake");
    }

    protected abstract void buildSendable(SendableBuilder builder);
}
