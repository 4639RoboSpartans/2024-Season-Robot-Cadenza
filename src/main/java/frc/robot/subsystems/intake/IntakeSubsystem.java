package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class IIntakeSubsystem extends SubsystemBase {
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
        return runOnce(() -> setExtendedState(state));
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
}
