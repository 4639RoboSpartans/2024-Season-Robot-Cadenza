package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DummyIntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private ExtensionState state;

    public DummyIntakeSubsystem() {
        state = ExtensionState.RETRACTED;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake state", state.toString());
    }

    @Override
    public Command setExtended(ExtensionState extended) {
        return null;
    }

    @Override
    public Command outtake() {
        return null;
    }

    @Override
    public Command amp() {
        return null;
    }

    @Override
    public Command stopIntake() {
        return null;
    }

    @Override
    public Command stop() {
        return null;
    }

    @Override
    public Command intake() {
        return null;
    }
}
