package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummyIntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private ExtensionState state;

    public DummyIntakeSubsystem() {
        state = ExtensionState.RETRACTED;
    }

    @Override
    public void setExtended(ExtensionState extended) {
        state = extended;
    }

    public void stop() {
    }

    public void intake() {
        
    }

    public void outtake() {

    }

    public void stopIntake() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake state", state.toString());
    }
}
