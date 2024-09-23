package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

public class SimIntake extends SubsystemBase implements IIntakeSubsystem {
    enum ExtensionState {
        EXTENDED,
        RETRACTED,
        AMP
    }

    private ExtensionState extensionState = ExtensionState.RETRACTED;

    public static SimIntake instance;

    private SimIntake() {
    }

    public Command stop() {
        return null;
    }

    public Command intake() {
        return null;
    }

    public Command outtake() {
        return null;
    }

    @Override
    public Command pivotExtend() {
        return runOnce(() -> extensionState = ExtensionState.EXTENDED);
    }

    @Override
    public Command pivotRetract() {
        return runOnce(() -> extensionState = ExtensionState.RETRACTED);
    }

    @Override
    public Command pivotOuttake() {
        return null;
    }

    @Override
    public Command ampPrep() {
        return null;
    }

    @Override
    public Command pivotIntake() {
        return null;
    }

    @Override
    public Command pivotAmp() {
        return runOnce(() -> extensionState = ExtensionState.AMP);
    }

    @Override
    public Command stopIntake() {
        return null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.addStringProperty("State", extensionState::toString, (Consumer <String>) null);
    }

    public static SimIntake getInstance() {
        if (instance == null) {
            instance = new SimIntake();
        }
        return instance;
    }
}