package frc.robot.subsystems.hopper;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Objects;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;

    public static HopperSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteHopperSubsystem::new);
    }

    public Command toggleIR() {
        return runOnce(this::toggleIRRun);
    }

    public Command feed() {
        return runOnce(this::feedRun);
    }

    public Command outake() {
        return runOnce(this::outtakeRun);
    }
    public Command stop() {
        return runOnce(this::stopRun);
    }

    public Trigger hasNote() {
        return new Trigger(this::hasNoteSupplier);
    }

    protected abstract void feedRun();

    protected abstract void outtakeRun();

    protected abstract void stopRun();

    protected abstract void toggleIRRun();

    protected abstract boolean hasNoteSupplier();

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
    }

    protected abstract void buildSendable(SendableBuilder builder);
}
