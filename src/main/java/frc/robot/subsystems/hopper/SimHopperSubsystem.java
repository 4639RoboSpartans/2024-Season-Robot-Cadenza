package frc.robot.subsystems.hopper;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SimHopperSubsystem extends HopperSubsystem {
    private boolean hasNoteBool;
    private double hopperOutput;

    public SimHopperSubsystem() {
        hopperOutput = 0;
        hasNoteBool = false;
    }

    @Override
    protected void feedRun() {
        hopperOutput = HopperConstants.HOPPER_SPEED;
    }

    @Override
    protected void outtakeRun() {
        hopperOutput = -HopperConstants.HOPPER_SPEED;

    }

    @Override
    protected void stopRun() {
        hopperOutput = 0;
    }

    @Override
    protected void toggleIRRun() {}

    @Override
    protected boolean hasNoteSupplier() {
        if (Robot.isInAuton()) return true;
        return hasNoteBool;
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hopper");
        builder.addDoubleProperty("Hopper output",
                () -> hopperOutput,
                null);
        builder.addBooleanProperty("Has note",
                () -> hasNoteBool,
                null);
    }

    public Command simToggleHasNote(boolean hasNote) {
        return runOnce(() -> hasNoteBool = hasNote);
    }
}
