package frc.robot.subsystems.hopper;

import edu.wpi.first.util.sendable.SendableBuilder;

public class SimHopperSubsystem extends HopperSubsystem {
    private double hopperOutput;

    public SimHopperSubsystem() {
        hopperOutput = 0;
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
        return true;
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hopper");
        builder.addDoubleProperty("Hopper output",
                () -> hopperOutput,
                null);
    }
}
