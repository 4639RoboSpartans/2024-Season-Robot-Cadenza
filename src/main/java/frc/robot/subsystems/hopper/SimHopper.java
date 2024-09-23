package frc.robot.subsystems.hopper;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimHopper extends SubsystemBase implements IHopperSubsystem {
    @Override
    public Command feed() {
        return null;
    }

    @Override
    public Command outtake() {
        return null;
    }

    @Override
    public Trigger hasNote() {
        return null;
    }

    @Override
    public Command stop() {
        return null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hopper");
    }
}
