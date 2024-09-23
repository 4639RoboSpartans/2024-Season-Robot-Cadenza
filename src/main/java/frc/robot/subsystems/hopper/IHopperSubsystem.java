package frc.robot.subsystems.hopper;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IHopperSubsystem extends Subsystem, Sendable {
    Command feed();

    Command outtake();

    Trigger hasNote();

    Command stop();
}
