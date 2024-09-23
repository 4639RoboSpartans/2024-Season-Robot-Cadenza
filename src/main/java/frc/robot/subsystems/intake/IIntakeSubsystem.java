package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIntakeSubsystem extends Subsystem, Sendable {
    Command intake();

    Command outtake();

    Command pivotExtend();

    Command pivotRetract();

    Command pivotOuttake();

    Command ampPrep();

    Command pivotIntake();

    Command pivotAmp();

    Command stopIntake();

    Command stop();
}
