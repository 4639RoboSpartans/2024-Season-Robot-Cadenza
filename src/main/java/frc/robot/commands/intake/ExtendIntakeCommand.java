package frc.robot.commands.intake;

import frc.robot.subsystems.intake.IIntakeSubsystem;

public class ExtendIntakeCommand extends IntakeExtensionsCommand {
  public ExtendIntakeCommand(IIntakeSubsystem intake) {
    super(intake, IIntakeSubsystem.ExtensionState.EXTENDED);
  }
}
