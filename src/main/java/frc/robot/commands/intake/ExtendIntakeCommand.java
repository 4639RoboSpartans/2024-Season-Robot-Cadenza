package frc.robot.commands.intake;

import frc.robot.subsystems.intake.IntakeSubsystem;

public class ExtendIntakeCommand extends IntakeExtensionsCommand {
    public ExtendIntakeCommand(IntakeSubsystem intake) {
        super(intake, IntakeSubsystem.ExtensionState.EXTENDED);
    }
}
