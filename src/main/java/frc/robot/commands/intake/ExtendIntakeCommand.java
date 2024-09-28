package frc.robot.commands.intake;

import frc.robot.subsystems.intake.ConcreteIntakeSubsystem;

public class ExtendIntakeCommand extends IntakeExtensionsCommand {
    public ExtendIntakeCommand(ConcreteIntakeSubsystem intake) {
        super(intake, ConcreteIntakeSubsystem.ExtensionState.EXTENDED);
    }
}
