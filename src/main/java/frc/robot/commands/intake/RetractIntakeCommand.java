package frc.robot.commands.intake;

import frc.robot.subsystems.intake.IIntakeSubsystem;

public class RetractIntakeCommand extends IntakeExtensionsCommand {
    public RetractIntakeCommand(IIntakeSubsystem intake) {
        super(intake, IIntakeSubsystem.ExtensionState.RETRACTED);
    }
}
