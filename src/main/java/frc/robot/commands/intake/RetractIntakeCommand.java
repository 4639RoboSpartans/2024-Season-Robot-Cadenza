package frc.robot.commands.intake;

import frc.robot.subsystems.intake.IntakeSubsystem;

public class RetractIntakeCommand extends IntakeExtensionsCommand {
    public RetractIntakeCommand(IntakeSubsystem intake) {
        super(intake, IntakeSubsystem.ExtensionState.RETRACTED);
    }
}
