package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem.ExtensionState;

public abstract class IntakeExtensionsCommand extends Command {
    private final IIntakeSubsystem intake;
    private final ExtensionState extended;

    public IntakeExtensionsCommand(IIntakeSubsystem intake, ExtensionState extended) {
        this.intake = intake;
        this.extended = extended;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(extended);
    }
}
