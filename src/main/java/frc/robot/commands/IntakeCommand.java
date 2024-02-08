package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IIntakeSubsystem;

public class IntakeCommand extends Command {
    private final IIntakeSubsystem intake;

    public IntakeCommand(IIntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
