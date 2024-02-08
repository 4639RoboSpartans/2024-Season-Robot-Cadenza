package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IIntakeSubsystem;

public class OuttakeCommand extends Command {
    private final IIntakeSubsystem intake;

    public OuttakeCommand(IIntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.outtake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
