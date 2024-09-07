package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IIntakeSubsystem;

public class ManualIntakeExtendCommand extends Command{
    private final IIntakeSubsystem intake;

    public ManualIntakeExtendCommand(IIntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }

    @Override
    public void execute() {
        intake.manualExtend();
    }

    @Override
    public void end(boolean interrupted) {
        intake.updateOffset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}