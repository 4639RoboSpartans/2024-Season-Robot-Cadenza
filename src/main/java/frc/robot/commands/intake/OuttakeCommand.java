package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;

public class OuttakeCommand extends Command {
    private final IIntakeSubsystem intake;
    private final IHopperSubsystem hopperSubsystem;

    public OuttakeCommand(IIntakeSubsystem intake, IHopperSubsystem hopper) {
        this.intake = intake;
        this.hopperSubsystem = hopper;

        addRequirements(intake, hopper);
    }

    @Override
    public void execute() {
        intake.outtake();
        hopperSubsystem.runBackwards();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        hopperSubsystem.stop();
    }
}
