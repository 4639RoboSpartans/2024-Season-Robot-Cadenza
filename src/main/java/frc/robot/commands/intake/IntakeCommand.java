package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;

public class IntakeCommand extends Command {
    private final IIntakeSubsystem intake;
    private final IHopperSubsystem hopper;
    private boolean shouldEnd = false;

    public IntakeCommand(IIntakeSubsystem intake, IHopperSubsystem hopper) {
        this.intake = intake;
        this.hopper = hopper;

        addRequirements(intake, hopper);
    }

    @Override
    public void initialize() {
        shouldEnd = false;
    }

    @Override
    public void execute() {
        intake.intake();
        hopper.run(true);
        if(hopper.getIR().hasNote()) shouldEnd = true;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        hopper.stop();

        if(!interrupted && shouldEnd) {
            intake.setExtended(false);
        }
        shouldEnd = false;
    }

    @Override
    public boolean isFinished(){
        return shouldEnd;
    }
}
