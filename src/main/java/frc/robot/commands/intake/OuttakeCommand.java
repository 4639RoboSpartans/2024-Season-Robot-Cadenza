package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo.HopperInfo;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class OuttakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final HopperSubsystem hopper;

    public OuttakeCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
        this.intake = intake;
        this.hopper = hopper;

        addRequirements(intake, hopper);
    }

    @Override
    public void execute() {
        intake.outtake();
        hopper.runBackwards(HopperInfo.HOPPER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        hopper.stop();
    }
}
