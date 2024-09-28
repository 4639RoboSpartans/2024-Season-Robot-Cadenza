package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoAmpCommand extends Command {
    private IntakeSubsystem intake;

    public AutoAmpCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(IntakeSubsystem.ExtensionState.AMP);
    }

    @Override
    public void execute() {
        intake.amp();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setExtended(IntakeSubsystem.ExtensionState.RETRACTED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}