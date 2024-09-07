package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoAmpCommand extends Command {
    private IIntakeSubsystem intake;

    public AutoAmpCommand(IIntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(IIntakeSubsystem.ExtensionState.AMP);
    }

    @Override
    public void execute() {
        intake.amp();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setExtended(IIntakeSubsystem.ExtensionState.RETRACTED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}