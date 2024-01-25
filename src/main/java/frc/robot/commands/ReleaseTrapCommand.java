package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.tramp.ITrapSubsystem;

public class ReleaseTrapCommand extends Command {
    private final ITrapSubsystem iTrapSubsystem;
    private final double startTime;

    public ReleaseTrapCommand(ITrapSubsystem iTrapSubsystem) {
        this.iTrapSubsystem = iTrapSubsystem;
        startTime = Timer.getFPGATimestamp();
    }

    public void initialize() {
        iTrapSubsystem.stop();
    }

    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime >= Constants.RobotInfo.TRAP_RELEASE_TIME) {
            iTrapSubsystem.intake();
        } else if (currentTime - startTime >= Constants.RobotInfo.TRAP_EXTEND_TIME) {
            iTrapSubsystem.setAngleDegrees(Constants.RobotInfo.TRAP_FRONT_ROTATOR_DEGREES);
        } else {
            iTrapSubsystem.setAngleDegrees(Constants.RobotInfo.TRAP_BACK_ROTATOR_DEGREES);
        }
    }

    public void end(boolean interrupted) {
        iTrapSubsystem.stop();
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= Constants.RobotInfo.TRAP_FINAL_TIME;
    }
}
