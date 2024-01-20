package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.tramp.ITrampSubsystem;

public class ReleaseTrampCommand extends Command {
    private final ITrampSubsystem iTrampSubsystem;
    private final double startTime;

    public ReleaseTrampCommand(ITrampSubsystem iTrampSubsystem) {
        this.iTrampSubsystem = iTrampSubsystem;
        startTime = Timer.getFPGATimestamp();
    }

    public void initialize() {
        iTrampSubsystem.stop();
    }

    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime >= Constants.RobotInfo.TRAMP_RELEASE_TIME) {
            iTrampSubsystem.setHookAngleDegrees(Constants.RobotInfo.TRAMP_HOOK_RELEASE_DEGREES);
        } else if (currentTime - startTime >= Constants.RobotInfo.TRAMP_EXTEND_TIME) {
            iTrampSubsystem.setAngleDegrees(Constants.RobotInfo.TRAMP_FRONT_ROTATOR_DEGREES);
        } else {
            iTrampSubsystem.setAngleDegrees(Constants.RobotInfo.TRAMP_BACK_ROTATOR_DEGREES);
        }
    }

    public void end(boolean interrupted) {
        iTrampSubsystem.stop();
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= Constants.RobotInfo.TRAMP_FINAL_TIME;
    }
}
