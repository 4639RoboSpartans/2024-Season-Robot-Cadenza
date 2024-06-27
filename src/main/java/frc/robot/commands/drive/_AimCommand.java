package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

class _AimCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;

    public _AimCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        AimSubsystem.lockToSpeaker();
        swerveDriveSubsystem.setMovement(AimSubsystem.getRobotRelative(swerveDriveSubsystem.getPose()));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return AimSubsystem.atRotation(swerveDriveSubsystem.getPose());
    }
}
