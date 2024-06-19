package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.util.MovementUtil;

class _AimCommand extends Command {
  private final ISwerveDriveSubsystem swerveDriveSubsystem;

  public _AimCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.stop();
    MovementUtil.reset();
    MovementUtil.lockToSpeaker();
  }

  @Override
  public void execute() {
    Pose2d pose = swerveDriveSubsystem.getPose();
    swerveDriveSubsystem.setMovement(MovementUtil.getRobotRelative(pose));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }
}
