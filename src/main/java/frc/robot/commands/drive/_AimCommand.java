package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

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
    swerveDriveSubsystem.setDesiredRotation(AimUtil.getRotation());
    double rotateSpeed =
        swerveDriveSubsystem.getRawRotationSpeed() * RobotInfo.SwerveInfo.TELEOP_AIM_SPEED;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotateSpeed);
    swerveDriveSubsystem.setMovement(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }
}
