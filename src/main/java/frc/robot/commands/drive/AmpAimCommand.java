package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.util.DriverStationUtil;

public class AmpAimCommand extends Command {
  private final ISwerveDriveSubsystem swerveDriveSubsystem;

  public AmpAimCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.stop();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    Translation2d AmpRelativeBotPose = AimUtil.getAmpVector();
    double x = AmpRelativeBotPose.getX();
    double y = AmpRelativeBotPose.getY();
    return Math.abs(x) < SwerveInfo.AimTranslationDeadzone
        && Math.abs(y) < SwerveInfo.AimTranslationDeadzone;
  }
}
