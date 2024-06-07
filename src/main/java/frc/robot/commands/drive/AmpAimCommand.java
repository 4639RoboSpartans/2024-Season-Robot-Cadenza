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
    Translation2d AmpRelativeBotPose = AimUtil.getAmpVector();
    double x = AmpRelativeBotPose.getX();
    double y = AmpRelativeBotPose.getY();
    double forwardsInput, sidewaysInput;
    if (Math.abs(x) < SwerveInfo.AimTranslationDeadzone) {
      x = 0;
    }
    if (Math.abs(y) < SwerveInfo.AimTranslationDeadzone) {
      y = 0;
    }
    SmartDashboard.putNumber("amp x", x);
    SmartDashboard.putNumber("amp y", y);
    if (DriverStationUtil.isRed()) {
      swerveDriveSubsystem.setDesiredRotation(Rotation2d.fromDegrees(90));
      forwardsInput = x * SwerveInfo.TeleopTranslationScalar;
      sidewaysInput = y * SwerveInfo.TeleopTranslationScalar;
    } else {
      swerveDriveSubsystem.setDesiredRotation(Rotation2d.fromDegrees(270));
      forwardsInput = -x * SwerveInfo.TeleopTranslationScalar;
      sidewaysInput = -y * SwerveInfo.TeleopTranslationScalar;
    }
    ChassisSpeeds toAmp =
        new ChassisSpeeds(forwardsInput, sidewaysInput, swerveDriveSubsystem.getRawRotationSpeed());
    swerveDriveSubsystem.setMovement(toAmp);
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
