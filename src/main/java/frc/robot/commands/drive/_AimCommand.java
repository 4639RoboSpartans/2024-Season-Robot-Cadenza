package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo.AimInfo;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

class _AimCommand extends Command {
  private final ISwerveDriveSubsystem swerveDriveSubsystem;
  private final PIDController aimPID;
  private final double timeSeconds;
  private final double startTime;

  public _AimCommand(ISwerveDriveSubsystem swerveDriveSubsystem, double timeSeconds) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.timeSeconds = timeSeconds;
    startTime = Timer.getFPGATimestamp();
    addRequirements(swerveDriveSubsystem);
    aimPID = AimInfo.aimPID.create();
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.stop();
  }

  @Override
  public void execute() {
    double rotateSpeed = getRotationSpeed();

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotateSpeed);
    swerveDriveSubsystem.setMovement(chassisSpeeds);
    SmartDashboard.putNumber("navX heading", SubsystemManager.getNavX().getHeading());
  }

  private double getRotationSpeed() {
    double rawSpeed;
    Translation2d SpeakerRelativeBotPose = AimUtil.getSpeakerToRobotVector();
    Rotation2d botAngle =
        new Rotation2d(Math.atan(SpeakerRelativeBotPose.getY() / SpeakerRelativeBotPose.getX()));
    swerveDriveSubsystem.setDesiredRotation(botAngle);
    Rotation2d currAngle = swerveDriveSubsystem.getRotation2d();
    double rotationOffset = currAngle.minus(botAngle).getRadians();
    double givenOffset =
        Math.copySign(Math.pow(rotationOffset, 1 + SwerveInfo.rotationScalar), rotationOffset);
    rawSpeed = aimPID.calculate(givenOffset) * SwerveInfo.ROTATION_MULTIPLIER;
    return rawSpeed;
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= timeSeconds;
  }
}
