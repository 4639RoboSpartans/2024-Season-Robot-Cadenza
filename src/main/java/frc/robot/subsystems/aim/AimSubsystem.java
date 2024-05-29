package frc.robot.subsystems.aim;

import static frc.robot.constants.RobotInfo.AimInfo;
import static frc.robot.constants.RobotInfo.ShooterInfo.ShooterSetpoint;
import static frc.robot.constants.RobotInfo.SwerveInfo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import math.Averager;
import math.MathUtil;

public class AimSubsystem extends SubsystemBase implements AimInterface {

  private final PIDController rotationPID = AimInfo.LIMELIGHT_AIM_PID_CONSTANTS.create();

  private final Averager angle = new Averager(Constants.POSE_WINDOW_LENGTH);
  private final Averager x = new Averager(Constants.POSE_WINDOW_LENGTH);
  private final Averager z = new Averager(Constants.POSE_WINDOW_LENGTH);
  private final ISwerveDriveSubsystem swerveDriveSubsystem;

  public AimSubsystem(ISwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  @Override
  public double getSwerveRotation() {
    if (!angle.hasMeasurements()) return 0;

    double yRotation = getYDegrees();
    yRotation *= (1 + getTxDerivative() * SwerveInfo.DERIVATIVE_MULTIPLIER);

    return rotationPID.calculate(MathUtil.signedPow(yRotation, AimInfo.AIM_ROT_POW))
        * SwerveInfo.AIM_ROTATION_SPEED;
  }

  public double getYDegrees() {
    Translation2d speakerVector = getVector();
    double yRotation = Math.toDegrees(Math.tan(speakerVector.getY() / speakerVector.getX()));
    return yRotation - swerveDriveSubsystem.getRotation2d().getDegrees();
  }

  public Translation2d getVector() {
    Pose2d currBotPose = swerveDriveSubsystem.getPose();
    Translation2d currBotTranslation = currBotPose.getTranslation();
    Translation2d speakerPose;
    if (SmartDashboard.getBoolean("Alliance", false)) {
      speakerPose = FieldConstants.speakerPose_red;
    } else {
      speakerPose = FieldConstants.speakerPose_blue;
    }
    return currBotTranslation.minus(speakerPose);
  }

  @Override
  public double getTxDerivative() {
    return x.getDerivative();
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(angle.getValue()) <= AimInfo.AIM_TOLERANCE && LimeLight.seesAprilTags();
  }

  @Override
  public ShooterSetpoint getShooterSetpoint() {
    ShooterSetpoint result = ShooterMeasurementLERPer.get(x.getValue(), z.getValue());

    SmartDashboard.putNumber("Shooter Angle: ", result.angle());
    SmartDashboard.putNumber("Shooter Speed: ", result.speed());

    return result;
  }

  @Override
  public void periodic() {
    Translation2d speakerVector = getVector();
    double x = speakerVector.getY();
    double z = speakerVector.getX();
    double angle = getYDegrees();

    double distance = Math.hypot(x, z);
    SmartDashboard.putNumber("distance ", distance);
    SmartDashboard.putNumber("angle", this.angle.getValue());
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("z", z);

    this.angle.addMeasurement(angle);

    this.x.addMeasurement(x);
    this.z.addMeasurement(z);
  }

  @Override
  public void resetPID() {
    rotationPID.reset();
  }
}
