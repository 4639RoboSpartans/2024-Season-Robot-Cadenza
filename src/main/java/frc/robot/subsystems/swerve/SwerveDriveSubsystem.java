package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {

  private final SwerveModule moduleFrontLeft, moduleFrontRight, moduleBackLeft, moduleBackRight;

  private final NavX navx;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private ChassisSpeeds chassisSpeeds;

  private final Field2d m_field = new Field2d();

  public SwerveDriveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    navx = SubsystemManager.getNavX();

    moduleFrontLeft = new SwerveModule(IDs.MODULE_FRONT_LEFT);
    moduleFrontRight = new SwerveModule(IDs.MODULE_FRONT_RIGHT);
    moduleBackLeft = new SwerveModule(IDs.MODULE_BACK_LEFT);
    moduleBackRight = new SwerveModule(IDs.MODULE_BACK_RIGHT);
    navx.reset();
    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveInfo.SWERVE_DRIVE_KINEMATICS,
            navx.getRotation2d(),
            getStates(),
            LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
    this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    setBrakeMode();

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::setRawMovement,
        new HolonomicPathFollowerConfig(
            SwerveInfo.TranslationPID, SwerveInfo.RotationPID, 4, 0.4, new ReplanningConfig()),
        () -> RobotContainer.alliance.getSelected(),
        this);

    sendSwerve();
  }

  public void useTeleopCurrentLimits() {
    moduleFrontLeft.useTeleopCurrentLimits();
    moduleFrontRight.useTeleopCurrentLimits();
    moduleBackLeft.useTeleopCurrentLimits();
    moduleBackRight.useTeleopCurrentLimits();
  }

  public void useAutonCurrentLimits() {
    moduleFrontLeft.useAutonCurrentLimits();
    moduleFrontRight.useAutonCurrentLimits();
    moduleBackLeft.useAutonCurrentLimits();
    moduleBackRight.useAutonCurrentLimits();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return chassisSpeeds;
  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public void setMovement(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds robotCentricSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navx.getRotation2d());
    setRawMovement(robotCentricSpeeds);
    chassisSpeeds = robotCentricSpeeds;
  }

  public void setRawMovement(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates =
        SwerveInfo.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModulesStates(
        swerveModuleStates[0], swerveModuleStates[1], swerveModuleStates[2], swerveModuleStates[3]);
    this.chassisSpeeds = chassisSpeeds;
  }

  private void setModulesStates(
      SwerveModuleState stateFrontLeft,
      SwerveModuleState stateFrontRight,
      SwerveModuleState stateBackLeft,
      SwerveModuleState stateBackRight) {
    moduleFrontLeft.setState(stateFrontLeft);
    moduleFrontRight.setState(stateFrontRight);
    moduleBackLeft.setState(stateBackLeft);
    moduleBackRight.setState(stateBackRight);
  }

  public void stop() {
    moduleBackLeft.stop();
    moduleBackRight.stop();
    moduleFrontLeft.stop();
    moduleFrontLeft.stop();
  }

  @Override
  public void periodic() {
    moduleFrontLeft.periodic();
    moduleFrontRight.periodic();
    moduleBackLeft.periodic();
    moduleBackRight.periodic();
    updateOdometry();
    m_field.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumberArray("FL", getStatesAsArray());
  }

  public void updateOdometry() {
    m_poseEstimator.update(navx.getRotation2d(), getStates());
    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(
        "limelight",
        m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    Pose2d curr_pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    if (curr_pose.getTranslation().equals(new Translation2d(0, 0))) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.4, 0.4, 9999999));
      m_poseEstimator.addVisionMeasurement(curr_pose, Timer.getFPGATimestamp());
    }
  }

  public void sendSwerve() {
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle",
                () -> Math.toRadians(moduleFrontLeft.getRotationInDegrees()),
                null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> moduleFrontLeft.getDriveVelocity(), null);

            builder.addDoubleProperty(
                "Front Right Angle",
                () -> Math.toRadians(moduleFrontRight.getRotationInDegrees()),
                null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> moduleFrontRight.getDriveVelocity(), null);

            builder.addDoubleProperty(
                "Back Left Angle",
                () -> Math.toRadians(moduleBackLeft.getRotationInDegrees()),
                null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> moduleBackLeft.getDriveVelocity(), null);

            builder.addDoubleProperty(
                "Back Right Angle",
                () -> Math.toRadians(moduleBackRight.getRotationInDegrees()),
                null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> moduleBackRight.getDriveVelocity(), null);

            builder.addDoubleProperty("Robot Angle", () -> navx.getRotation2d().getRadians(), null);
          }
        });
  }

  public void setBrakeMode() {
    moduleFrontLeft.setBrakeMode();
    moduleFrontRight.setBrakeMode();
    moduleBackLeft.setBrakeMode();
    moduleBackRight.setBrakeMode();
  }

  public void setCoastMode() {
    moduleFrontLeft.setCoastMode();
    moduleFrontRight.setCoastMode();
    moduleBackLeft.setCoastMode();
    moduleBackRight.setCoastMode();
  }

  public SwerveModule getSwerveModule(String module) {
    return switch (module) {
      case "FL" -> moduleFrontLeft;
      case "BL" -> moduleBackLeft;
      case "FR" -> moduleFrontRight;
      case "BR" -> moduleBackRight;
      default -> null;
    };
  }

  public double getHeading() {
    return navx.getHeading();
  }

  public SwerveModulePosition[] getStates() {
    return new SwerveModulePosition[] {
      moduleFrontLeft.getPosition(),
      moduleFrontRight.getPosition(),
      moduleBackLeft.getPosition(),
      moduleBackRight.getPosition()
    };
  }

  public double[] getStatesAsArray() {
    double[] res = new double[8];
    res[0] = moduleFrontLeft.getState().angle.getDegrees();
    res[1] = moduleFrontLeft.getStateAsArray()[0];
    res[2] = moduleFrontRight.getState().angle.getDegrees();
    res[3] = moduleFrontRight.getStateAsArray()[0];
    res[4] = moduleBackLeft.getState().angle.getDegrees();
    res[5] = moduleBackLeft.getStateAsArray()[0];
    res[6] = moduleBackRight.getState().angle.getDegrees();
    res[7] = moduleBackRight.getStateAsArray()[0];
    return res;
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(new Rotation2d(getHeading()), getStates(), pose);
  }
}
