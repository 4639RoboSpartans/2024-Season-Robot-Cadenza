// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotInfo.*;
import frc.robot.util.AimUtil;
import frc.robot.util.DriverStationUtil;
import frc.robot.util.LocalADStarAK;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(10.5);
  private static final double DRIVE_BASE_RADIUS = 0.245;
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  Field2d field = new Field2d();
  private Pose2d desiredPose;
  private final PIDController rotationController = SwerveInfo.TeleopRotationPID.create();

  private final SwerveDriveKinematics kinematics = SwerveInfo.SWERVE_DRIVE_KINEMATICS;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public SwerveDriveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::setMovement,
        new HolonomicPathFollowerConfig(
            SwerveInfo.TranslationPID,
            SwerveInfo.RotationPID,
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        DriverStationUtil::isRed,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
    sendSwerve();
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
    desiredPose = getPose();
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    SmartDashboard.putNumber("desired rotation", desiredPose.getRotation().getDegrees());
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    Pose2d limeLightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    Pose2d limeLight2Pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-slhs");
    addVisionMeasurement(limeLightPose);
    addVisionMeasurement(limeLight2Pose);

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);
    field.setRobotPose(getPose());

    Translation2d speakerPose = AimUtil.getSpeakerVector();
    double distance = Math.hypot(speakerPose.getX(), speakerPose.getY());
    SmartDashboard.putNumber("distance", distance);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void setMovement(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("gyro heading", gyroInputs.yawPosition.getDegrees());
    // Calculate module setpoints
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroInputs.yawPosition);
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    setMovement(new ChassisSpeeds());
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   */
  public void addVisionMeasurement(Pose2d visionPose) {
    Pose3d botPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    double dist =
        Math.sqrt(
            Math.pow(botPose.getX(), 2)
                + Math.pow(botPose.getY(), 2)
                + Math.pow(botPose.getZ(), 2));
    if (!(visionPose.getX() == 0) && !(visionPose.getY() == 0)) {
      poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(dist * VisionInfo.visionScalar, dist * VisionInfo.visionScalar, 0.1));
      poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
  }

  public Rotation2d getRotation2d() {
    return gyroInputs.yawPosition;
  }

  public void sendSwerve() {
    SmartDashboard.putData(
        "Swerve Drive",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> Math.toRadians(modules[0].getAngle().getDegrees()), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> Math.toRadians(modules[1].getAngle().getDegrees()), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> Math.toRadians(modules[2].getAngle().getDegrees()), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> Math.toRadians(modules[3].getAngle().getDegrees()), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

          builder.addDoubleProperty("Robot Angle", () -> gyroInputs.yawPosition.getDegrees(), null);
        });
  }

  @Override
  public void resetDesiredRotation() {
    setDesiredRotation(getRotation2d());
  }

  @Override
  public void setDesiredRotation(Rotation2d desiredRotation) {
    desiredPose = new Pose2d(getPose().getTranslation(), desiredRotation);
  }

  @Override
  public double getRawRotationSpeed() {
    return -rotationController.calculate(
            desiredPose.getRotation().minus(getRotation2d()).getDegrees())
        * SwerveInfo.TELEOP_AIM_SPEED;
  }

  @Override
  public double getAimRotationSpeed(double forwardsSpeed, double sidewaysSpeed) {
    Translation2d speakerTranslation = AimUtil.getSpeakerVector();
    Rotation2d rawRotation =
        Rotation2d.fromRadians(Math.atan(speakerTranslation.getY() / speakerTranslation.getX()))
            .minus(getRotation2d());
    double totalSpeed = Math.hypot(forwardsSpeed, sidewaysSpeed);
    double speakerSidewaysSpeed = totalSpeed * Math.sin(rawRotation.getRadians());
    return getRawRotationSpeed() * Math.abs(speakerSidewaysSpeed) * SwerveInfo.AimTranslationScalar;
  }

  @Override
  public double getRawXSpeed() {
    double x = desiredPose.getX();
    double forwardsInput;
    if (Math.abs(x) < SwerveInfo.AimTranslationDeadzone) {
      x = 0;
    }
    if (DriverStationUtil.isRed()) {
      forwardsInput = x * SwerveInfo.TeleopTranslationScalar;
    } else {
      forwardsInput = -x * SwerveInfo.TeleopTranslationScalar;
    }
    return forwardsInput;
  }

  @Override
  public double getRawYSpeed() {
    double y = desiredPose.getY();
    if (Math.abs(y) < SwerveInfo.AimTranslationDeadzone) {
      y = 0;
    }
    double sidewaysInput;
    if (DriverStationUtil.isRed()) {
      sidewaysInput = y * SwerveInfo.TeleopTranslationScalar;
    } else {
      sidewaysInput = -y * SwerveInfo.TeleopTranslationScalar;
    }
    return sidewaysInput;
  }

  @Override
  public void setDesiredPose(Pose2d pose) {
    desiredPose = pose;
  }

  @Override
  public void setDesiredTranslation(Translation2d translation) {
    desiredPose = new Pose2d(translation, getRotation2d());
  }

  @Override
  public Command ampAimCommand() {
    return new RunCommand(
        () -> {
          if (DriverStationUtil.isRed()) {
            setDesiredTranslation(FieldConstants.ampPose_red);
            setDesiredRotation(Rotation2d.fromDegrees(90));
          } else {
            setDesiredTranslation(FieldConstants.ampPose_blue);
            setDesiredRotation(Rotation2d.fromDegrees(-90));
          }
          setMovement(new ChassisSpeeds(getRawXSpeed(), getRawYSpeed(), getRawRotationSpeed()));
        },
        this);
  }

  @Override
  public Command speakerAimCommand(DoubleSupplier forwardsSpeeds, DoubleSupplier sidewaysSpeeds) {
    return new RunCommand(
        () -> {
          setDesiredRotation(
              AimUtil.getSpeakerRotation(
                  forwardsSpeeds.getAsDouble(), sidewaysSpeeds.getAsDouble()));
          setMovement(
              new ChassisSpeeds(
                  forwardsSpeeds.getAsDouble(),
                  sidewaysSpeeds.getAsDouble(),
                  getAimRotationSpeed(forwardsSpeeds.getAsDouble(), sidewaysSpeeds.getAsDouble())));
        },
        this);
  }
}
