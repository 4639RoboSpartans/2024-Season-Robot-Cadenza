package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final int ModuleCount;

  private final CTRSwerveModule[] modules;
  private final Pigeon2 pigeon2;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final SwerveModulePosition[] modulePositions;
  private final Translation2d[] moduleLocations;
  private final OdometryThread odometryThread;
  private final Field2d field;
  private final PIDController turnPid;
  private final Notifier telemetry;
  private DriveState state;

  public enum DriveState{
    MANUAL,
    SPEAKER_LOCK
  }

  /* Put smartdashboard calls in separate thread to reduce performance impact */
  private void telemeterize() {
    SmartDashboard.putNumber("Successful Daqs", odometryThread.getSuccessfulDaqs());
    SmartDashboard.putNumber("Failed Daqs", odometryThread.getFailedDaqs());
    SmartDashboard.putNumber("X Pos", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y Pos", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Angle", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Odometry Loop Time", odometryThread.getTime());
  }

  /* Perform swerve module updates in a separate thread to minimize latency */
  private class OdometryThread extends Thread {
    private BaseStatusSignal[] m_allSignals;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;

    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;
    private double averageLoopTime = 0;

    public OdometryThread() {
      super();
      // 4 signals for each module + 2 for Pigeon2
      m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
      for (int i = 0; i < ModuleCount; ++i) {
        var signals = modules[i].getSignals();
        m_allSignals[(i * 4) + 0] = signals[0];
        m_allSignals[(i * 4) + 1] = signals[1];
        m_allSignals[(i * 4) + 2] = signals[2];
        m_allSignals[(i * 4) + 3] = signals[3];
      }
      m_allSignals[m_allSignals.length - 2] = pigeon2.getYaw();
      m_allSignals[m_allSignals.length - 1] = pigeon2.getAngularVelocityZDevice();
    }

    @Override
    public void run() {
      /* Make sure all signals update at around 250hz */
      for (var sig : m_allSignals) {
        sig.setUpdateFrequency(250);
      }
      /* Run as fast as possible, our signals will control the timing */
      while (true) {
        /* Synchronously wait for all signals in drivetrain */
        var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
        lastTime = currentTime;
        currentTime = Utils.getCurrentTimeSeconds();
        averageLoopTime = lowpass.calculate(currentTime - lastTime);

        /* Get status of the waitForAll */
        if (status.isOK()) {
          SuccessfulDaqs++;
        } else {
          FailedDaqs++;
        }

        /* Now update odometry */
        for (int i = 0; i < ModuleCount; ++i) {
          /* No need to refresh since it's automatically refreshed from the waitForAll() */
          modulePositions[i] = modules[i].getPosition(false);
        }
        // Assume Pigeon2 is flat-and-level so latency compensation can be performed
        double yawDegrees =
            BaseStatusSignal.getLatencyCompensatedValue(
                pigeon2.getYaw(), pigeon2.getAngularVelocityZDevice());

        odometry.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
        field.setRobotPose(odometry.getPoseMeters());
      }
    }

    public double getTime() {
      return averageLoopTime;
    }

    public int getSuccessfulDaqs() {
      return SuccessfulDaqs;
    }

    public int getFailedDaqs() {
      return FailedDaqs;
    }
  }

  public SwerveDriveSubsystem(
      SwerveDriveTrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    ModuleCount = modules.length;

    pigeon2 = new Pigeon2(driveTrainConstants.Pigeon2Id, driveTrainConstants.CANbusName);

    this.modules = new CTRSwerveModule[ModuleCount];
    modulePositions = new SwerveModulePosition[ModuleCount];
    moduleLocations = new Translation2d[ModuleCount];

    int iteration = 0;
    for (SwerveModuleConstants module : modules) {
      this.modules[iteration] = new CTRSwerveModule(module, driveTrainConstants.CANbusName);
      moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
      modulePositions[iteration] = this.modules[iteration].getPosition(true);

      iteration++;
    }
    kinematics = new SwerveDriveKinematics(moduleLocations);
    odometry = new SwerveDriveOdometry(kinematics, pigeon2.getRotation2d(), getSwervePositions());
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    turnPid = new PIDController(driveTrainConstants.TurnKp, 0, driveTrainConstants.TurnKd);
    turnPid.enableContinuousInput(-Math.PI, Math.PI);

    odometryThread = new OdometryThread();
    odometryThread.start();

    telemetry = new Notifier(this::telemeterize);
    telemetry.startPeriodic(0.1); // Telemeterize every 100ms

    state = DriveState.MANUAL;
  }

  private SwerveModulePosition[] getSwervePositions() {
    return modulePositions;
  }

  public void driveRobotCentric(ChassisSpeeds speeds) {
    var swerveStates = kinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i < ModuleCount; ++i) {
      modules[i].apply(swerveStates[i]);
    }
  }

  public void driveFieldCentric(ChassisSpeeds speeds) {
    var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pigeon2.getRotation2d());
    var swerveStates = kinematics.toSwerveModuleStates(roboCentric);
    for (int i = 0; i < ModuleCount; ++i) {
      modules[i].apply(swerveStates[i]);
    }
  }

  public void driveFullyFieldCentric(double xSpeeds, double ySpeeds, Rotation2d targetAngle) {
    var currentAngle = pigeon2.getRotation2d();
    double rotationalSpeed = turnPid.calculate(currentAngle.getRadians(), targetAngle.getRadians());

    var roboCentric =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeeds, ySpeeds, rotationalSpeed, pigeon2.getRotation2d());
    var swerveStates = kinematics.toSwerveModuleStates(roboCentric);
    for (int i = 0; i < ModuleCount; ++i) {
      modules[i].apply(swerveStates[i]);
    }
  }

  public void driveStopMotion() {
    /* Point every module toward (0,0) to make it close to a X configuration */
    for (int i = 0; i < ModuleCount; ++i) {
      var angle = moduleLocations[i].getAngle();
      modules[i].apply(new SwerveModuleState(0, angle));
    }
  }

  public void seedFieldRelative() {
    pigeon2.setYaw(0);
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public double getSuccessfulDaqs() {
    return odometryThread.SuccessfulDaqs;
  }

  public double getFailedDaqs() {
    return odometryThread.FailedDaqs;
  }

  @Override
  public void periodic() {
    switch (state) {
      case MANUAL:
        MovementUtil.reset();
      case SPEAKER_LOCK:
        if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          MovementUtil.setLocked(FieldConstants.speakerPose_blue);
        } else {
          MovementUtil.setLocked(FieldConstants.speakerPose_red);
        }
    }
  }

  public void setState(DriveState state){
    this.state = state;
  }
}
