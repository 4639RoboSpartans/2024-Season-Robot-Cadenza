package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.network.LimelightHelpers;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.network.LimelightHelpers.PoseEstimate;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, ISwerveDriveSubsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.ApplyChassisSpeeds TeleopRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final Field2d field = new Field2d();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative,  // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public Rotation2d getRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    @Override
    public void setFieldCentricMovement(ChassisSpeeds chassisSpeeds) {
        setControl(TeleopRequest.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d())));
    }

    @Override
    public void setRawMovement(ChassisSpeeds chassisSpeeds) {
        setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(chassisSpeeds));
    }

    @Override
    public void stop() {
        this.setFieldCentricMovement(new ChassisSpeeds());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter angle", SubsystemManager.getShooterPivot(SubsystemManager.getShooter()).getCurrentAngle());
        field.setRobotPose(this.getState().Pose);
        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("heading", getRotation2d().getDegrees());
        PoseEstimate pose = validatePoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight"), Timer.getFPGATimestamp());
        if (pose != null) {
            addVisionMeasurement(pose.pose, Timer.getFPGATimestamp());
        }
    }

    public void reset() {
        m_pigeon2.reset();
    }

    public PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate, double deltaSeconds) {
        if (poseEstimate == null) return null;
        Pose2d pose2d = poseEstimate.pose;
        Translation2d trans = pose2d.getTranslation();
        if (trans.getX() == 0 && trans.getY() == 0) {
            return null;
        }
        return poseEstimate;
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }
    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     * @param pathName The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(pathName, resetPosition, null);
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     * @param pathName The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition, Function<Double, Double> rotationOverride) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition, rotationOverride);
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     * @param trajectory The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition, Function<Double, Double> rotationOverride) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> {
                seedFieldRelative(Robot.isRed() ? trajectory.getFlippedInitialPose() : trajectory.getInitialPose());
            }));
        }
        commands.add(rotationOverride != null ? choreoRotationCommand(trajectory, rotationOverride) : choreoSwerveCommand(trajectory));
        return CommandsUtil.sequence(commands);
    }

    // This is a helper method that creates a command that makes the robot follow a Choreo path
    private Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
                trajectory,
                this::getPose,
                choreoX,
                choreoY,
                choreoRotation,
                this::drive,
                DriverStationUtil::isRed,
                this
        );
    }
}