package frc.robot.subsystems.swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.Controls;
import frc.robot.subsystems.swerve.constants.TunerConstants;
import frc.robot.util.AimUtil;
import frc.robot.util.CommandsUtil;
import frc.robot.util.DriverStationUtil;
import frc.robot.util.LimelightHelpers;

import static frc.robot.subsystems.swerve.constants.SwerveConstants.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem, so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements ISwerveDriveSubsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.FieldCentricFacingAngle SOTFRequest = new SwerveRequest.FieldCentricFacingAngle();

    private final Field2d field = new Field2d();
    private DriveState state;

    private Trigger inShootingRange, inShootingSector, isAligned, inSpinupRange, inLaunchRange;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        seedFieldRelative(new Pose2d());
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        state = DriveState.TELEOP;
        SOTFRequest.HeadingController = new PhoenixPIDController(8, 0, 0);
        SOTFRequest.HeadingController.setTolerance(Rotation2d.fromDegrees(7.5).getRadians());
        inShootingRange = new Trigger(this::inShootingRangeSupplier);
        inShootingSector = new Trigger(this::inShootingSectorSupplier);
        isAligned = new Trigger(this::isAlignedSupplier);
        inSpinupRange = new Trigger(this::inSpinupRangeSupplier);
        inLaunchRange = new Trigger(this::inLaunchRangeSupplier);
    }

    private SwerveRequest fieldCentricRequestSupplier() {
        double forwardsSpeed = Controls.DriverControls.SwerveForwardAxis.getAsDouble();
        double sidewaysSpeed = Controls.DriverControls.SwerveStrafeAxis.getAsDouble();
        double rotationSpeed = Controls.DriverControls.SwerveRotationAxis.getAsDouble();
        return fieldCentricRequest
                .withVelocityX(forwardsSpeed)
                .withVelocityY(sidewaysSpeed)
                .withRotationalRate(rotationSpeed);
    }

    private SwerveRequest SOTFRequestSupplier() {
        return SOTFRequest
                .withSteerRequestType(
                        SwerveModule.SteerRequestType.MotionMagic
                )
                .withTargetDirection(
                        AimUtil.getSpeakerRotation(
                                Controls.DriverControls.SwerveStrafeAxis.getAsDouble()
                        )
                )
                .withVelocityX(
                        Controls.DriverControls.SwerveForwardAxis.getAsDouble() / 2
                ).withVelocityY(
                        Controls.DriverControls.SwerveStrafeAxis.getAsDouble() / 2
                );
    }

    public Command pathfindCommand(Pose2d targetPose) {
        state = DriveState.PATHFIND;
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        PathConstraints constraints = new PathConstraints(
                6, 3,
                2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
                targetPose,
                constraints
        );
    }

    public Command driveFieldCentricCommand() {
        state = DriveState.TELEOP;
        return applyRequest(this::fieldCentricRequestSupplier);
    }

    public Command SOTFCommand() {
        state = DriveState.SOTF;
        return applyRequest(this::SOTFRequestSupplier);
    }

    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
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
    public void periodic() {
        field.setRobotPose(getPose());
        SmartDashboard.putData("Drive/Field", field);
        LimelightHelpers.PoseEstimate pose = validatePoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight"), Timer.getFPGATimestamp());
        if (pose != null) {
            addVisionMeasurement(pose.pose, Timer.getFPGATimestamp());
        }
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.020, RobotController.getBatteryVoltage());
        field.setRobotPose(getPose());
        SmartDashboard.putData("field", field);
    }

    public void reset() {
        m_pigeon2.reset();
    }

    private LimelightHelpers.PoseEstimate validatePoseEstimate(LimelightHelpers.PoseEstimate poseEstimate, double deltaSeconds) {
        if (poseEstimate == null) return null;
        Pose2d pose2d = poseEstimate.pose;
        Translation2d trans = pose2d.getTranslation();
        if (trans.getX() == 0 && trans.getY() == 0) {
            return null;
        }
        Translation2d botTrans = getPose().getTranslation();
        Translation2d botDistTrans = botTrans.minus(trans);
        double dist = Math.hypot(botDistTrans.getX(), botDistTrans.getY());
        if (dist > 1) {
            if (botTrans.getX() == 0 && botTrans.getY() == 0) {
                return poseEstimate;
            }
            return null;
        }
        return poseEstimate;
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public Rotation2d getRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    @Override
    public void stop() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param pathName      The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition);
    }

    @Override
    public Trigger isAligned() {
        return isAligned;
    }

    @Override
    public Trigger inShootingRange() {
        return inShootingRange;
    }

    @Override
    public Trigger inShootingSector() {
        return inShootingSector;
    }

    @Override
    public Trigger inSpinupRange() {
        return inSpinupRange;
    }

    @Override
    public Trigger inLaunchRange() {
        return inLaunchRange;
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param trajectory    The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> {
                seedFieldRelative(DriverStationUtil.isRed() ? trajectory.getFlippedInitialPose() : trajectory.getInitialPose());
            }));
        }
        commands.add(choreoSwerveCommand(trajectory));
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
                (ChassisSpeeds speeds) -> setControl(
                        autoRequest.withSpeeds(speeds)
                ),
                DriverStationUtil::isRed,
                this
        );
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                DriverStationUtil::isRed,
                this);
    }

    private boolean inShootingRangeSupplier() {
        return AimUtil.getSpeakerDist() <= 3.5;
    }

    private boolean inShootingSectorSupplier() {
        Rotation2d rotation = AimUtil.getSpeakerRotation();
        return Math.abs(rotation.getDegrees()) <= 35;
    }

    private boolean isAlignedSupplier() {
        if (!Robot.isInAuton()) {
            return SOTFRequest.HeadingController.atSetpoint();
        } else {
            return Math.abs(AimUtil.getSpeakerOffset().getDegrees()) <= 5;
        }
    }

    private boolean inSpinupRangeSupplier() {
        return AimUtil.getSpeakerDist() <= 4;
    }

    private boolean inLaunchRangeSupplier() {
        return AimUtil.getSpeakerVector().getX() <= 10.6934;
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drive");
        builder.addDoubleProperty("Left Y",
                Controls.DriverControls.SwerveForwardAxis,
                (DoubleConsumer) null);
        builder.addDoubleProperty("Left X",
                Controls.DriverControls.SwerveStrafeAxis,
                (DoubleConsumer) null);
        builder.addDoubleProperty("Right X",
                Controls.DriverControls.SwerveRotationAxis,
                (DoubleConsumer) null);
        builder.addDoubleProperty("Heading",
                () -> getRotation2d().getDegrees(),
                (DoubleConsumer) null);
    }

    public static CommandSwerveDrivetrain getInstance() {
        return TunerConstants.DriveTrain;
    }
}