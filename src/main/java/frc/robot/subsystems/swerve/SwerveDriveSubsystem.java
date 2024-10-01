package frc.robot.subsystems.swerve;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public abstract class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem, Sendable {
    public static SwerveDriveSubsystem getInstance() {
        return TunerConstants.DriveTrain;
    }

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    enum DriveState {
        TELEOP,
        TRACK_TARGET,
        SOTF,
        PATHFIND
    }

    public abstract Rotation2d getRotation2d();

    public abstract void stop();

    public abstract void reset();

    public abstract Pose2d getPose();

    public abstract Command pathfindCommand(Pose2d targetPose);

    public abstract Command driveFieldCentricCommand();

    public abstract Command SOTFCommand();

    public abstract Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition);

    public abstract Command followChoreoPath(String pathName, boolean resetPosition);

    public Trigger isAligned() {
        return new Trigger(this::isAlignedSupplier);
    }

    public Trigger inShootingRange() {
        return new Trigger(this::inShootingRangeSupplier);
    }

    public Trigger inShootingSector() {
        return new Trigger(this::inShootingSectorSupplier);
    }

    protected abstract boolean isAlignedSupplier();

    protected abstract boolean inShootingRangeSupplier();

    protected abstract boolean inShootingSectorSupplier();

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
    }

    protected abstract void buildSendable(SendableBuilder builder);
}