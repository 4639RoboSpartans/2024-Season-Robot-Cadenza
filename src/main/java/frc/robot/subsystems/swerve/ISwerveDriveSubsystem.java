package frc.robot.subsystems.swerve;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerveDriveSubsystem extends Subsystem {
    enum DriveState {
        TELEOP,
        TRACK_TARGET,
        SOTF,
        PATHFIND
    }

    Rotation2d getRotation2d();
    void stop();

    void reset();

    Pose2d getPose();

    Command pathfindCommand(Pose2d targetPose);

    Command driveFieldCentricCommand();

    Command trackTargetCommand(Pose2d targetPose);

    Command SOTFCommand();

    Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition);


    Command followChoreoPath(String pathName, boolean resetPosition);

    boolean isAligned();

    boolean inShootingRange();

    boolean inShootingSector();

    boolean inSpinupRange();
}