package frc.robot.subsystems.swerve;

import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.util.DriverStationUtil;

public interface ISwerveDriveSubsystem extends Subsystem {

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
}