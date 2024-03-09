package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.NavX;

public class OdometrySubsystem extends SubsystemBase{
    private final ISwerveDriveSubsystem driveSubsystem;
    private final NavX navx;
    private final SwerveDriveOdometry odometry;

    public OdometrySubsystem(ISwerveDriveSubsystem driveSubsystem, NavX navx){
        this.driveSubsystem = driveSubsystem;
        this.navx = navx;
        this.odometry = new SwerveDriveOdometry(
            Constants.RobotInfo.SwerveInfo.SWERVE_DRIVE_KINEMATICS,
            navx.getRotation2d(), 
            new SwerveModulePosition[] {
            driveSubsystem.getSwerveModule("FL").getPosition(),
            driveSubsystem.getSwerveModule("FR").getPosition(),
            driveSubsystem.getSwerveModule("BL").getPosition(),
            driveSubsystem.getSwerveModule("BR").getPosition()
        });
    }

    @Override
    public void periodic(){
        odometry.update(navx.getRotation2d(), new SwerveModulePosition[] {
            driveSubsystem.getSwerveModule("FL").getPosition(),
            driveSubsystem.getSwerveModule("FR").getPosition(),
            driveSubsystem.getSwerveModule("BL").getPosition(),
            driveSubsystem.getSwerveModule("BR").getPosition()
        });
    }

    public SwerveDriveOdometry getOdometry(){
        return this.odometry;
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(navx.getRotation2d(), 
            new SwerveModulePosition[] {
                driveSubsystem.getSwerveModule("FL").getPosition(),
                driveSubsystem.getSwerveModule("FR").getPosition(),
                driveSubsystem.getSwerveModule("BL").getPosition(),
                driveSubsystem.getSwerveModule("BR").getPosition()
            },
            pose
        );
    }
}
