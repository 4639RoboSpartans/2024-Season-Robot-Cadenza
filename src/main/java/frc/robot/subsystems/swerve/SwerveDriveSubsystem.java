package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotInfo.SwerveInfo;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {

    private final SwerveModule
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight;

    private final NavX navx;

    private final OdometrySubsystem odometrySubsystem;

    private ChassisSpeeds chassisSpeeds;

    public SwerveDriveSubsystem() {
        navx = SubsystemManager.getNavX();

        moduleFrontLeft = new SwerveModule(Constants.IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(Constants.IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft = new SwerveModule(Constants.IDs.MODULE_BACK_LEFT);
        moduleBackRight = new SwerveModule(Constants.IDs.MODULE_BACK_RIGHT);
        odometrySubsystem = new OdometrySubsystem(this, navx);
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        setBrakeMode();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::setRawMovement,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2, 0.1, 0.0), //TODO: find constants for 2024 robot
                        new PIDConstants(2, 0, 0.0), //TODO: find constants for 2024 robot
                        4,
                        0.4,
                        new ReplanningConfig()
                ),
                () -> DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Blue).isPresent(),
                this
        );

        resetOdometry(new Pose2d());
    }

    public Pose2d getPose() {
        return odometrySubsystem.getOdometry().getPoseMeters();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return chassisSpeeds;
    }

    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    public void setMovement(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navx.getRotation2d());
        setRawMovement(robotCentricSpeeds);
        chassisSpeeds = robotCentricSpeeds;
    }

    public void setRawMovement(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SwerveInfo.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModulesStates(
                swerveModuleStates[0],
                swerveModuleStates[1],
                swerveModuleStates[2],
                swerveModuleStates[3]
        );
        this.chassisSpeeds = chassisSpeeds;
    }

    private void setModulesStates(
            SwerveModuleState stateFrontLeft,
            SwerveModuleState stateFrontRight,
            SwerveModuleState stateBackLeft,
            SwerveModuleState stateBackRight
    ) {
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

    public void resetOdometry(Pose2d pose) {
        resetPose(pose);
    }

    public void resetPose(Pose2d pose) {
        odometrySubsystem.resetOdometry(pose);
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
}