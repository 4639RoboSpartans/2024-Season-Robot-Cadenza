package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.network.LimelightHelpers;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {

    private final Module
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight;

    private final NavX navx;

    private final SwerveDrivePoseEstimator poseEstimator;

    private ChassisSpeeds chassisSpeeds;

    private final Field2d field = new Field2d();

    public SwerveDriveSubsystem() {
        navx = SubsystemManager.getNavX();

        moduleFrontLeft = new Module(new ModuleIOTalonFX(0), 0);
        moduleFrontRight = new Module(new ModuleIOTalonFX(1), 1);
        moduleBackLeft = new Module(new ModuleIOTalonFX(2), 2);
        moduleBackRight = new Module(new ModuleIOTalonFX(3), 3);
        poseEstimator = new SwerveDrivePoseEstimator(SwerveInfo.SWERVE_DRIVE_KINEMATICS, getRotation2d(), getPositions(), new Pose2d());
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::setRawMovement,
                new HolonomicPathFollowerConfig(
                        SwerveInfo.TranslationPID, //TODO: find constants for 2024 robot
                        SwerveInfo.RotationPID, //TODO: find constants for 2024 robot
                        4,
                        0.4,
                        new ReplanningConfig()
                ),
                () -> RobotContainer.alliance.getSelected(),
                this
        );

        resetOdometry(new Pose2d());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
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
        moduleFrontLeft.runSetpoint(stateFrontLeft);
        moduleFrontRight.runSetpoint(stateFrontRight);
        moduleBackLeft.runSetpoint(stateBackLeft);
        moduleBackRight.runSetpoint(stateBackRight);
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
        SmartDashboard.putData(field);
        Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        if (limelightPose.getTranslation().getX() != 0 && limelightPose.getTranslation().getY() != 0){
            poseEstimator.addVisionMeasurement(limelightPose, getHeading());
        }
    }

    public void resetOdometry(Pose2d pose) {
        resetPose(pose);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(navx.getRotation2d(), getPositions(), pose);
    }

    public Module getSwerveModule(String module) {
        return switch (module) {
            case "FL" -> moduleFrontLeft;
            case "BL" -> moduleBackLeft;
            case "FR" -> moduleFrontRight;
            case "BR" -> moduleBackRight;
            default -> null;
        };
    }

    public double getHeading(){
        return navx.getHeading();
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            getSwerveModule("FL").getPosition(),
            getSwerveModule("FR").getPosition(),
            getSwerveModule("BL").getPosition(),
            getSwerveModule("BR").getPosition(),
        };
    }
}