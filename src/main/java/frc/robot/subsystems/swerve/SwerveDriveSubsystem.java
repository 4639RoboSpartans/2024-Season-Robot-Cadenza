package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.network.LimelightHelpers;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {

    private final SwerveModule
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight;

    private final NavX navx;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    private ChassisSpeeds chassisSpeeds;

    public SwerveDriveSubsystem() {
        navx = SubsystemManager.getNavX();

        moduleFrontLeft = new SwerveModule(IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft = new SwerveModule(IDs.MODULE_BACK_LEFT);
        moduleBackRight = new SwerveModule(IDs.MODULE_BACK_RIGHT);
        poseEstimator = new SwerveDrivePoseEstimator(SwerveInfo.SWERVE_DRIVE_KINEMATICS, navx.getRotation2d(), getPositions(), new Pose2d());
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        setBrakeMode();

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
        field = new Field2d();
    }

    public void useTeleopCurrentLimits() {
        moduleFrontLeft.useTeleopCurrentLimits();
        moduleFrontRight.useTeleopCurrentLimits();
        moduleBackLeft.useTeleopCurrentLimits();
        moduleBackRight.useTeleopCurrentLimits();
    }

    public void useAutonCurrentLimits() {
        moduleFrontLeft.useAutonCurrentLimits();
        moduleFrontRight.useAutonCurrentLimits();
        moduleBackLeft.useAutonCurrentLimits();
        moduleBackRight.useAutonCurrentLimits();
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
        poseEstimator.update(navx.getRotation2d(), getPositions());
        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        Translation2d limelightTrans = limelightPose.getTranslation();
        if (limelightTrans.getX() != 0 && limelightTrans.getY() != 0) {
            poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp());
        }
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
        poseEstimator.resetPosition(navx.getRotation2d(), getPositions(), pose);
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