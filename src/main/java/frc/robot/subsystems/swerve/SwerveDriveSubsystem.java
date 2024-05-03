package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {

    private final SwerveModule
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight;

    private final NavX navx;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private ChassisSpeeds chassisSpeeds;

    private final Field2d m_field = new Field2d();

    public SwerveDriveSubsystem() {
        SmartDashboard.putData("Field", m_field);
        navx = SubsystemManager.getNavX();

        moduleFrontLeft = new SwerveModule(IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft = new SwerveModule(IDs.MODULE_BACK_LEFT);
        moduleBackRight = new SwerveModule(IDs.MODULE_BACK_RIGHT);
        m_poseEstimator = new SwerveDrivePoseEstimator(SwerveInfo.SWERVE_DRIVE_KINEMATICS, navx.getRotation2d(), getStates(), LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        setBrakeMode();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::setRawMovement,
                new HolonomicPathFollowerConfig(
                        SwerveInfo.TranslationPID,
                        SwerveInfo.RotationPID,
                        4,
                        0.4,
                        new ReplanningConfig()
                ),
                () -> RobotContainer.alliance.getSelected(),
                this
        );
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
        return m_poseEstimator.getEstimatedPosition();
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
        updateOdometry();
        m_field.setRobotPose(getPose());
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("FL", moduleFrontLeft.getPosition().distanceMeters);
        SmartDashboard.putNumber("FR", moduleFrontRight.getPosition().distanceMeters);
        SmartDashboard.putNumber("BL", moduleBackLeft.getPosition().distanceMeters);
        SmartDashboard.putNumber("BR", moduleBackRight.getPosition().distanceMeters);
    }

    public void updateOdometry() {
        m_poseEstimator.update(
            navx.getRotation2d(),
            getStates());
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(navx.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
        doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
        doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
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

    public SwerveModulePosition[] getStates(){
        return new SwerveModulePosition[]{
            moduleFrontLeft.getPosition(),
            moduleFrontRight.getPosition(),
            moduleBackLeft.getPosition(),
            moduleBackRight.getPosition()
        };
    }

    public void resetPose(Pose2d pose){
        m_poseEstimator.resetPosition(new Rotation2d(getHeading()), getStates(), pose);
    }
}