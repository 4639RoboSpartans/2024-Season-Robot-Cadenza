package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

public class TeleopSwerveDriveCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final PIDController RotationPID;
    private final OI oi;

    public TeleopSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        RotationPID = SwerveInfo.TeleopRotationPID.create();
        this.oi = oi;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double rawForwardsSpeed = DriverControls.SwerveForwardAxis.getAsDouble() * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        double rawSidewaysSpeed = DriverControls.SwerveStrafeAxis.getAsDouble() * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        double rawRotationSpeed = DriverControls.SwerveRotationAxis.getAsDouble() * SwerveInfo.TELOP_ROTATION_SPEED;
        double forwardsSpeed = getForwardsSpeed(rawForwardsSpeed), sidewaysSpeed = getSidewaysSpeed(rawSidewaysSpeed);
        double rotationMultiplier = Math.hypot(forwardsSpeed, sidewaysSpeed) / 2;
        double rotateSpeed = getRotationSpeed(rawForwardsSpeed, rawSidewaysSpeed, rawRotationSpeed, rotationMultiplier);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardsSpeed, sidewaysSpeed, rotateSpeed);
        swerveDriveSubsystem.setFieldCentricMovement(chassisSpeeds);
        // SmartDashboard.putNumber("navX heading", SubsystemManager.getNavX().getHeading());
    }

    private double getRotationSpeed(double rawForwardsSpeed,
                                    double rawSidewaysSpeed,
                                    double rawRotationSpeed,
                                    double rotationMultiplier) {
        double rawSpeed;
        Rotation2d heading = swerveDriveSubsystem.getRotation2d();
        Rotation2d speaker = AimUtil.getSpeakerRotation(0, 0);
        SmartDashboard.putNumber("speaker angle", speaker.getDegrees());
        SmartDashboard.putNumber("speaker x", AimUtil.getSpeakerVector().getX());
        SmartDashboard.putNumber("speaker y", AimUtil.getSpeakerVector().getY());
        SmartDashboard.putNumber("heading", heading.getDegrees());
        SmartDashboard.putNumber("speaker offset", heading.getDegrees() - speaker.getDegrees());
        if (DriverControls.AimButton.getAsBoolean()) {
            rawSpeed = RotationPID.calculate(swerveDriveSubsystem.getRotation2d().getDegrees(),
                    AimUtil.getSpeakerRotation().getRadians());
        } else if (DriverControls.SOTF.getAsBoolean()) {
            rawSpeed = RotationPID.calculate(swerveDriveSubsystem.getRotation2d().getDegrees(),
                    AimUtil.getSpeakerRotation(rawForwardsSpeed, rawSidewaysSpeed).getDegrees());
        } else {
            rawSpeed = rawRotationSpeed;
        }
        return rawSpeed * (1 + rotationMultiplier);
    }

    public double getForwardsSpeed(double forwardsSpeed) {
        if (DriverControls.SOTF.getAsBoolean()) {
            return forwardsSpeed / 2;//TODO: tune this
        }
        return forwardsSpeed;
    }

    public double getSidewaysSpeed(double sidewaysSpeed) {
        if (DriverControls.SOTF.getAsBoolean()) {
            return sidewaysSpeed / 2;//TODO: tune this
        }
        return sidewaysSpeed;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
