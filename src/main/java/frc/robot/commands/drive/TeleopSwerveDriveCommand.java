package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotInfo.*;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class TeleopSwerveDriveCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final OI oi;
    private final PIDController rotationPID;
    private final PIDController aimPID;
    private boolean turning = false;

    public TeleopSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.oi = oi;
        addRequirements(swerveDriveSubsystem);
        rotationPID = SwerveInfo.TELEOP_ROTATION_PID.create();
        aimPID = AimInfo.aimPID.create();
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double forwardsSpeed =
                oi.driverController().getAxis(DriverControls.SwerveForwardAxis)
                        * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        double sidewaysSpeed =
                -oi.driverController().getAxis(DriverControls.SwerveStrafeAxis)
                        * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        double rotationMultiplier = Math.hypot(forwardsSpeed, sidewaysSpeed) / 2;
        double rotateSpeed = getRotationSpeed(rotationMultiplier);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardsSpeed, sidewaysSpeed, rotateSpeed);
        swerveDriveSubsystem.setMovement(chassisSpeeds);
        SmartDashboard.putNumber("navX heading", SubsystemManager.getNavX().getHeading());
    }

    private double getRotationSpeed(double rotationMultiplier) {
        double driverRotationInput = -oi.driverController().getAxis(DriverControls.SwerveRotationAxis);
        double rawSpeed;
        if (oi.driverController().getButton(DriverControls.AimButton).getAsBoolean()) {
            Translation2d SpeakerRelativeBotPose = getSpeakerToRobotVector();
            Rotation2d botAngle = new Rotation2d(
                    Math.atan(SpeakerRelativeBotPose.getY()
                            / SpeakerRelativeBotPose.getX()));
            swerveDriveSubsystem.setDesiredRotation(botAngle);
            Rotation2d currAngle = swerveDriveSubsystem.getRotation2d();
            double rotationOffset = currAngle.minus(botAngle).getRadians();
            double givenOffset = Math.copySign(
                    Math.pow(rotationOffset, 1 + rotationMultiplier * AimInfo.aimScalar),
                    rotationOffset);
            rawSpeed = aimPID.calculate(givenOffset);
            if (turning) turning = false;
        }
        else if (driverRotationInput != 0) {
            rawSpeed = driverRotationInput * SwerveInfo.TELOP_ROTATION_SPEED;
            turning = true;
            return rawSpeed * (1 + rotationMultiplier);
        } else {
            Rotation2d currAngle = swerveDriveSubsystem.getRotation2d();
            Rotation2d desiredAngle = swerveDriveSubsystem.getDesiredRotation();
            double rotationOffset = currAngle.minus(desiredAngle).getRadians();
            rawSpeed = rotationPID.calculate(rotationOffset);
            if (turning) {
                swerveDriveSubsystem.resetDesiredRotation();
                turning = false;
            }
        }
        return rawSpeed;
    }

    public Translation2d getSpeakerToRobotVector() {
        Pose2d currBotPose = swerveDriveSubsystem.getPose();
        Translation2d currBotTranslation = currBotPose.getTranslation();
        Translation2d speakerPose;
        if (SmartDashboard.getBoolean("Alliance", false)) {
            speakerPose = FieldConstants.speakerPose_red;
        } else {
            speakerPose = FieldConstants.speakerPose_blue;
        }
        return currBotTranslation.minus(speakerPose);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
