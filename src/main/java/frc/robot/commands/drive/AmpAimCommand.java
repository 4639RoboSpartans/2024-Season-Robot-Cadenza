package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotInfo;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.util.DriverStationUtil;

public class AmpAimCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final PIDController rotationPID;
    private final PIDController xPID, yPID;

    public AmpAimCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        rotationPID = RobotInfo.SwerveInfo.TeleopRotationPID.create();
        xPID = RobotInfo.SwerveInfo.TeleopTranslationPID.create();
        yPID = RobotInfo.SwerveInfo.TeleopTranslationPID.create();
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        Translation2d ampPose = DriverStationUtil.isRed()? FieldConstants.ampPose_red: FieldConstants.ampPose_blue;
        double rotateSpeed = rotationPID.calculate(SubsystemManager.getSwerveDrive().getRotation2d().getRadians(), AimUtil.getAmpRotation().getRadians());
        double forwardsCorrection = xPID.calculate(ampPose.getY(), SubsystemManager.getSwerveDrive().getPose().getY());
        double sidewaysCorrection = -yPID.calculate(ampPose.getX(), SubsystemManager.getSwerveDrive().getPose().getX());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                forwardsCorrection,
                sidewaysCorrection,
                rotateSpeed
        );
        swerveDriveSubsystem.setRawMovement(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
