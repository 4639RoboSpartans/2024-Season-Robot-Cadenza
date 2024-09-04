package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class AmpAimCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;

    public AmpAimCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double forwardsCorrection = (LimeLight.getZDistance()) * 0.25;
        double sidewaysCorrection = -(LimeLight.getXDistance()) * 0.25;
        double rotateSpeed = Math.tanh(LimeLight.getYRotation() * 0.08) * 0.1;

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
