package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.oi.OI;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class ManualSwerveDriveCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final OI oi;

    public ManualSwerveDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.oi = oi;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double forwardsSpeed = oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveForwardAxis);
        double sidewaysSpeed = -oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveStrafeAxis);
        double rotateSpeed = -oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveRotationAxis);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardsSpeed, sidewaysSpeed, rotateSpeed);
        swerveDriveSubsystem.setMovement(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}