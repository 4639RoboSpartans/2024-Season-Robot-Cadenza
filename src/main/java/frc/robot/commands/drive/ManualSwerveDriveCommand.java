package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Controls.DriverControls;
import frc.robot.Constants.Controls.OperatorControls;
import frc.robot.oi.OI;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

import static frc.robot.Constants.Controls.*;

public class ManualSwerveDriveCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final AimSubsystem aimSubsystem;
    private final OI oi;

    public ManualSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem, AimSubsystem aimSubsystem, OI oi) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.aimSubsystem = aimSubsystem;
        this.oi = oi;
        addRequirements(swerveDriveSubsystem, aimSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double forwardsSpeed = oi.driverController().getAxis(DriverControls.SwerveForwardAxis);
        double sidewaysSpeed = -oi.driverController().getAxis(DriverControls.SwerveStrafeAxis);

        double rotateSpeed;
        if(oi.operatorController().getButton(OperatorControls.AimButton).getAsBoolean()) {
            rotateSpeed = -aimSubsystem.getRotationSpeed();
        }
        else {
            rotateSpeed = -oi.driverController().getAxis(DriverControls.SwerveRotationAxis);
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardsSpeed, sidewaysSpeed, rotateSpeed);
        swerveDriveSubsystem.setMovement(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
