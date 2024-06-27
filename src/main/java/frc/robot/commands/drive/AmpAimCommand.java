package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.aim.AimSubsystem;
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
        AimSubsystem.driveToAmp();
        swerveDriveSubsystem.setMovement(AimSubsystem.getRobotRelative(swerveDriveSubsystem.getPose()));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    // TODO: test before using in matches
//    @Override
//    public boolean isFinished() {
//        return aimSubsystem.isAtSetpoint();
//    }
}
