package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls;
import frc.robot.oi.OI;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.MovementUtil;

public class TeleopSwerveDriveCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final OI oi;
    private boolean turning = true;

    public TeleopSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.oi = oi;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
        MovementUtil.reset();
        MovementUtil.lockToSpeaker();
    }

    @Override
    public void execute() {
        OI.AxesInputs turnInputs = oi.getRightStickInputs();
        double turnInput = turnInputs.x;
        if (oi.driverController().getButton(Controls.DriverControls.AimButton).getAsBoolean()){
            turning = false;
        }
        if (turnInput != 0) {
            turning = true;
        }
        MovementUtil.setLocked(turnInput == 0 && !turning);
        Pose2d currPose = swerveDriveSubsystem.getPose();
        swerveDriveSubsystem.setMovement(MovementUtil.getRobotRelative(currPose));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
        MovementUtil.reset();
    }
}
