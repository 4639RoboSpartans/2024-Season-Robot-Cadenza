package frc.robot.commands.semiauto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.oi.OI;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import math.MathUtil;

import java.util.ArrayDeque;


public class CenterLimelight extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final AimSubsystem aimSubsystem;
    private final OI oi;
    
    private PIDController rotationPID = Constants.RobotInfo.LIMELIGHT_AIM_PID.create();

    public CenterLimelight(SwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
        this.oi = oi;
        this.swerveDrive = swerveDriveSubsystem;
        aimSubsystem = new AimSubsystem(oi);
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDrive.stop();
        rotationPID.reset();
        rotationPID.setSetpoint(0);
        SmartDashboard.putNumber("rotatorPID kD", 0);
    }

    @Override
    public void execute() {
        swerveDrive.setRawMovement(new ChassisSpeeds(0, 0, aimSubsystem.getRotation()));
        aimSubsystem.updateKD();
        SmartDashboard.putNumber("rotatorPID kD", rotationPID.getD());
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}
