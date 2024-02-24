package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import math.MathUtil;

import static frc.robot.Constants.RobotInfo.*;

public class AimShooterCommand extends Command {
    private final ISwerveDriveSubsystem ISwerveDriveSubsystem;
    private final IShooterPivotSubsystem iShooterPivotSubsystem;

    public AimShooterCommand(LimeLight limeLight, ISwerveDriveSubsystem ISwerveDriveSubsystem, IShooterPivotSubsystem IShooterPivotSubsystem) {
        this.ISwerveDriveSubsystem = ISwerveDriveSubsystem;
        this.iShooterPivotSubsystem = IShooterPivotSubsystem;

        addRequirements(ISwerveDriveSubsystem);
        addRequirements(IShooterPivotSubsystem);
    }

    /*
     * @returns {v0, aimPosition(degrees)}
     */
    public void calculateShooter() {
        double v0X, v0Y;
        v0Y = Math.pow(
                2 * Constants.FieldDistances.SpeakerOptimalHeight * 9.81,
                0.5
        );
        v0X = LimeLight.getZDistance() * 9.81 / v0Y;
        double aimDegrees = MathUtil.atan(v0Y / v0X);
        SmartDashboard.putNumber("Shoot degrees", aimDegrees);
        SmartDashboard.putNumber("Shoot Velocity", Math.pow(v0X * v0X + v0Y * v0Y, 0.5));
    }

    @Override
    public void initialize() {
        iShooterPivotSubsystem.stop();
        ISwerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        double error = LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation())
                - LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation());
        if (error > -AimInfo.AIM_ERROR_CM && error < AimInfo.AIM_ERROR_CM) {
            double v0X, v0Y;
            v0Y = Math.pow(
                    2 * Constants.FieldDistances.SpeakerOptimalHeight * 9.81,
                    0.5
            );
            v0X = LimeLight.getZDistance() * 9.81 / v0Y;
            double aimDegrees = MathUtil.atan(v0Y / v0X);
            iShooterPivotSubsystem.setAngleDegrees(aimDegrees);
        }
        if (Math.abs(error - Constants.FieldDistances.SpeakerApriltagSeparation) < AimInfo.AIM_ERROR_CM) {
            if (error < 0) {
                ISwerveDriveSubsystem.setMovement(
                        new ChassisSpeeds(
                                AimInfo.AIM_SPEED
                                        * Math.pow(LimeLight.getZDistance()
                                        * AimInfo.AIM_SHOOTER_ERROR_CORRECTION_FACTOR, 2),
                                0,
                                0
                        )
                );
            } else if (error > 0) {
                ISwerveDriveSubsystem.setMovement(
                        new ChassisSpeeds(
                                -AimInfo.AIM_SPEED
                                        * Math.pow(LimeLight.getZDistance()
                                        * AimInfo.AIM_SHOOTER_ERROR_CORRECTION_FACTOR, 2),
                                0,
                                0
                        )
                );
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        iShooterPivotSubsystem.stop();
        ISwerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double error = LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation())
                - LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation());
        return Math.abs(error - Constants.FieldDistances.SpeakerApriltagSeparation) < AimInfo.AIM_ERROR_CM;
    }
}
