package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.ILimelightSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import math.MathUtil;

public class AimShooterCommand extends Command{
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final IShooterPivotSubsystem iShooterPivotSubsystem;
    private final ILimelightSubsystem iLimelightSubsystem;

    public AimShooterCommand(ILimelightSubsystem iLimelightSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, IShooterPivotSubsystem iShooterPivotSubsystem){
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.iShooterPivotSubsystem = iShooterPivotSubsystem;
        this.iLimelightSubsystem = iLimelightSubsystem;

        addRequirements(swerveDriveSubsystem);
        addRequirements(iShooterPivotSubsystem);
    }

    @Override
    public void initialize(){
        iShooterPivotSubsystem.stop();
        swerveDriveSubsystem.stop();
    }


    //todo: swap hardcoded IDs for code determining IDs
    //4 is middle and 3 is right
    @Override
    public void execute(){
        double error = iLimelightSubsystem.getDistZ(4) * MathUtil.sin(iLimelightSubsystem.getDegreesX(4))
        - iLimelightSubsystem.getDistZ(3) * MathUtil.sin(iLimelightSubsystem.getDegreesX(3));
        if (error > -Constants.RobotInfo.AIM_ERROR_CM && error < Constants.RobotInfo.AIM_ERROR_CM){
            double v0X, v0Y;
            v0Y = Math.pow(
                2 * Constants.FieldDistances.SpeakerOptimalHeight * 9.81,
                0.5
            );
            v0X = iLimelightSubsystem.getDistZ(4) * 9.81 / v0Y;
            double aimDegrees = MathUtil.atan(v0Y/v0X);
            iShooterPivotSubsystem.setAngleDegrees(aimDegrees);
        }
        if (Math.abs(error - Constants.FieldDistances.SpeakerApriltagSeparation) < Constants.RobotInfo.AIM_ERROR_CM){
            if (error < 0){
                swerveDriveSubsystem.setMovement(
                    new ChassisSpeeds(
                        Constants.RobotInfo.AIM_SPEED
                        * Math.pow(iLimelightSubsystem.getDistZ(4) 
                        * Constants.RobotInfo.ERROR_CORRECTION_FACTOR, 2),
                        0,
                        0
                    )
                );
            }
            else if (error > 0){
                swerveDriveSubsystem.setMovement(
                    new ChassisSpeeds(
                        -Constants.RobotInfo.AIM_SPEED
                        * Math.pow(iLimelightSubsystem.getDistZ(4) 
                        * Constants.RobotInfo.ERROR_CORRECTION_FACTOR, 2),
                        0,
                        0
                    )
                );
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        iShooterPivotSubsystem.stop();
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        double error = iLimelightSubsystem.getDistZ(4) * MathUtil.sin(iLimelightSubsystem.getDegreesX(4))
        - iLimelightSubsystem.getDistZ(3) * MathUtil.sin(iLimelightSubsystem.getDegreesX(3));
        return Math.abs(error - Constants.FieldDistances.SpeakerApriltagSeparation) < Constants.RobotInfo.AIM_ERROR_CM;
    }
}
