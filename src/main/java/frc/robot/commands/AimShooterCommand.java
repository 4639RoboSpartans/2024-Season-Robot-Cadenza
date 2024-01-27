package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import math.MathUtil;

public class AimShooterCommand extends Command{
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final IShooterPivotSubsystem iShooterPivotSubsystem;

    public AimShooterCommand(LimeLight limeLight, SwerveDriveSubsystem swerveDriveSubsystem, IShooterPivotSubsystem iShooterPivotSubsystem){
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.iShooterPivotSubsystem = iShooterPivotSubsystem;

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
        double error = LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation())
        - LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation());
        if (error > -Constants.RobotInfo.AIM_ERROR_CM && error < Constants.RobotInfo.AIM_ERROR_CM){
            double v0X, v0Y;
            v0Y = Math.pow(
                2 * Constants.FieldDistances.SpeakerOptimalHeight * 9.81,
                0.5
            );
            v0X = LimeLight.getZDistance() * 9.81 / v0Y;
            double aimDegrees = MathUtil.atan(v0Y/v0X);
            iShooterPivotSubsystem.setAngleDegrees(aimDegrees);
        }
        if (Math.abs(error - Constants.FieldDistances.SpeakerApriltagSeparation) < Constants.RobotInfo.AIM_ERROR_CM){
            if (error < 0){
                swerveDriveSubsystem.setMovement(
                    new ChassisSpeeds(
                        Constants.RobotInfo.AIM_SPEED
                        * Math.pow(LimeLight.getZDistance() 
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
                        * Math.pow(LimeLight.getZDistance() 
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
        double error = LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation())
        - LimeLight.getZDistance() * MathUtil.sin(LimeLight.getXRotation());
        return Math.abs(error - Constants.FieldDistances.SpeakerApriltagSeparation) < Constants.RobotInfo.AIM_ERROR_CM;
    }
}
