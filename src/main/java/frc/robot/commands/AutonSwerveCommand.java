package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutonSwerveCommand extends Command{
    public final SwerveDriveSubsystem swerveDriveSubsystem;
    public final double distanceX, distanceY, maxSpeed;
    public final double startTime;
    public final double vMaxX, vMaxY;

    public AutonSwerveCommand(SwerveDriveSubsystem swerveDriveSubsystem, double distanceX, double distanceY, double maxSpeed){
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.distanceX = distanceX;
        this.distanceY = distanceY;
        this.maxSpeed = maxSpeed;
        vMaxX = maxSpeed / Math.pow(Math.pow(distanceX / distanceY, 2) + 1, 0.5);
        vMaxY = maxSpeed / Math.pow(Math.pow(distanceY / distanceX, 2) + 1, 0.5);
        startTime = Timer.getFPGATimestamp();

        addRequirements(swerveDriveSubsystem);
    }

    public double getSpeed(double time, double distance, double maxSpeed){
        double acceleration = maxSpeed * maxSpeed;
        double halfTime = distance/maxSpeed;
        return -acceleration * Math.abs(time - halfTime) + maxSpeed;
    }

    @Override
    public void initialize(){
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute(){
        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        double xVelocity = getSpeed(timeElapsed, distanceX, vMaxX);
        double yVelocity = getSpeed(timeElapsed, distanceY, vMaxY);
        swerveDriveSubsystem.setMovement(
            new ChassisSpeeds(xVelocity, yVelocity, 0)
        );
    }

    @Override
    public void end(boolean interrupted){
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
