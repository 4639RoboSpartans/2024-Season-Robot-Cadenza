package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class MoveCommand extends Command{
    private final SwerveDriveSubsystem swerve;
    private SlewRateLimiter xLimiter, yLimiter;
    private double xLimit, yLimit = 0.65;
    private double xSpeedRaw, ySpeedRaw, turningSpeedRaw, startTime, duration;
    public MoveCommand(SwerveDriveSubsystem swerve, double xSpeedRaw, double ySpeedRaw, double turningSpeedRaw, double duration){
        this.swerve = swerve;
        this.xSpeedRaw = xSpeedRaw;
        this.ySpeedRaw = ySpeedRaw;
        this.turningSpeedRaw = turningSpeedRaw;
        this.duration = duration;
        
        xLimiter = new SlewRateLimiter(xLimit);
        yLimiter = new SlewRateLimiter(yLimit);

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        swerve.setBrakeMode();
        swerve.setMovement(new ChassisSpeeds());
    }

    @Override
    public void execute(){
        double currTime = getElapsedTime();
        double speedMultiplier = 1 - Math.abs(1 - 2 * currTime / duration);

        double xSpeed = xLimiter.calculate(xSpeedRaw * speedMultiplier)*Constants.RobotInfo.MAX_ROBOT_SPEED;
        double ySpeed = yLimiter.calculate(ySpeedRaw * speedMultiplier)*Constants.RobotInfo.MAX_ROBOT_SPEED;
        double turningSpeed = turningSpeedRaw*Constants.RobotInfo.MAX_ROBOT_SPEED;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, turningSpeed, swerve.getRotation2d());

        swerve.setMovement(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted){
        swerve.stop();
        swerve.setCoastMode();
    }

    private double getElapsedTime() {
        return Timer.getFPGATimestamp() - startTime;
    }

    @Override
    public boolean isFinished() {
        return getElapsedTime() > duration;
    }
}
