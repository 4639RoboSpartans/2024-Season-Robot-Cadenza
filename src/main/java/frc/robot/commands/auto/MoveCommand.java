package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class MoveCommand extends Command{
    private final SwerveDriveSubsystem swerve;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private double xLimit;
    private final double yLimit = 0.65;
    private final double xSpeedRaw;
    private final double ySpeedRaw;
    private final double turningSpeedRaw;
    private double startTime;
    private final double duration;
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

//        double xSpeed = xLimiter.calculate(xSpeedRaw * speedMultiplier) * Constants.RobotInfo.MAX_ROBOT_SPEED;
//        double ySpeed = yLimiter.calculate(ySpeedRaw * speedMultiplier) * Constants.RobotInfo.MAX_ROBOT_SPEED;
        double xSpeed = xSpeedRaw * speedMultiplier * SwerveInfo.MAX_ROBOT_SPEED;
        double ySpeed = ySpeedRaw * speedMultiplier * SwerveInfo.MAX_ROBOT_SPEED;
        double turningSpeed = turningSpeedRaw * speedMultiplier * SwerveInfo.MAX_ROBOT_SPEED;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(-ySpeed, xSpeed, turningSpeed);

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
