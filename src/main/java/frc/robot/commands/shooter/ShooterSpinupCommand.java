package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShooterSpinupCommand extends Command {
    private final IShooterSubsystem shooterSubsystem;
    private double startTime;
    private final double time;
    public ShooterSpinupCommand(IShooterSubsystem shooterSubsystem, double time){
        this.shooterSubsystem = shooterSubsystem;
        this.time = time;
        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize(){
        shooterSubsystem.setShootingMode(RobotInfo.ShooterInfo.ShootingMode.AUTO_SPEAKER);
        startTime = Timer.getFPGATimestamp();
    }
    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() - startTime >= time;
    }
}
