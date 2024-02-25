package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;

public class ManualShootCommand extends Command{
    private final IShooterSubsystem shooterSubsystem;
    private final IShooterPivotSubsystem shooterPivotSubsystem;
    private final IHopperSubsystem hopperSubsystem;
    private boolean isShooting = false;

    public ManualShootCommand(IShooterSubsystem shooterSubsystem, IShooterPivotSubsystem shooterPivotSubsystem, IHopperSubsystem hopper){
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopper;

        addRequirements(shooterPivotSubsystem, shooterSubsystem, hopper);
    }

    @Override
    public void initialize(){
        shooterPivotSubsystem.setShooting(true);
        shooterPivotSubsystem.setManual(true);
        shooterPivotSubsystem.setAngleDegrees(Constants.RobotInfo.ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);
    }

    @Override
    public void execute(){
        shooterSubsystem.runShooter();

        if (shooterSubsystem.isUpToSpeed() && shooterPivotSubsystem.isAtSetPoint()){
            isShooting = true;
        }

        if (isShooting){
            hopperSubsystem.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        hopperSubsystem.stop();
    }
}
