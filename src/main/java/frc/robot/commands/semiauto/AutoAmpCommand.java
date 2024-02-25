package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotInfo.AimInfo.LIMELIGHT_STATUS;
import frc.robot.Constants.RobotInfo.ShooterInfo.SHOOTING_MODE;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;

public class AutoAmpCommand extends Command {
    private final IShooterSubsystem shooter;
    private final IShooterPivotSubsystem shooterPivot;
    private final IHopperSubsystem hopper;

    private boolean isShooting;

    public AutoAmpCommand(IShooterSubsystem shooter, IShooterPivotSubsystem shooterPivot, IHopperSubsystem hopper) {
        this.shooter = shooter;
        this.shooterPivot = shooterPivot;
        this.hopper = hopper;

        addRequirements(shooter, shooterPivot, hopper);
    }

    @Override
    public void initialize() {
        isShooting = false;
        shooterPivot.setShooting(SHOOTING_MODE.AMP);
        shooterPivot.setManual(LIMELIGHT_STATUS.LIMELIGHT);
        shooter.setShooting(SHOOTING_MODE.AMP);

        shooterPivot.setAngleDegrees(
            Constants.RobotInfo.ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT
        );
    }

    @Override
    public void execute() {
        shooter.runShooter();

        if(shooter.isUpToSpeed() && shooterPivot.isAtSetPoint()) {
            isShooting = true;
        }

        if(isShooting) {
            hopper.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        hopper.stop();
    }
}