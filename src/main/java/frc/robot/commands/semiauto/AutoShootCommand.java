package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;

public class AutoShootCommand extends Command {
    private final IShooterSubsystem shooter;
    private final IShooterPivotSubsystem shooterPivot;
    private final IHopperSubsystem hopper;

    private boolean isShooting;

    public AutoShootCommand(IShooterSubsystem shooter, IShooterPivotSubsystem shooterPivot, IHopperSubsystem hopper) {
        this.shooter = shooter;
        this.shooterPivot = shooterPivot;
        this.hopper = hopper;

        addRequirements(shooter, shooterPivot, hopper);
    }

    @Override
    public void initialize() {
        isShooting = false;
    }

    @Override
    public void execute() {
        shooter.runShooter();

        if(shooter.isUpToSpeed()) {
            isShooting = true;
        }

        if(isShooting) {
            //hopper.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        hopper.stop();
    }
}