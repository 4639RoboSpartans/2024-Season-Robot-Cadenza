package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.oi.OI;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;

public class ShooterPivotCommand extends Command{
    private final IShooterPivotSubsystem iShooterPivotSubsystem;
    private final OI oi;

    public ShooterPivotCommand(IShooterPivotSubsystem iShooterPivotSubsystem, OI oi){
        this.iShooterPivotSubsystem = iShooterPivotSubsystem;
        this.oi = oi;

        addRequirements(iShooterPivotSubsystem);
    }

    @Override
    public void initialize(){
        iShooterPivotSubsystem.stop();
    }

    @Override
    public void execute(){
        iShooterPivotSubsystem.setAngleDegrees(oi.getOperatorController().getAxis(OI.Axes.LEFT_STICK_Y));
    }

    @Override
    public void end(boolean interrupted){
        iShooterPivotSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
