package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.oi.OI;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;

public class ManualShooterPivotCommand extends Command{
    private final IShooterPivotSubsystem iShooterPivotSubsystem;
    private final OI oi;

    public ManualShooterPivotCommand(IShooterPivotSubsystem iShooterPivotSubsystem, OI oi){
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
        iShooterPivotSubsystem.manualSet(oi.getOperatorController().getAxis(
                Constants.Controls.Operator.ShooterPivotAxis
        ));
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
