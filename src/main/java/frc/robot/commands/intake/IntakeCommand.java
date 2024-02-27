package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.sensors.IRTest;

public class IntakeCommand extends Command {
    private final IIntakeSubsystem intake;
    private final IHopperSubsystem hopper;
    private final IRTest ir;

    public IntakeCommand(IIntakeSubsystem intake, IHopperSubsystem hopper, IRTest ir) {
        this.intake = intake;
        this.hopper = hopper;
        this.ir = ir;

        addRequirements(intake, hopper);
    }

    @Override
    public void execute() {
        intake.intake();
        hopper.run();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        hopper.stop();
    }

    @Override
    public boolean isFinished(){
        if (Constants.RobotInfo.HopperInfo.usingIRSensor){
            if(ir.getIRSensor()){
                intake.setExtended(false);
                return true;
            }
        }
        return false;
    }
}
