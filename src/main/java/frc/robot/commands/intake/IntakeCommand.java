package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DisplayInfo;
import frc.robot.constants.RobotInfo.HopperInfo;
import frc.robot.constants.RobotInfo.IntakeInfo;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem.ExtensionState;

public class IntakeCommand extends Command {
    private final IIntakeSubsystem intake;
    private final IHopperSubsystem hopper;
    private final LEDStrip strip;
    private double noteTime;
    private boolean seen = false;

    public IntakeCommand(IIntakeSubsystem intake, IHopperSubsystem hopper, LEDStrip strip) {
        this.intake = intake;
        this.hopper = hopper;
        this.strip = strip;

        addRequirements(intake, hopper, strip);
        
    }

    @Override
    public void execute() {
        intake.setExtended(IIntakeSubsystem.ExtensionState.EXTENDED);
        intake.intake();
        if (!seen){
            hopper.run(false, HopperInfo.HOPPER_SPEED);
            if (hopper.getIR().hasNote()){
                noteTime = Timer.getFPGATimestamp();
                seen = true;
                intake.setExtended(ExtensionState.RETRACTED);
            }
        }
        else {
            hopper.run(false, HopperInfo.HOPPER_SPEED / 2, true);
        }

        strip.usePattern(DisplayInfo.intakePattern);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        hopper.stop();

        if(!interrupted) {
            intake.setExtended(IIntakeSubsystem.ExtensionState.RETRACTED);
        }

        seen = false;
    }

    @Override
    public boolean isFinished(){
        return seen && Timer.getFPGATimestamp() - noteTime > 0.12;
    }

}