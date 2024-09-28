package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls;
import frc.robot.constants.DisplayInfo;
import frc.robot.constants.RobotInfo.HopperInfo;
import frc.robot.led.LEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.ConcreteIntakeSubsystem;
import frc.robot.subsystems.intake.ConcreteIntakeSubsystem.ExtensionState;

public class IntakeRollersCommand extends Command {
    private final ConcreteIntakeSubsystem intake;
    private final HopperSubsystem hopper;
    private final LEDStrip strip;
    private final OI oi;
    private double noteTime;
    private boolean seen = false;

    public IntakeRollersCommand(ConcreteIntakeSubsystem intake, HopperSubsystem hopper, LEDStrip strip, OI oi) {
        this.intake = intake;
        this.hopper = hopper;
        this.strip = strip;
        this.oi = oi;

        addRequirements(intake, hopper, strip);

    }

    @Override
    public void execute() {
        intake.setExtended(ConcreteIntakeSubsystem.ExtensionState.EXTENDED);
        intake.intake();
        if (!seen){
            hopper.run(false, HopperInfo.HOPPER_SPEED);
            if (hopper.hasNote()){
                noteTime = Timer.getFPGATimestamp();
                seen = true;
                intake.setExtended(ExtensionState.RETRACTED);
                oi.driverController().rumble(Controls.rumbleStrength);
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
            intake.setExtended(ConcreteIntakeSubsystem.ExtensionState.RETRACTED);
        }

        seen = false;
        oi.driverController().stopRumble();
    }

    @Override
    public boolean isFinished(){
        return seen && Timer.getFPGATimestamp() - noteTime > 0;
    }

}