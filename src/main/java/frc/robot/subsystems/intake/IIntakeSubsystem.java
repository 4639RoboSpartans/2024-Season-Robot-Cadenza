package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface IIntakeSubsystem extends Subsystem {
    enum ExtensionState {
        EXTENDED, RETRACTED, AMP
    }

    Command setExtended(ExtensionState extended);

    Command outtake();

    Command amp();

    Command stopIntake();

    Command stop();

    Command intake();
}
