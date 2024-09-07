package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIntakeSubsystem extends Subsystem {
    enum ExtensionState {
        EXTENDED, RETRACTED, AMP, MANUAL
    }

    void setExtended(ExtensionState extended);

    void outtake();

    void amp();

    void stopIntake();

    void stop();

    void intake();
    
    void manualExtend();

    void updateOffset();
}
