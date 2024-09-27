package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DummyIntakeSubsystem extends IIntakeSubsystem {
    private ExtensionState state;
    private final DCMotorSim simPivot;

    public DummyIntakeSubsystem() {
        state = ExtensionState.RETRACTED;
        simPivot = new DCMotorSim(DCMotor.getNeo550(2), 1.0/81.0*16.0/36.0, )
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake state", state.toString());
    }

    @Override
    protected void setExtendedState(ExtensionState extended) {

    }

    @Override
    protected void outtakeRun() {

    }

    @Override
    protected void ampRun() {

    }

    @Override
    protected void stopIntakeRun() {

    }

    @Override
    protected void stopRun() {

    }

    @Override
    protected void intakeRun() {

    }

    @Override
    public Command setExtended(ExtensionState extended) {
        return null;
    }

    @Override
    public Command outtake() {
        return null;
    }

    @Override
    public Command amp() {
        return null;
    }

    @Override
    public Command stopIntake() {
        return null;
    }

    @Override
    public Command stop() {
        return null;
    }

    @Override
    public Command intake() {
        return null;
    }
}
