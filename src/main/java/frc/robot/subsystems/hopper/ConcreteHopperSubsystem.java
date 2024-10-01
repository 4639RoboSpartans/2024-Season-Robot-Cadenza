package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class ConcreteHopperSubsystem extends HopperSubsystem {
    private final CANSparkMax motor;
    private final DigitalInput ir;
    private boolean hasNote = false;
    private boolean irActive;
    private Debouncer irDebouncer = new Debouncer(0.2);

    protected ConcreteHopperSubsystem() {
        motor = new CANSparkMax(HopperConstants.IDs.HOPPER_MOTOR, CANSparkMax.MotorType.kBrushed);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.ir = new DigitalInput(HopperConstants.IDs.IR_SENSOR_1_DIO_PORT);
        irActive = true;
    }

    @Override
    protected void feedRun() {
        motor.set(HopperConstants.HOPPER_SPEED);
    }


    @Override
    protected void outtakeRun() {
        motor.set(-HopperConstants.HOPPER_SPEED);
    }

    @Override
    protected void stopRun() {
        motor.stopMotor();
    }

    @Override
    protected void toggleIRRun() {
        irActive = !irActive;
    }

    protected boolean hasNoteSupplier() {
        return hasNote && !irActive;
    }
    
    public void periodic() {
        hasNote = irDebouncer.calculate(!ir.get());
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hopper");
        builder.addDoubleProperty("Hopper output",
                motor::getAppliedOutput,
                null);
        builder.addBooleanProperty("Has note",
                this::hasNoteSupplier,
                null);
    }

    @Override
    public Command simToggleHasNote(boolean hasNote) {
        return run(this::hasNoteSupplier); // ignore this since it is only for simulation purposes
    }
}
