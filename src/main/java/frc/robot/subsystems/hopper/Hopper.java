package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hopper.constants.HopperConstants;

import java.util.function.DoubleConsumer;

public class Hopper extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax hopper;
    private final Trigger hasNoteTrigger;
    private boolean hasNote;
    private final DigitalInput ir;
    private final Debouncer noteDebouncer;

    private static Hopper instance;

    private Hopper() {
        hopper = new CANSparkMax(HopperConstants.IDs.HOPPER_MOTOR, CANSparkMax.MotorType.kBrushed);
        noteDebouncer = new Debouncer(0.22);
        ir = new DigitalInput(HopperConstants.IDs.IR_SENSOR_1_DIO_PORT);
        hasNoteTrigger = new Trigger(this::hasNoteSupplier);
    }

    @Override
    public Command feed() {
        return runOnce(() -> hopper.set(HopperConstants.HOPPER_SPEED));
    }

    @Override
    public Command outtake() {
        return runOnce(() -> hopper.set(-HopperConstants.HOPPER_SPEED));
    }

    @Override
    public Trigger hasNote() {
        return hasNoteTrigger;
    }

    @Override
    public Command stop() {
        return runOnce(() -> hopper.set(0));
    }

    private boolean rawHasNoteSupplier() {
        return !ir.get();
    }

    private boolean hasNoteSupplier() {
        return hasNote;
    }

    @Override
    public void periodic() {
        hasNote = noteDebouncer.calculate(rawHasNoteSupplier());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hopper");
        builder.addBooleanProperty("Has note",
                this::hasNoteSupplier,
                (BooleanConsumer) null);
        builder.addDoubleProperty("Speed",
                hopper::getAppliedOutput,
                (DoubleConsumer) null);
    }

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }
}
