package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class HopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax motor;
    private final DigitalInput ir;
    private boolean hasNote = false;
    private boolean irActive;
    private Debouncer irDebouncer = new Debouncer(0.175);

    public HopperSubsystem(int motorID) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.ir = new DigitalInput(IDs.IR_SENSOR_1_DIO_PORT);
        irActive = true;
    }

    @Override
    public void run(boolean checkNote, double speed) {
        if (hasNote && checkNote) {
            motor.set(0);
        } else {
            motor.set(speed);
        }
    }

    @Override
    public void run(boolean checkNote, double speed, boolean reversed){
        if (reversed){
            run(checkNote, -speed);
        }
        else{
            run(checkNote, speed);
        }
    }

    @Override
    public void runBackwards(double speed) {
        motor.set(-speed);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean hasNote() {
        return hasNote && irActive;
    }

    public boolean isIrActive() {
        return irActive;
    }

    public Command toggleIR() {
        return Commands.runOnce(() -> {
            irActive = !irActive;
        }, this);
    }

    @Override
    public void periodic() {
        hasNote = irDebouncer.calculate(!ir.get());
        SmartDashboard.putBoolean("Hopper/has note", hasNote);
        SmartDashboard.putBoolean("Hopper/ir active", irActive);
    }
}
