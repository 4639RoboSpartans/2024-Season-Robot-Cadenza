package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class IRSensor extends SubsystemBase {
    private final DigitalInput sensor;
    private boolean active;

    public IRSensor(){
        sensor = new DigitalInput(Constants.IDs.IR_SENSOR);
    }

    public boolean isClear(){
        return sensor.get();
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return this.active;
    }

    public boolean hasNote() {
        return active && !isClear();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("IR sensor detects note", hasNote());
    }
}
