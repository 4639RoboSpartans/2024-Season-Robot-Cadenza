package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.led.SolidLEDPattern;
import frc.robot.subsystems.SubsystemManager;
import math.Averager;

public class IRSensor extends SubsystemBase {
    private final DigitalInput sensor;
    private boolean active = true;

    private final Averager detection = new Averager(3);

    public IRSensor(){
        sensor = new DigitalInput(Constants.IDs.IR_SENSOR);
    }

    public boolean rawSensorIsClear() {
        return sensor.get();
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return this.active;
    }

    public boolean hasNote() {
        return active && detection.getValue() >= 0.5;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("IR sensor detects note", !rawSensorIsClear());
        SmartDashboard.putBoolean("IR sensor active", isActive());
        SmartDashboard.putBoolean("IR sensor has note", hasNote());

        detection.addMeasurement(rawSensorIsClear() ? 0 : 1);

        if(hasNote()) {
            SubsystemManager.getLedStrip().usePattern(new SolidLEDPattern(new Color8Bit(255, 200, 0)));
        }
        else {
            SubsystemManager.getLedStrip().usePattern(new SolidLEDPattern(new Color8Bit(0, 0, 255)));
        }
    }
}
