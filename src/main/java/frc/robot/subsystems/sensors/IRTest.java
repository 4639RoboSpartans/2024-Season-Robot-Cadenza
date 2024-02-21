package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IRTest extends SubsystemBase{
    private final DigitalInput ir_sensor;
    public IRTest(){
        ir_sensor = new DigitalInput(Constants.IDs.IR_SENSOR);
    }

    public boolean getIRSensor(){
        return ir_sensor.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Test IR Sensor", getIRSensor());
    }
}
