package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.RobotInfo.*;

public class HopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax motor;
    private final DigitalInput ir_sensor;

    public HopperSubsystem(int motorID) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        ir_sensor = new DigitalInput(Constants.IDs.IR_SENSOR);
    }

    @Override
    public void run() {
        motor.set(HopperInfo.HOPPER_SPEED);
    }

    @Override
    public void runBackwards() {
        motor.set(-HopperInfo.HOPPER_SPEED);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean getIRSensor(){
        return ir_sensor.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("IR Sensor", getIRSensor());
    }
}
