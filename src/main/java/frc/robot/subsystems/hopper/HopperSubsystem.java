package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.IRTest;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class HopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax motor;
    private final IRTest ir;

    public HopperSubsystem(int motorID, IRTest ir) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.ir = ir;
    }

    @Override
    public void run() {
        if (Constants.RobotInfo.HopperInfo.usingIRSensor)
            if (ir.getIRSensor())
                motor.set(HopperInfo.HOPPER_SPEED);
            else
                motor.set(0);
        else
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

    public IRTest getIR(){
        return ir;
    }
}
