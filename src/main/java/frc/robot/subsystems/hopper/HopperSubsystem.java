package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.sensors.IRSensor;

import static frc.robot.Constants.RobotInfo.HopperInfo;

public class HopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax motor;
    private final IRSensor ir, ir2;

    public HopperSubsystem(int motorID) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.ir = SubsystemManager.getIRSensor();
        this.ir2 = SubsystemManager.getIrSensor2();
    }

    @Override
    public void run(boolean checkNote) {
        if ((ir.hasNote() || ir2.hasNote()) && checkNote) {
            motor.set(0);
        } else {
            motor.set(HopperInfo.HOPPER_SPEED);
        }
    }

    @Override
    public void runBackwards() {
        motor.set(-HopperInfo.HOPPER_SPEED);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public IRSensor getIR(){
        return ir;
    }

    public IRSensor getIR2(){
        return ir2;
    }
}
