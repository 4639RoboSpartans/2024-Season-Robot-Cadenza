package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.sensors.IRSensor;

import static frc.robot.constants.RobotInfo.HopperInfo;

public class HopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax motor;
    private final IRSensor ir;

    public HopperSubsystem(int motorID) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.ir = SubsystemManager.getIRSensor();
    }

    @Override
    public void run(boolean checkNote, double speed) {
        if (ir.hasNote() && checkNote) {
            motor.set(0);
        } else {
            motor.set(speed);
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

    public IRSensor getIR(){
        return ir;
    }
}
