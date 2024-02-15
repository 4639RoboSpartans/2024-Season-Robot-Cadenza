package frc.robot.subsystems.shooterPivot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    private final CANSparkMax aimMotor;
    private final RelativeEncoder encoder;
    private final PIDController aimPID;
    private boolean isUsingPID = true;

    public NeoShooterPivotSubsystem(int aimMotorID) {
        aimMotor = new CANSparkMax(aimMotorID, CANSparkMax.MotorType.kBrushless);
        aimMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        encoder = aimMotor.getEncoder();
        aimPID = Constants.RobotInfo.SHOOTER_AIM_PID.create();
    }

    public void setAngleDegrees(double degrees) {
        aimPID.setSetpoint(degrees);
        isUsingPID = true;
    }

    public void manualSet(double speed){
        aimMotor.set(speed);
        isUsingPID = false;
    }

    @Override
    public void periodic() {
        if(!isUsingPID) return;

        double currentAimMotorDegrees = encoder.getPosition();
        aimMotor.set(aimPID.calculate(currentAimMotorDegrees));
    }

    public void stop(){
        aimMotor.stopMotor();
    }
}
