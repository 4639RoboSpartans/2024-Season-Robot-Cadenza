package frc.robot.subsystems.shooterPivot;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    private final CANSparkMax aimMotor;
    private final DutyCycleEncoder encoder;
    private final PIDController aimPID;
    private boolean isUsingPID = true;

    public NeoShooterPivotSubsystem(int aimMotorID) {
        aimMotor = new CANSparkMax(aimMotorID, CANSparkMax.MotorType.kBrushless);
        aimMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        encoder = new DutyCycleEncoder(0);
        aimPID = ShooterInfo.SHOOTER_AIM_PID.create();
        aimPID.setSetpoint(ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);
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

        double currentAimMotorDegrees = encoder.getAbsolutePosition();
        double pidOutput = aimPID.calculate(currentAimMotorDegrees);
        aimMotor.set(pidOutput);

        SmartDashboard.putNumber("CurrentShooterAngle", currentAimMotorDegrees);
        SmartDashboard.putNumber("AimPIDOutput", pidOutput);
    }

    public void stop(){
        aimMotor.stopMotor();
    }
}
