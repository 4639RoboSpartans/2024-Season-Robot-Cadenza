package frc.robot.subsystems.shooterPivot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class FalconShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    private final TalonFX aimMotor;

    private final PIDController aimPID;

    public FalconShooterPivotSubsystem(int aimMotorID) {
        aimMotor = new TalonFX(aimMotorID);

        aimPID = Constants.RobotInfo.SHOOTER_AIM_PID.create();
    }

    public void setAngleDegrees(double degrees) {
        aimPID.setSetpoint(degrees);
    }

    @Override
    public void periodic() {
        double currentAimMotorDegrees = aimMotor.getPosition().getValue();
        aimMotor.set(aimPID.calculate(currentAimMotorDegrees));
    }

    public void stop(){
        aimMotor.stopMotor();
    }
}
