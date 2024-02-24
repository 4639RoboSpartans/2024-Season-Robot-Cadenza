package frc.robot.subsystems.shooterPivot;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;
import frc.robot.subsystems.swerve.AimSubsystem;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    private final CANSparkMax aimMotor;
    private final DutyCycleEncoder encoder;
    private final PIDController aimPID;
    private final AimSubsystem aimSubsystem;
    private boolean isUsingPID = true;

    public NeoShooterPivotSubsystem(int aimMotorID, AimSubsystem aimSubsystem) {
        aimMotor = new CANSparkMax(aimMotorID, CANSparkMax.MotorType.kBrushless);
        aimMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        encoder = new DutyCycleEncoder(Constants.IDs.SHOOTER_PIVOT_ENCODER_CHANNEL);
        aimPID = ShooterInfo.SHOOTER_AIM_PID.create();
        aimPID.setSetpoint(ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);
        this.aimSubsystem = aimSubsystem;
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


        ShooterInfo.ShooterSetpoint setpoint = aimSubsystem.getShooterSetpoint();
        double targetAngle = setpoint.angle();
        aimPID.setSetpoint(targetAngle);

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
