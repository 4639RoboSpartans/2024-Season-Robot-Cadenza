package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.aim.AimSubsystem;

import static frc.robot.constants.Constants.RobotInfo.ShooterInfo;

public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    // Components
    private final CANSparkMax aimMotor;
    private final DutyCycleEncoder encoder;
    // References to other subsystems
    private final IShooterSubsystem shooter;
    private final AimSubsystem aimSubsystem;
    // Control
    private final PIDController aimPID;
    private boolean isUsingPID = true;

    public NeoShooterPivotSubsystem(int aimMotorID, IShooterSubsystem shooter) {
        aimMotor = new CANSparkMax(aimMotorID, CANSparkMax.MotorType.kBrushless);
        aimMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        encoder = new DutyCycleEncoder(Constants.IDs.SHOOTER_PIVOT_ENCODER_CHANNEL);
        
        this.shooter = shooter;
        aimSubsystem = SubsystemManager.getAimSubsystem();
        
        aimPID = ShooterInfo.SHOOTER_AIM_PID_CONSTANTS.create();
        aimPID.setSetpoint(ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);
    }

    public boolean isAtSetPoint(){
        double error = Math.abs(aimPID.getSetpoint() - encoder.getAbsolutePosition());
        return error < ShooterInfo.SHOOTER_PIVOT_ERROR;
    }

    @Override
    public void periodic() {
        if(!isUsingPID) return;

        double targetAngle = switch (shooter.getShootingMode()) {
            case AUTO_SPEAKER -> aimSubsystem.getShooterSetpoint().angle();
            case SPEAKER -> ShooterInfo.SHOOTER_SPEAKER_SETPOINT.angle();
            case AMP -> ShooterInfo.SHOOTER_AMP_SETPOINT.angle();
            case TRAP -> ShooterInfo.SHOOTER_TRAP_SETPOINT.angle();
            case IDLE -> ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT;
        };

        aimPID.setSetpoint(targetAngle);

        double currentAngle = getCurrentAngle();
        double pidOutput = aimPID.calculate(currentAngle);

        aimMotor.set(pidOutput);

        SmartDashboard.putNumber("CurrentShooterAngle", currentAngle);
        SmartDashboard.putNumber("TargetShooterAngle", targetAngle);
        SmartDashboard.putNumber("AimPIDOutput", pidOutput);
    }

    private double getCurrentAngle() {
        return encoder.getAbsolutePosition();
    }

    public void stop(){
        aimMotor.stopMotor();
    }
}
