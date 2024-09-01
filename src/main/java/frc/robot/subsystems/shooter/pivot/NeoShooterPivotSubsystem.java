package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.util.AimUtil;

import static frc.robot.constants.RobotInfo.ShooterInfo;

public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    // Components
    private final CANSparkMax aimMotorLeft;
    private final CANSparkMax aimMotorRight;
    
    private final DutyCycleEncoder encoder;
    // References to other subsystems
    private final IShooterSubsystem shooter;
    // Control
    private final PIDController aimPID;
    private boolean isUsingPID = true;

    public NeoShooterPivotSubsystem(int aimMotorLeftID, int aimMotorRightID, IShooterSubsystem shooter) {
        aimMotorLeft = new CANSparkMax(aimMotorLeftID, CANSparkMax.MotorType.kBrushless);
        aimMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        encoder = new DutyCycleEncoder(IDs.SHOOTER_PIVOT_ENCODER_DIO_PORT);
        
        aimMotorRight = new CANSparkMax(aimMotorRightID, CANSparkMax.MotorType.kBrushless);
        aimMotorRight.follow(aimMotorLeft, true);

        this.shooter = shooter;
        
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
            case AUTO_SPEAKER -> AimUtil.getVelocityCompensatedShooterSetpoint().angle();
            case SPEAKER -> ShooterInfo.SHOOTER_SPEAKER_SETPOINT.angle();
            case AMP -> ShooterInfo.SHOOTER_AMP_SETPOINT.angle();
            case TRAP -> ShooterInfo.SHOOTER_TRAP_SETPOINT.angle();
            case IDLE -> ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT;
            case LAUNCH -> ShooterInfo.SHOOTER_LAUNCH_SETPOINT.angle();
            case INTAKE -> ShooterInfo.SHOOTER_INTAKE_SETPOINT.angle();
        } + ShooterInfo.AngleOffset;

        aimPID.setSetpoint(targetAngle);

        double currentAngle = getCurrentAngle();
        double pidOutput = aimPID.calculate(currentAngle);

        aimMotorLeft.set(pidOutput);
         SmartDashboard.putNumber("TargetShooterAngle", targetAngle);
         SmartDashboard.putNumber("AimPIDOutput", pidOutput);
    }

    public double getCurrentAngle() {
        return encoder.getAbsolutePosition();
    }

    public void stop(){
        aimMotorLeft.stopMotor();
    }
}
