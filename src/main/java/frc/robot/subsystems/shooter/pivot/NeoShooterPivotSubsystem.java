package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotInfo.AimInfo.LIMELIGHT_STATUS;
import frc.robot.Constants.RobotInfo.ShooterInfo.SHOOTING_MODE;
import frc.robot.subsystems.swerve.AimSubsystem;

import static frc.robot.Constants.RobotInfo.ShooterInfo;

@SuppressWarnings("unused")
public class NeoShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    private final CANSparkMax aimMotor;
    private final DutyCycleEncoder encoder;
    private final PIDController aimPID;
    private final AimSubsystem aimSubsystem;
    private boolean isUsingPID = true;
    private boolean atSetPoint = false;
    private SHOOTING_MODE speakerShooting = SHOOTING_MODE.SPEAKER;
    private LIMELIGHT_STATUS isUsingLimeLight = LIMELIGHT_STATUS.LIMELIGHT;

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

    public boolean isAtSetPoint(){
        return atSetPoint;
    }

    public void setShooting(SHOOTING_MODE shooting){
        speakerShooting = shooting;
    }

    public void setManual(LIMELIGHT_STATUS manual){
        isUsingLimeLight = manual;
    }

    @Override
    public void periodic() {
        if(!isUsingPID) return;


        double targetAngle;
        if(!(isUsingLimeLight == LIMELIGHT_STATUS.LIMELIGHT)){
            targetAngle = Constants.RobotInfo.ShooterInfo.SHOOTER_PIVOT_SPEAKER_SETPOINT;
        }
        else if(speakerShooting == SHOOTING_MODE.SPEAKER){
            ShooterInfo.ShooterSetpoint setpoint = aimSubsystem.getShooterSetpoint();
            targetAngle = setpoint.angle();
        }        
        else {
            targetAngle = Constants.RobotInfo.ShooterInfo.SHOOTER_PIVOT_AMP_SETPOINT;
        }
        aimPID.setSetpoint(targetAngle);

        double error = Math.abs(targetAngle - encoder.getAbsolutePosition());
        if (error < 0.01){
            atSetPoint = true;
        }

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
