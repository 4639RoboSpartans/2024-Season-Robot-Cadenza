package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.VelocityDutyCycleEncoder;
import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.IntakeInfo;
import frc.robot.tuning.RobotConfiguration;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotInfo.IntakeInfo;

public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;
    private final DutyCycleEncoder encoder;

    private final ProfiledPIDController pivotPID;

    SysIdRoutine routine;

    public IntakeSubsystem() {
        pivotMotorLeft = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IDs.INTAKE_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        pivotMotorLeft.restoreFactoryDefaults();
        pivotMotorRight.restoreFactoryDefaults();

        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotMotorLeft.setSmartCurrentLimit(40);
        pivotMotorRight.setSmartCurrentLimit(40);

        pivotPID = new ProfiledPIDController(
                IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.kp(),
                IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.ki(),
                IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.kd(),
                new TrapezoidProfile.Constraints(
                        2, 2
                )
        );

        encoder = new DutyCycleEncoder(IDs.INTAKE_ENCODER_DIO_PORT);
        encoder.setDutyCycleRange(-2, 2);

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(5), Seconds.of(100), null),
                new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                    pivotMotorLeft.setVoltage(volts.in(Volts));
                    pivotMotorRight.setVoltage(volts.in(Volts));
                },
                        log -> {
                            log.motor("intake-pivot")
                                    .voltage(Volts.of(pivotMotorLeft.get() * RobotController.getBatteryVoltage()))
                                    .angularPosition(Rotations.of(getCurrentAngle()));
                        }, this)
        );

        pivotMotorRight.setInverted(true);
        setExtended(ExtensionState.RETRACTED);
    }

    @Override
    public Command setExtended(ExtensionState state) {
        return runOnce(() -> setExtendedState(state));
    }

    private void setExtendedState(ExtensionState extended) {
        pivotPID.setGoal(switch (extended) {
            case EXTENDED -> IntakeInfo.INTAKE_PIVOT_EXTENDED_SETPOINT;
            case RETRACTED -> IntakeInfo.INTAKE_PIVOT_DEFAULT_SETPOINT;
            case AMP -> IntakeInfo.INTAKE_PIVOT_AMP_SETPOINT;
        });
    }

    @Override
    public Command outtake() {
        return runOnce(this::outtakeRun);
    }

    @Override
    public Command amp() {
        return runOnce(this::ampRun);
    }

    @Override
    public Command stopIntake() {
        return runOnce(this::stopIntakeRun);
    }

    @Override
    public Command stop() {
        return runOnce(this::stopRun);
    }

    @Override
    public Command intake() {
        return runOnce(this::intakeRun);
    }

    private void outtakeRun() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    private void ampRun() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    private void stopIntakeRun() {
        intakeMotor.set(0);
    }

    private void stopRun() {
        intakeMotor.set(0);
        pivotPID.setGoal(getCurrentAngle());
    }

    private void intakeRun() {
        intakeMotor.set(IntakeInfo.INTAKE_SPEED);
    }

    public double getCurrentAngle() {
        return ((encoder.getAbsolutePosition() + 0.15) % 1 + 1) % 1;
    }

    @Override
    public void periodic(){

        double pidOutput = -pivotPID.calculate(getCurrentAngle());
        
        SmartDashboard.putNumber("Intake Pivot Angle", getCurrentAngle());
        SmartDashboard.putNumber("Intake Pivot Goal", pivotPID.getGoal().position);
        SmartDashboard.putNumber("Intake Pivot Setpoint", pivotPID.getSetpoint().position);
        SmartDashboard.putNumber("Intake PID output", pidOutput);
        
        pivotMotorRight.set(pidOutput);
        pivotMotorLeft.set(pidOutput);

        if(Robot.getDisabled()) {
            pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kCoast);
            pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kCoast);
        }
        else {
            pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
            pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        }
    }
}