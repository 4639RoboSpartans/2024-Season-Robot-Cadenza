package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import static frc.robot.constants.RobotInfo.IntakeInfo;

public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;
    private final RelativeEncoder leftEncoder;
    private final double downPosition, upPosition;

    private final PIDController pivotPID;

    public IntakeSubsystem(int pivotMotorLeftID, int pivotMotorRightID, int intakeMotorID, int encoderID) {
        pivotMotorLeft = new CANSparkMax(pivotMotorLeftID, CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(pivotMotorRightID, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorID, CANSparkMax.MotorType.kBrushless);

        pivotPID = IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.create();

        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotMotorLeft.setInverted(false);
        pivotMotorRight.follow(pivotMotorLeft, true);

        leftEncoder = pivotMotorLeft.getEncoder();
        downPosition = leftEncoder.getPosition();
        upPosition = downPosition - 55;
        setExtended(ExtensionState.RETRACTED);
    }

    public void setExtended(ExtensionState extended) {
        pivotPID.setSetpoint(switch (extended){
            case EXTENDED -> downPosition;
            case RETRACTED -> upPosition;
        });
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(IntakeInfo.INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        double pidOutput = -pivotPID.calculate(getPosition());
        SmartDashboard.putNumber("down postion", downPosition);
        SmartDashboard.putNumber("up position", upPosition);
        SmartDashboard.putNumber("pivot encoder angle", getPosition());

        if(pidOutput > 0) pidOutput *= Constants.INTAKE_PIVOT_UP_MULTIPLIER;

        // SmartDashboard.putNumber("target pivot pos", pivotPID.getSetpoint());
        // SmartDashboard.putNumber("current pivot pos", getPosition());
        // SmartDashboard.putNumber("pivot pid output", pidOutput);

        // pivotMotorLeft.set(pidOutput);

        // SmartDashboard.putNumber("intake pivot output", pivotMotorRight.getAppliedOutput());
    }

    public void stop() {
        pivotPID.setSetpoint(getPosition());

        pivotMotorLeft.stopMotor();
        pivotMotorRight.stopMotor();
        intakeMotor.stopMotor();
    }

    private double getPosition() {
        return leftEncoder.getPosition();
    }
}
