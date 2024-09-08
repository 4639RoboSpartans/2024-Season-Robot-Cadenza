package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;

import static frc.robot.constants.RobotInfo.IntakeInfo;

public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;
    private final RelativeEncoder encoder;
    private double downPosition, upPosition, ampPosition;
    private ExtensionState state;

    private final PIDController pivotPID;

    public IntakeSubsystem() {
        pivotMotorLeft = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IDs.INTAKE_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        pivotPID = IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.create();

        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotMotorRight.setInverted(true);
        pivotMotorLeft.follow(pivotMotorRight, true);

        encoder = pivotMotorRight.getEncoder();
        downPosition = encoder.getPosition();
        upPosition = downPosition - 55;
        ampPosition = downPosition - 50;
        state = ExtensionState.RETRACTED;
    }

    public void setExtended(ExtensionState extended) {
        state = extended;
    }

    public void manualExtend() {
        state = ExtensionState.MANUAL;
        pivotMotorRight.set(Math.abs(SubsystemManager.getOI().operatorController().getAxis(OI.Axes.LEFT_STICK_Y) / 2));
    }

    public void updateOffset() {
      downPosition = encoder.getPosition();
      upPosition = downPosition - 55;
      ampPosition = downPosition - 50;
      state = ExtensionState.RETRACTED;
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(IntakeInfo.INTAKE_SPEED);
    }

    public void amp() {
      intakeMotor.set(IntakeInfo.AMP_OUTTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        pivotPID.setSetpoint(switch (state) {
            case RETRACTED -> upPosition;
            case EXTENDED -> downPosition;
            case MANUAL -> downPosition;
            case AMP -> ampPosition;
        });
        double pidOutput = pivotPID.calculate(getPosition());
        SmartDashboard.putString("Intake/Intake position", state.toString());
        SmartDashboard.putNumber("Intake/Intake angle", getPosition());
        SmartDashboard.putNumber("Intake/Intake PID output", pidOutput);

        if (state != ExtensionState.MANUAL)
          pivotMotorRight.set(pidOutput);
    }

    public void stop() {
        pivotPID.setSetpoint(getPosition());

        pivotMotorLeft.stopMotor();
        pivotMotorRight.stopMotor();
        intakeMotor.stopMotor();
    }

    private double getPosition() {
        return encoder.getPosition();
    }
}