package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ConcreteClimberSubsystem extends ClimberSubsystem {
    private final CANSparkMax leftMotor, rightMotor;
    private final PIDController leftPID, rightPID;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private double leftPosition, rightPosition, leftDown, rightDown;

    private MechanismRoot2d leftRoot, rightRoot;
    private MechanismLigament2d leftClimber, rightClimber;

    public ConcreteClimberSubsystem() {
        leftMotor = new CANSparkMax(ClimberConstants.IDs.CLIMBER_LEFT, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.IDs.CLIMBER_RIGHT, CANSparkMax.MotorType.kBrushless);

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        rightMotor.setInverted(true);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftPID = ClimberConstants.positionController.create();
        rightPID = ClimberConstants.positionController.create();

        leftPosition = leftEncoder.getPosition();
        rightPosition = rightEncoder.getPosition();

        leftDown = leftPosition;
        rightDown = rightPosition;
    }

    private double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    private double getRightPosition() {
        return rightEncoder.getPosition();
    }

    @Override
    protected void runPID() {
        double PIDLeft = leftPID.calculate(getLeftPosition());
        double PIDRight = rightPID.calculate(getRightPosition());
        leftMotor.set(PIDLeft);
        rightMotor.set(PIDRight);
    }

    @Override
    protected void runUp() {
        leftMotor.set(ClimberConstants.CLIMBER_SPEED);
        leftMotor.set(ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    protected void runDown() {
        leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
        leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    protected void runSwapLeftUp() {
        leftMotor.set(ClimberConstants.CLIMBER_SPEED);
        leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    protected void runSwapRightUp() {
        leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
        leftMotor.set(ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    public void transitionToPIDControl() {
        leftPosition = leftEncoder.getPosition();
        rightPosition = rightEncoder.getPosition();
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        builder.addDoubleProperty("Left position",
                this::getLeftPosition,
                null);
        builder.addDoubleProperty("Right position",
                this::getRightPosition,
                null);
    }

    @Override
    public void initMech(Mechanism2d mech) {
        leftRoot = mech.getRoot("Left climber", 1.8, 0);
        rightRoot = mech.getRoot("Right climber", 2.2, 0);
        leftClimber = leftRoot.append(
                new MechanismLigament2d(
                        "Left",
                        1,
                        90
                )
        );
        rightClimber = rightRoot.append(
                new MechanismLigament2d(
                        "Right",
                        1,
                        90
                )
        );
    }

    public Command rootMech() {
        return runOnce(() -> {
            leftDown = getLeftPosition();
            rightDown = getRightPosition();
        });
    }

    @Override
    public void periodic() {
        leftClimber.setLength((getLeftPosition() - leftDown) * ClimberConstants.CLIMBER_SCALAR_TO_MECHANISM);
        rightClimber.setLength((getRightPosition() - rightDown) * ClimberConstants.CLIMBER_SCALAR_TO_MECHANISM);
    }
}
