package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SimClimberSubsystem extends ClimberSubsystem {
    private final DCMotorSim simClimberLeft, simClimberRight;
    private final PIDController leftPID, rightPID;
    private double leftPosition, rightPosition, leftDown, rightDown;

    private MechanismRoot2d leftRoot, rightRoot;
    private MechanismLigament2d leftClimber, rightClimber;

    public SimClimberSubsystem() {
        simClimberLeft = new DCMotorSim(DCMotor.getNEO(1), 1 / 81.0, 0.3);
        simClimberRight = new DCMotorSim(DCMotor.getNEO(1), 1 / 81.0, 0.3);

        leftPID = ClimberConstants.positionController.create();
        rightPID = ClimberConstants.positionController.create();

        leftPosition = simClimberLeft.getAngularPositionRotations();
        rightPosition = simClimberRight.getAngularPositionRotations();

        leftDown = leftPosition;
        rightDown = rightPosition;
    }

    private double getLeftPosition() {
        return simClimberLeft.getAngularPositionRotations();
    }

    private double getRightPosition() {
        return simClimberRight.getAngularPositionRotations();
    }

    @Override
    protected void runPID() {
        double PIDLeft = leftPID.calculate(getLeftPosition());
        double PIDRight = rightPID.calculate(getRightPosition());
        simClimberLeft.setInputVoltage(PIDLeft);
        simClimberRight.setInputVoltage(PIDRight);
    }

    @Override
    protected void runUp() {
        simClimberLeft.setInputVoltage(ClimberConstants.CLIMBER_SPEED);
        simClimberRight.setInputVoltage(ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    protected void runDown() {
        simClimberRight.setInputVoltage(-ClimberConstants.CLIMBER_SPEED);
        simClimberRight.setInputVoltage(-ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    protected void runSwapLeftUp() {
        simClimberLeft.setInputVoltage(ClimberConstants.CLIMBER_SPEED);
        simClimberRight.setInputVoltage(-ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    protected void runSwapRightUp() {
        simClimberLeft.setInputVoltage(-ClimberConstants.CLIMBER_SPEED);
        simClimberRight.setInputVoltage(ClimberConstants.CLIMBER_SPEED);
    }

    @Override
    public void transitionToPIDControl() {
        leftPosition = simClimberLeft.getAngularPositionRotations();
        rightPosition = simClimberRight.getAngularPositionRotations();
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
        leftClimber.setLength((getLeftPosition() - leftDown) + 0.75);
        rightClimber.setLength((getRightPosition() - rightDown) + 0.75);
    }
}
