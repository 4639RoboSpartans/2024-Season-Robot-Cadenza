package frc.robot.subsystems.swerve;

import java.util.ArrayDeque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.oi.OI;
import math.MathUtil;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class AimSubsystem extends SubsystemBase{

    private final PIDController rotationPID = AimInfo.LIMELIGHT_AIM_PID.create();

    private final ArrayDeque<Double> prevPoses = new ArrayDeque<>();
    public AimSubsystem(){}

    public double getRotationSpeed(){
        if (prevPoses.isEmpty()) return 0;

        double yRotation = prevPoses.stream().mapToDouble(Double::doubleValue).sum() / prevPoses.size();

        return rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * SwerveInfo.MAX_ROBOT_SPEED;
    }

    @Override
    public void periodic() {
        double angle = -Math.toRadians(LimeLight.getTx());

        if(prevPoses.size() >= Constants.CENTER_LIMELIGHT_AVERAGING_WINDOW_LENGTH) {
            prevPoses.removeFirst();
        }
        prevPoses.addLast(angle);
    }

    public void resetPID(){
        rotationPID.reset();
    }
}
