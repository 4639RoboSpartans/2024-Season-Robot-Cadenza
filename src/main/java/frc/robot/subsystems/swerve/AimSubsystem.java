package frc.robot.subsystems.swerve;

import java.util.ArrayDeque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.oi.OI;
import math.MathUtil;

public class AimSubsystem extends SubsystemBase{
    private final OI oi;

    private final PIDController rotationPID = Constants.RobotInfo.LIMELIGHT_AIM_PID.create();

    private final ArrayDeque<Double> prevPoses = new ArrayDeque<>();
    public AimSubsystem(OI oi){
        this.oi = oi;
    }

    public double getRotation(){
//        acceptInput();
//
//        if (prevPoses.isEmpty())
//            return -oi.getDriverController().getAxis(Constants.Controls.Driver.SwerveRotationAxis);
//
//        double yRotation = prevPoses.stream().mapToDouble(RobotPose::yRotation).sum() / prevPoses.size();
//        double yRtSpd = rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * Constants.RobotInfo.MAX_ROBOT_SPEED;
//        return yRtSpd;
        return 0;
    }

    private void acceptInput() {

//        double angle = -Math.toRadians(LimeLight.getTx());
//
//        if(prevPoses.size() >= Constants.CENTER_LIMELIGHT_AVERAGING_WINDOW_LENGTH) {
//            prevPoses.removeFirst();
//        }
//        prevPoses.addLast(new RobotPose(angle));
    }

    public void resetPID(){
        rotationPID.reset();
    }
}
