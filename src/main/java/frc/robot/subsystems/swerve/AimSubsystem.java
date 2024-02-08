package frc.robot.subsystems.swerve;

import java.util.ArrayDeque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.oi.OI;
import math.MathUtil;

record RobotPose(double yRotation) {}

public class AimSubsystem extends SubsystemBase{
    private final OI oi;
    private final PIDController rotationPID = Constants.RobotInfo.ROTATION_PID.create();
    private final ArrayDeque<RobotPose> prevPoses = new ArrayDeque<>();
    public AimSubsystem(OI oi){
        this.oi = oi;
    }


    public double getRotation(){
        acceptInput();
        
        if (prevPoses.isEmpty()) 
            return -oi.getDriverController().getAxis(Constants.Controls.SwerveRotationAxis);

        double yRotation = prevPoses.stream().mapToDouble(RobotPose::yRotation).sum() / prevPoses.size();
        double yRtSpd = rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * Constants.RobotInfo.MAX_ROBOT_SPEED;
        return yRtSpd;
    }

    private void acceptInput() {

        double angle = -Math.toRadians(LimeLight.getTx());

        if(prevPoses.size() >= Constants.CENTER_LIMELIGHT_AVERAGING_WINDOW_LENGTH) {
            prevPoses.removeFirst();
        }
        prevPoses.addLast(new RobotPose(angle));
    }

    public void resetPID(){
        rotationPID.reset();
    }
}
