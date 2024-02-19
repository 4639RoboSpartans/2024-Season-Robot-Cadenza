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


public class AimSubsystem extends SubsystemBase{

    private final PIDController rotationPID = AimInfo.LIMELIGHT_AIM_PID.create();
    private final OI oi;

    private final ArrayDeque<Double> prevPoses = new ArrayDeque<>();
    public AimSubsystem(OI oi){
        this.oi = oi;
    }

    public double getRotation(){
       if (prevPoses.isEmpty())
           return -oi.driverController().getAxis(Constants.Controls.DriverControls.SwerveRotationAxis);

       double yRotation = prevPoses.stream().mapToDouble(x -> (double) x).sum() / prevPoses.size();
       double yRtSpd = rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * SwerveInfo.MAX_ROBOT_SPEED;
       return yRtSpd;
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

    public void updateKD(){
        double kD = 1/Math.pow(Math.abs(SmartDashboard.getNumber("AprilTag: y rotation", 1)), 2) * 7;
        rotationPID.setD(kD);
    }
}
