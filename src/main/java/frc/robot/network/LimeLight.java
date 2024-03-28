package frc.robot.network;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.constants.RobotInfo.ShooterInfo;

public class LimeLight {
    private LimeLight() {}

    public static void writeValuesToSmartDashboard() {
        SmartDashboard.putNumber("AprilTag: x distance", getXDistance());
        SmartDashboard.putNumber("AprilTag: y distance", getYDistance());
        SmartDashboard.putNumber("AprilTag: z distance", getZDistance());
        SmartDashboard.putNumber("AprilTag: x rotation", getXRotation());
        SmartDashboard.putNumber("AprilTag: y rotation", getYRotation());
        SmartDashboard.putNumber("AprilTag: z rotation", getZRotation());

        SmartDashboard.putNumber("AprilTag: tx", Network.getTable("limelight").getDouble("tx"));
        SmartDashboard.putNumber("AprilTag: ty", Network.getTable("limelight").getDouble("ty"));
    }

    private static double getOrNaN(double[] arr, int idx){
        return arr.length > idx ? arr[idx] : Double.NaN;
    }

    public static double getTx() {
        return Network.getTable("limelight").getDouble("tx") + ShooterInfo.LimelightTxOffset;
    }

    public static double getTy() {
        return Network.getTable("limelight").getDouble("ty");
    }

    public static double getXDistance() {
        double rawDistance =  getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 0);
        return rawDistance + ShooterInfo.LimelightOffsetX;
    }
    public static double getYDistance() {
        double rawDistance =  getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 1);
        return rawDistance + ShooterInfo.LimelightOffsetY;
    }
    public static double getZDistance() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 2);
    }
    public static double getXRotation() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 3);
    }
    public static double getYRotation() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 4);
    }
    public static double getZRotation() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 5);
    }

    public static double getRobotRelativeXDistance() {
        double rawDistance =  getOrNaN(Network.getTable("limelight").getDoubleArray("targetpose_robotspace"), 0);
        return rawDistance + ShooterInfo.LimelightOffsetX;
    }
    public static double getRobotRelativeYDistance() {
        double rawDistance =  getOrNaN(Network.getTable("limelight").getDoubleArray("targetpose_robotspace"), 1);
        return rawDistance + ShooterInfo.LimelightOffsetY;
    }
    public static double getRobotRelativeZDistance() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("targetpose_robotspace"), 2);
    }
}