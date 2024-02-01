package frc.robot.network;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
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

    public static double getZDistance() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 0);
    }
    public static double getXDistance() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 1);
    }
    public static double getYDistance() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 2);
    }
    public static double getZRotation() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 3);
    }
    public static double getXRotation() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 4);
    }
    public static double getYRotation() {
        return getOrNaN(Network.getTable("limelight").getDoubleArray("camerapose_targetspace"), 5);
    }
}