package frc.robot.commands.semiauto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import math.MathUtil;

import java.util.ArrayDeque;

record RobotPose(double xOffset, double zOffset, double yRotation) {}

public class CenterLimelight extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final ArrayDeque<RobotPose> prevPoses = new ArrayDeque<>();
    
    private PIDController rotationPID = Constants.RobotInfo.ROTATION_PID.create();

    public CenterLimelight(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDrive = swerveDriveSubsystem;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDrive.stop();
        prevPoses.clear();
        rotationPID.reset();
        rotationPID.setSetpoint(0);
        SmartDashboard.putNumber("rotatorPID kD", 0);
    }

    @Override
    public void execute() {
        acceptInput();

        if(prevPoses.isEmpty()) return;

        double yRotation = prevPoses.stream().mapToDouble(RobotPose::yRotation).sum() / prevPoses.size();

        double yRtSpd = rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * Constants.RobotInfo.MAX_ROBOT_SPEED;

        swerveDrive.setRawMovement(new ChassisSpeeds(0, 0, yRtSpd));

        double kD = 1/Math.pow(Math.abs(SmartDashboard.getNumber("AprilTag: y rotation", 1)), 2) * 7;
        if (kD > 10) kD = 0.01;

        rotationPID.setD(kD);
        SmartDashboard.putNumber("rotatorPID kD", rotationPID.getD());
    }

    private void acceptInput() {
        double xOffset = LimeLight.getXDistance();
        double zOffset = LimeLight.getZDistance() - Constants.FieldDistances.ShooterApriltagZDistance;
//        double yRotation = LimeLight.getYRotation();

        double angle = -Math.toRadians(LimeLight.getTx());

        if(xOffset == 0) return;

        if(prevPoses.size() >= Constants.CENTER_LIMELIGHT_AVERAGING_WINDOW_LENGTH) {
            prevPoses.removeFirst();
        }
        prevPoses.addLast(new RobotPose(xOffset, zOffset, angle));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}