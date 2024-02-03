package frc.robot.commands.semiauto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import math.MathUtil;

import java.util.ArrayDeque;

public class CenterLimelight extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final ArrayDeque<Double> prevPoses = new ArrayDeque<Double>();

    // TODO: move pid into Constants.java
    private final PIDController rotationPID = new PIDController(0.375, 0.0002, 0.03);

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
    }

    @Override
    public void execute() {
        acceptInput();

        if(prevPoses.isEmpty()) return;

        double yRotation = prevPoses.stream().mapToDouble(Double::doubleValue).sum() / prevPoses.size();

        double yRtSpd = rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * 2.3;

        swerveDrive.setRawMovement(new ChassisSpeeds(0, 0, yRtSpd));
    }

    private void acceptInput() {
        double angle = -Math.toRadians(LimeLight.getTx());

        if(angle == 0) return;

        if(prevPoses.size() >= Constants.CENTER_LIMELIGHT_AVERAGING_WINDOW_LENGTH) {
            prevPoses.removeFirst();
        }
        prevPoses.addLast(angle);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}
