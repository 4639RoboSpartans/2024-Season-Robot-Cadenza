package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILimelightSubsystem extends Subsystem{
    public double getDistZ(int ID);
    public double getDegreesY(int ID);
    public double getDegreesX(int ID);
}
