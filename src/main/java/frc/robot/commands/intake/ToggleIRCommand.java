package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ToggleIRCommand extends Command{
    @Override
    public void initialize(){
        Constants.RobotInfo.HopperInfo.usingIRSensor = !Constants.RobotInfo.HopperInfo.usingIRSensor;
    }
}
