// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.tuning.RobotConfiguration;

public class Robot extends TimedRobot {
    private static boolean isAuton = false;
    private static boolean isDisabled = false;
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    public static boolean isInAuton() {
        return isAuton;
    }

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        RobotConfiguration.updateAll();
        SmartDashboard.putBoolean("auton?", isAuton);

        robotContainer.sendSubsystems();
    }

    @Override
    public void disabledInit() {
        isDisabled = true;
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        isDisabled = false;
    }

    public static boolean getDisabled() {
        return isDisabled;
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            SmartDashboard.putString("Auton selected", autonomousCommand.getName());
            CommandScheduler.getInstance().schedule(
                Commands.sequence(robotContainer.getDelay(), autonomousCommand)
            );
        }
        isAuton = true;
    }

    @Override
    public void autonomousExit() {
        isAuton = false;
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().removeComposedCommand(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        isAuton = false;
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
