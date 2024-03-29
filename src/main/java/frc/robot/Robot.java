// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        LimeLight.writeValuesToSmartDashboard();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        ((SwerveDriveSubsystem) SubsystemManager.getSwerveDrive()).useAutonCurrentLimits();

        if (autonomousCommand != null) {
            new WaitCommand(robotContainer.autonDelay.getSelected())
                .andThen(autonomousCommand)
                .schedule();
        }
    }

    @Override
    public void autonomousExit() {
        if(autonomousCommand != null){
            CommandScheduler.getInstance().removeComposedCommand(autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        ((SwerveDriveSubsystem) SubsystemManager.getSwerveDrive()).useTeleopCurrentLimits();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
