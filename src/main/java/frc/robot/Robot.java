// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.autos.AutoFactory;
import frc.robot.led.LEDPattern;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.tuning.RobotConfiguration;

public class Robot extends TimedRobot {
    private static boolean isAuton = false;
    private static boolean isDisabled = false;
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    static {
        RobotConfiguration.loadFile("config/shooter.rcfg");
    }

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

        LimeLight.writeValuesToSmartDashboard();
        SmartDashboard.putBoolean("auton?", isAuton);
    }

    @Override
    public void disabledInit() {
        isDisabled = true;
    }

    @Override
    public void disabledPeriodic() {

        var alliance = DriverStation.getAlliance();

        LEDPattern pattern = alliance.<LEDPattern>map(value -> switch (value) {
            case Red -> (led, time) -> {
                double x = led * 0.1 + time * 3;
                double h = 8 * Math.pow(Math.sin(x / 2), 2);
                return new Color8Bit(Color.fromHSV((int) h, 255, 255));
            };
            case Blue -> (led, time) -> {
                time *= 2;
                double x = led * 0.2 + time * 3;
                double h = 20 * Math.pow(Math.sin(x), 2) + 90;
                double v = Math.pow(Math.sin(time), 2) * 0.9 + 0.1;

                return new Color8Bit(Color.fromHSV((int) (h), 255, (int) (255 * v)));
            };
        }).orElse(LEDPattern.BLANK);

        SubsystemManager.getLedStrip().usePattern(pattern);
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
            CommandScheduler.getInstance().schedule(autonomousCommand);
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
