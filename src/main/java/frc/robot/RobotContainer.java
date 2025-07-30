// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.controlConstants;
import frc.robot.commands.DriveTrain.SwerveJoystick;
import frc.robot.commands.Elevator.ElevatorManual;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FalconFlare;
import frc.robot.subsystems.DriveTrain.SwerveSubsystem;

public class RobotContainer { 
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Elevator elevator = new Elevator();
  private final FalconFlare flare = new FalconFlare();
  private final CommandXboxController driverControler = new CommandXboxController(0);
  private final CommandXboxController operatorControler = new CommandXboxController(1);

  public RobotContainer() {
    System.out.println("robot start");
    swerve.zeroHeading();
    elevator.resetEncoders();
    flare.setLights("white");
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driverControler.getLeftY(), 
      () -> -driverControler.getLeftX(), 
      () -> -driverControler.getRightX(), 
      () -> !driverControler.getHID().getLeftBumper()));
    configureBindings();
  }

  private void configureBindings() {
    operatorControler.axisMagnitudeGreaterThan(5, controlConstants.operatorRSDeadZone).whileTrue(new ElevatorManual(elevator, () -> (-operatorControler.getRightY() + 0.03)*0.2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
