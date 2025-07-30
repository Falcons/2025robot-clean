// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllModuleX extends ParallelCommandGroup {
  SwerveSubsystem swerve;
  /** Creates a new AllModuleX. */
  public AllModuleX(SwerveSubsystem swerve) {
    addRequirements(swerve);
    addCommands(
      // swerve.moduleRawSetpoint("Front Left", 0),
      // swerve.moduleRawSetpoint("Front Right", 0),
      // swerve.moduleRawSetpoint("Back Left", 0),
      // swerve.moduleRawSetpoint("Back Right", 0),
      swerve.modulePIDTuning("Front Left"),
      swerve.modulePIDTuning("Front Right"),
      swerve.modulePIDTuning("Back Left"),
      swerve.modulePIDTuning("Back Right")
    );
  }
}
