// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.ElevatorIDs;

public class Elevator extends SubsystemBase {
      private final SparkMax leftMoter, rightMoter;
      private SparkMaxConfig leftMoterConfig, rightMoterConfig;

      /** Creates a new Elevator. */
  public Elevator() {
    leftMoter = new SparkMax(ElevatorIDs.leftMotor, MotorType.kBrushless);
    rightMoter = new SparkMax(ElevatorIDs.rightMotor, MotorType.kBrushless);
    rightMoterConfig = new SparkMaxConfig();
    leftMoterConfig = new SparkMaxConfig();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
