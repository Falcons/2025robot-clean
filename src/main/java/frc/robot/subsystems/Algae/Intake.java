// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.DeviceIDs.AlgaeIDs;

public class Intake extends SubsystemBase {
  private final SparkMax motor;
  private SparkMaxConfig motorConfig;
  /** Creates a new Intake. */
  public Intake() {
    //setup intake motor
    motor = new SparkMax(AlgaeIDs.intakeMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(30);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.encoder.positionConversionFactor(AlgaeConstants.intakeMotorRotToRad);
    motorConfig.encoder.velocityConversionFactor(AlgaeConstants.intakeMotorRotToRad/60);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
