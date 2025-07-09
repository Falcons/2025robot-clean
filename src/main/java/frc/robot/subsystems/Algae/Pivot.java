// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.DeviceIDs.AlgaeIDs;

public class Pivot extends SubsystemBase {
  private final SparkMax motor;
  private SparkMaxConfig motorConfig;
  PIDController pivotPid = new PIDController(0.3, 0, 0);
  /** Creates a new Pivot. */
  public Pivot() {
    //setup pivot motor
    motor = new SparkMax(AlgaeIDs.pivotMotor, SparkMax.MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(30);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.encoder.positionConversionFactor(AlgaeConstants.pivotMotorRotToRad);
    motorConfig.encoder.velocityConversionFactor(AlgaeConstants.pivotMotorRotToRad/60);
    motor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
