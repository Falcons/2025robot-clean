// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.DeviceIDs.ElevatorIDs;

public class Elevator extends SubsystemBase {
      private final SparkMax rightMoter, leftMoter;
      private SparkMaxConfig rightMoterConfig, leftMoterConfig;
      private TimeOfFlight TOF = new TimeOfFlight(ElevatorIDs.TOF);
      public boolean cancelUp, cancelDown; 
      PIDController Pid = new PIDController(0, 0, 0);//kp was 0.7
      ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);

  /** Creates a new Elevator. */
  public Elevator() {
    //setup right motor
    rightMoter = new SparkMax(ElevatorIDs.rightMotor, MotorType.kBrushless);
    rightMoterConfig = new SparkMaxConfig();
    rightMoterConfig.encoder.positionConversionFactor(2);//TODO: update constants to inchs then remove conversion factors
    rightMoterConfig.encoder.velocityConversionFactor(2);
    rightMoterConfig.idleMode(IdleMode.kBrake);
    rightMoterConfig.smartCurrentLimit(40);
    rightMoter.configure(rightMoterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //setup left motor
    leftMoter = new SparkMax(ElevatorIDs.leftMotor, MotorType.kBrushless);
    leftMoterConfig = new SparkMaxConfig();
    leftMoterConfig.apply(rightMoterConfig);
    leftMoterConfig.inverted(true);
    leftMoter.configure(leftMoterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cancelDown = getPosition() <= ElevatorConstants.Min;
    cancelUp = getPosition() >= ElevatorConstants.Max;

    //smart dashboard
    SmartDashboard.putBoolean("Elevator/upwards movement disabled", cancelUp);
    SmartDashboard.putBoolean("Elevator/downwards movement disabled", cancelDown);
    SmartDashboard.putNumber("Elevator/Position", getPosition());
    SmartDashboard.putNumber("Elevator/Velocity", getVelocity());
    //pid tuning, disable once complete
    SmartDashboard.putData(Pid);
    feedforward.setKs(SmartDashboard.getNumber("Elevator/FeedForward/Ks", 0));
    feedforward.setKg(SmartDashboard.getNumber("Elevator/FeedForward/Kg", 0));
    feedforward.setKv(SmartDashboard.getNumber("Elevator/FeedForward/Kv", 0));
    feedforward.setKa(SmartDashboard.getNumber("Elevator/FeedForward/Ka", 0));
  }

  public void set(double speed) {
    if (cancelUp && speed > 0) {speed = 0;}
    if (cancelDown && speed < 0) {speed = 0;}
    rightMoter.set(speed);
    leftMoter.set(speed);
  }
  public void setSetpoint(double setpoint){
    if (setpoint > ElevatorConstants.Max)setpoint = ElevatorConstants.Max;
    else if (setpoint < ElevatorConstants.Min)setpoint = ElevatorConstants.Min;
    SmartDashboard.putNumber("Elevator/PID/setpoint", setpoint);
    double pid = Pid.calculate(getPosition(), setpoint);
    pid += feedforward.calculate(setpoint);
    SmartDashboard.putNumber("Elevator/PID/ Output", pid);
    set(pid);
  }
  public void stop() {
    rightMoter.stopMotor();
    leftMoter.stopMotor();
  }
  
  public double getLeftPosition(){
    return leftMoter.getEncoder().getPosition();
  }
  public double getRightPosition(){
    return rightMoter.getEncoder().getPosition();
  }
  public double getPosition(){
    return (getLeftPosition() + getRightPosition()) / 2;
  }
  public double getLeftVelocity(){
    return leftMoter.getEncoder().getVelocity();
  }
  public double getRightVelocity(){
    return rightMoter.getEncoder().getVelocity();
  }
  public double getVelocity(){
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }
  public void resetEncoders() {
    rightMoter.getEncoder().setPosition(0);
    leftMoter.getEncoder().setPosition(0);
  }
}
