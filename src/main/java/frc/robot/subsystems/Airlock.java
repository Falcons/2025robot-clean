// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIDs.AirlockIDs;

public class Airlock extends SubsystemBase {
  private LaserCan front, back; //front is away from elevator
  /** Creates a new Airlock. */
  public Airlock() {
    front = new LaserCan(AirlockIDs.frontLazerCan);
    back = new LaserCan(AirlockIDs.backLazerCan);
    try{
      front.setRangingMode(RangingMode.SHORT);
      back.setRangingMode(RangingMode.SHORT);
    }catch(ConfigurationFailedException e){
      System.err.print(e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
