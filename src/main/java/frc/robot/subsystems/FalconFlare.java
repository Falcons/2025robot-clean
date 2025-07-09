// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.falconFlareConstants;
import frc.robot.DeviceIDs.falconFlareIDs;

public class FalconFlare extends SubsystemBase {
  DigitalOutput D1 = new DigitalOutput(falconFlareIDs.dio1);
  DigitalOutput D2 = new DigitalOutput(falconFlareIDs.dio2);
  DigitalOutput D3 = new DigitalOutput(falconFlareIDs.dio3);
  Optional<DriverStation.Alliance> Alliance = DriverStation.getAlliance();
  Timer timer = new Timer();
  /** Creates a new FalconFlare. */
  public FalconFlare() {}

  @Override
  public void periodic() {
    if(DriverStation.isDisabled()){
      timer.restart();
      if(timer.get() >= 15) reset();
    }
    // This method will be called once per scheduler run
  }

  public void setLights(Boolean[] code) {
    D1.set(code[0]);
    D2.set(code[1]);
    D3.set(code[2]);
  }
  public void setLights(String colour){
    Boolean[] code = falconFlareConstants.colours.get(colour);
    if (code == null) {
      System.out.println("FalconFlare: Invalid colour code: " + colour);
      return;
    }
    setLights(code);
  }
  public void reset(){
    boolean isRed = Alliance.get() == DriverStation.Alliance.Red;
    Boolean[] code = {false, false, isRed};
    setLights(code);
  }
}
