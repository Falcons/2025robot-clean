// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import java.util.Map;

//import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
// import frc.robot.Constants.limelightConstants;
// import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
  RobotConfig config;
  private final SwerveModule frontLeft = new SwerveModule(
    "Front Left",
    ModuleConstants.frontLeftDriveCANID, 
    ModuleConstants.frontLeftTurningCANID,
    ModuleConstants.frontLeftTurningEncoderID,//Use this constant (-1) if connected with Spark Max Breakout Board instead of RIO Analog IN
    ModuleConstants.frontLeftReversed, 
    ModuleConstants.frontLeftAbsoluteOffset);

  private final SwerveModule frontRight = new SwerveModule(
    "Front Right",
    ModuleConstants.frontRightDriveCANID, 
    ModuleConstants.frontRightTurningCANID,
    ModuleConstants.FrontRightTurningEncoderID,
    ModuleConstants.frontRightReversed, 
    ModuleConstants.frontRightAbsoluteOffset);
  
  private final SwerveModule backLeft = new SwerveModule(
    "Back Left",
    ModuleConstants.backLeftDriveCANID, 
    ModuleConstants.backLeftTurningCANID,
    ModuleConstants.backLeftTurningEncoderID,
    ModuleConstants.backLeftReversed, 
    ModuleConstants.backLeftAbsoluteOffset);

  private final SwerveModule backRight = new SwerveModule(
    "Back Right",
    ModuleConstants.backRightDriveCANID, 
    ModuleConstants.backRightTurningCANID,
    ModuleConstants.backRightTurningEncoderID,
    ModuleConstants.backRightReversed, 
    ModuleConstants.backRightAbsoluteOffset);

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID);
  // private double maxSpeed = ModuleConstants.driveMaxSpeedMPS;

  PPHolonomicDriveController holoController = new PPHolonomicDriveController(
        new PIDConstants(DriveConstants.translationKP, DriveConstants.translationKI, DriveConstants.translationKD),
        new PIDConstants(DriveConstants.rotationKP, DriveConstants.rotationKI, DriveConstants.rotationKD));
  //PhotonCamera photonCam;

  private final PIDController xPID = new PIDController(DriveConstants.translationKP, DriveConstants.translationKI, DriveConstants.translationKD);
  private final PIDController yPID = new PIDController(DriveConstants.translationKP, DriveConstants.translationKI, DriveConstants.translationKD);
  private final PIDController rotationPID = new PIDController(DriveConstants.rotationKP, DriveConstants.rotationKI, 0);
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), new Pose2d());
  private Alert slowModeAlert = new Alert("drive slow mode active", AlertType.kInfo);
  public double speedMod = 1;
  public Boolean invert = false;

  // private final Field2d field2024 = new Field2d();
  private final Field2d field2025 = new Field2d();

  private final SwerveModule[] modules = new SwerveModule[] {
    frontLeft,
    frontRight,
    backLeft,
    backRight
  };

  private final Map<String, SwerveModule> swerveMap = Map.of(
    "Front Left", frontLeft, 
    "Front Right", frontRight,
    "Back Left", backLeft,
    "Back Right", backRight);

  private final Map<Character, PIDController> pidMap = Map.of(
    'x', xPID,
    'y', yPID,
    'o', rotationPID);

  private final DigitalInput opticalLimit = new DigitalInput(0);

  StructArrayPublisher<SwerveModuleState> commandedStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Commanded", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates/Actual", SwerveModuleState.struct).publish();
  //StructPublisher<Pose2d> posPublisher = NetworkTableInstance.getDefault().getStructTopic("SwervePose/Actual", Pose2d.struct).publish();

  public SwerveSubsystem() {
    //SmartDashboard.putNumber("swerve/max speed", ModuleConstants.driveMaxSpeedMPS); //adjust via SmartDashboard 
    //photonCam = new PhotonCamera("USB2.0_PC_CAMERA");
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setIZone(0.05);

    xPID.reset();
    yPID.reset();
    rotationPID.reset();

    //copied from 0toAuto swerve video, waits 1 second for gyro calibration to zero heading
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        // zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    resetPose(new Pose2d());

    // Sends PID Controllers to Shuffleboard
    SmartDashboard.putData("RobotPID/X PID", xPID);
    SmartDashboard.putData("RobotPID/Y PID", yPID);
    SmartDashboard.putData("RobotPID/Rotation PID", rotationPID);
    SmartDashboard.putNumber("swerve/Module Setpoint", 0);
    try{
      config = RobotConfig.fromGUISettings();
    }catch (Exception e) {
      e.printStackTrace();
    }

    // PathPlanner Initialization
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      holoController,
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
      );
  }

  @Override
  public void periodic() {
    statePublisher.set(getModuleStates());
    //posPublisher.set(poseEstimator.getEstimatedPosition());

    updatePoseEstimator();
    field2025.setRobotPose(poseEstimator.getEstimatedPosition());
    
    for (SwerveModule module:modules) {
      SmartDashboard.putNumber("Module/Speed/" + module.moduleName, module.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Module/Angle/" + module.moduleName, module.getState().angle.getDegrees());
      SmartDashboard.putNumber(module.moduleName + "raw Abs", module.getAbsEncoderRaw());
      SmartDashboard.putNumber(module.moduleName + "raw Abs", module.getRawPositionWithOffset());
      module.driveFaultAlert.set(module.hasActiveDriveFault()); module.driveFaultAlert.setText(module.getActiveDriveFaults().toString());
      module.turningFaultAlert.set(module.hasActiveTurningFault()); module.turningFaultAlert.setText(module.getActiveTurningFaults().toString());
      module.driveWarningAlert.set(module.hasActiveDriveWarning()); module.driveWarningAlert.setText(module.getActiveDriveWarnings().toString());
      module.turningWarningAlert.set(module.hasActiveTurningWarning()); module.turningWarningAlert.setText(module.getActiveTurningWarnings().toString());
    }

    // SmartDashboard.putNumber("Back Right Raw Abs", backRight.getAbsEncoderRaw());
    // SmartDashboard.putNumber("Back Right Raw w offset", backRight.getRawPositionWithOffset());

    SmartDashboard.putNumber("Robot/FieldX", getPose().getX());
    SmartDashboard.putNumber("Robot/X Speed", getChassisSpeeds().vxMetersPerSecond);

    SmartDashboard.putNumber("Robot/FieldY", getPose().getY());
    SmartDashboard.putNumber("Robot/Y Speed", getChassisSpeeds().vyMetersPerSecond);

    SmartDashboard.putNumber("Robot/Heading Radians", getWrappedHeadingRadians());
    SmartDashboard.putNumber("Robot/Turning Speed", getChassisSpeeds().omegaRadiansPerSecond);

    SmartDashboard.putData(field2025);
    SmartDashboard.putBoolean("Limit/Get", opticalLimit.get());
    /*
    var result = photonCam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("Has targets", hasTargets);

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      double area = target.getArea();
      SmartDashboard.putNumber("PhotonCam/Area", area);
    } 
    */
  }

  /** Stops all Swerve Motors */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

public Command followPathCommand(PathPlannerPath path) {
  try{
      return new FollowPathCommand(
              path,
              this::getPose, // Robot pose supplier
              this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
              holoController, // The controller that will generate feedforwards
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
  } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
  }
}

// Gyro

  /** Resets Gyro Heading to 0 */
  public void zeroHeading() {
    gyro.reset();
    System.out.println("gyro heading reset");
  }
  /** sets Gyro Heading */
  public void setHeading(double angle){
    gyro.setYaw(angle);
    System.out.println("gyro heading set to " + angle);
    /*
    double radAngle = Units.degreesToRadians(angle);
    poseEstimator.resetRotation(new Rotation2d(radAngle));
    */
  }
  /** @return Gyro Rotation2d, continuous */
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }
  public double getYawInDegrees(){
    return gyro.getYaw().getValueAsDouble();
  }
  /** @return Gyro Heading in Radians (-Pi, Pi) CCW+ */
  public double getWrappedHeadingRadians() {
    return Math.IEEEremainder(getRotation2d().getRadians(), 2 * Math.PI);
  }

  /** @return Gyro Heading in Degrees(-180, 180) CCW+ */
  public double getWrappedHeadingDegrees() {
    return getWrappedHeadingRadians() * 180 / Math.PI;
  }

// SwerveModuleStates

  /** Sets all 4 Modules to specified Speed and Angle */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    commandedStatePublisher.set(desiredStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.driveMaxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** @return Array of all 4 Module Speed and Angle */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

// ChassisSpeeds

  /** @return Robot-Relative chassisSpeeds */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** Sets Module states from a chassisSpeeds */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

// Odometry/Pose Estimation

  /** Sets Robot Pose */
  public void resetPose(Pose2d pose) {
    //odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /** @return Robot Pose */
  public Pose2d getPose() {
    //return odometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  /** @return Array of current Position and Angle of all 4 Modules */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void setSlowMode(boolean toggle) {
    if (toggle) {
      speedMod = ModuleConstants.slowModeSpeed;
    } else {
      speedMod = 1;
    }
    slowModeAlert.set(toggle);
  }
  /** @return true if slow mode is active */
  public boolean getSlowMode() {
    return speedMod == ModuleConstants.slowModeSpeed;
  }

  /** Updates Robot Pose based on Gyro and Module Positions */
  public void updatePoseEstimator() {
    poseEstimator.update(getRotation2d(), getModulePositions());
    /*
    boolean useMegaTag2 = true; 
    boolean doTRejectUpdate = false;
    boolean doERejectUpdate = false;
    
    //using megatag 1
    if (!useMegaTag2) { 
      LimelightHelpers.PoseEstimate mt1_T = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-tag");
      LimelightHelpers.PoseEstimate mt1_E = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-end");

      if (mt1_T.tagCount == 1) {
        if (mt1_T.rawFiducials.length == 1){
          if (mt1_T.rawFiducials[0].ambiguity > 0.7 || mt1_T.rawFiducials[0].distToCamera > 3) {
            doTRejectUpdate = true;
          }
        }
      }
      if (mt1_E.tagCount == 1 && mt1_E.rawFiducials.length == 1) {
        if (mt1_E.rawFiducials[0].ambiguity > 0.7 || mt1_E.rawFiducials[0].distToCamera > 3) {
          doERejectUpdate = true;
        }
      } 

      if (mt1_T.tagCount == 0) doTRejectUpdate = true;
      if (mt1_E.tagCount == 0) doERejectUpdate = true;
      

      if(!doTRejectUpdate || !doERejectUpdate) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999)); //StdDev from Limelight website
      }
      if(!doTRejectUpdate) poseEstimator.addVisionMeasurement(mt1_T.pose, mt1_T.timestampSeconds);
      if(!doERejectUpdate) poseEstimator.addVisionMeasurement(mt1_E.pose, mt1_E.timestampSeconds);
      
    // using megatag 2 (updated)
    } else {
      LimelightHelpers.SetRobotOrientation("limelight-tag", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.SetRobotOrientation("limelight-end", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate mt2_T = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-tag");
      LimelightHelpers.PoseEstimate mt2_E = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-end");
  
      if (Math.abs(-gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
        doTRejectUpdate = true;
        doERejectUpdate = true;
      }
      if(mt2_T == null) {doTRejectUpdate = true; System.out.println("MT2_T is null");} 
      else if (mt2_T.tagCount == 0) doTRejectUpdate = true;
      if(mt2_E == null) {doERejectUpdate = true; System.out.println("MT2_E is null");}
      else if (mt2_E.tagCount == 0) doERejectUpdate = true;
      
      

      if (!doTRejectUpdate || !doERejectUpdate) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999)); //StdDev from Limelight website
      }
      if (!doTRejectUpdate) poseEstimator.addVisionMeasurement(mt2_T.pose, mt2_T.timestampSeconds);
      if (!doTRejectUpdate) poseEstimator.addVisionMeasurement(mt2_E.pose, mt2_E.timestampSeconds);
    }*/
    
  }

// PID

  /**
   * Commands Swerve Module to Setpoint
   * @param moduleName Name of Swerve Module (e.g. "Front Left")
   */
  public FunctionalCommand modulePIDTuning(String moduleName) {
    SwerveModule mod = swerveMap.get(moduleName);
    return new FunctionalCommand(
      mod::pidReset,
      mod::pidTuning,
      interrupted -> mod.stop(),
      () -> false);
  }

  public void moduleRawSetpoint(String moduleName, double setpoint){
    SwerveModule mod = swerveMap.get(moduleName);
    mod.rawSetPoint(setpoint);
  }
  /** Sets all 4 Module Setpoint from Smartdashboard value */
  public void allModuleSetpoint() {
    frontLeft.setpoint();
    frontRight.setpoint();
    backLeft.setpoint();
    backRight.setpoint();
  }

  /** Gets Robot PID Setpoint */
  public double robotPIDSetpoint(char controller) {
    PIDController pid = pidMap.get(controller);
    return pid.getSetpoint();
  }

  /**
   * Method for any Robot PID calculation
   * @param controller 'x', 'y', or 'o' for xPID, yPID, rotationPID
   * @param measurement sensor to read
   * @param setpoint target value
   * @return output of chosen PIDController
   */
  public double robotPIDCalc(char controller, double measurement, double setpoint) {
    PIDController pid = pidMap.get(controller);
    return pid.calculate(measurement, setpoint);
  } 
}
