// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

    public static final class controlConstants {
        public static final double operatorLSDeadZone = 0.2; //Left Stick Deadzone
        public static final double operatorRSDeadZone = 0.2; //Right Stick Deadzone
        public static final double driverLSDeadZone = 0.2; //Left Stick Deadzone
        public static final double driverRSDeadZone = 0.2; //Right Stick Deadzone
    }
    public static final class ModuleConstants {
        public static final double neoFreeSpeedRPM = 5820;
        public static final int sparkMaxDataPort = -1;
        
        //Drive Motor
        public static final double driveMotorRotToMetre = 14.0/50.0 * 27.0/17.0 * 15.0/45.0 * 4 * Math.PI / 39.3;
        public static final double driveMotorRPMToMetresPerSecond = driveMotorRotToMetre / 60.0;

        //4.60 m/s
        public static final double driveMaxSpeedMPS = neoFreeSpeedRPM * driveMotorRPMToMetresPerSecond;
        public static final double slowModeSpeed = 0.15;

        //Turning Motor

        public static final double turningRotToWheelDegree = 1.0 / (150.0 / 7.0) * 360;
        public static final double turningRPMToDegreePerSecond = turningRotToWheelDegree / 60.0;

        public static final double turningMotorRotToRadian = 1.0 / (150.0 / 7.0) * 2 * Math.PI;
        public static final double turningMotorRPMToRadianPerSecond = turningMotorRotToRadian / 60.0;

        //Absolute Encoder
        public static final double voltToDegree = 360.0 / 3.3;
        public static final double voltToRad = 2 * Math.PI / 3.3;
        
        //Front Left
        public static final int frontLeftDriveCANID = 3;
        public static final int frontLeftTurningCANID = 1;
        public static final int frontLeftTurningEncoderID = 2; //:)
        public static final boolean frontLeftReversed = true;
        public static final double frontLeftAbsoluteOffset = 213.9;//146.2; 
        public static final double frontLeftTurningkP = 0.0035;
        public static final double frontLeftTurningkI = 0.05;

        //Front Right
        public static final int frontRightDriveCANID = 9;
        public static final int frontRightTurningCANID = 11;
        public static final int FrontRightTurningEncoderID = 3;
        public static final boolean frontRightReversed = true;
        public static final double frontRightAbsoluteOffset = 149; //152.70
        public static final double frontRightTurningkP = 0.004;
        public static final double frontRightTurningkI = 0.05;

        //Back Left
        public static final int backLeftDriveCANID = 4;
        public static final int backLeftTurningCANID = 2;
        public static final int backLeftTurningEncoderID = 0;
        public static final boolean backLeftReversed = true;
        public static final double backLeftAbsoluteOffset = 72.7; //72.73
        public static final double backLeftTurningkP = 0.004;
        public static final double backLeftTurningkI = 0.05;

        //Back Right
        public static final int backRightDriveCANID = 8;
        public static final int backRightTurningCANID = 10;
        public static final int backRightTurningEncoderID = 1;
        public static final boolean backRightReversed = true;
        public static final double backRightAbsoluteOffset = 95.2;//Units.radiansToDegrees(1.639)//95.95;
        public static final double backRightTurningkP = 0.004;
        public static final double backRightTurningkI = 0.05;
    }

    public static final class DriveConstants {
        public static final int pigeonCANID = 6;
        public static final double kDeadband = 0.05;

        public static final double kTrackwidth = Units.inchesToMeters(23.75);
            //Math to get Max Angular Speed
            private static final double rotDiameter = Math.sqrt(2 * kTrackwidth * kTrackwidth);
            private static final double rotCircumference = rotDiameter * Math.PI;
            private static final double secondsForOneRot = rotCircumference / ModuleConstants.driveMaxSpeedMPS;
            private static final double maxAngularSpeedRotPerSecond = 1.0 / secondsForOneRot;
        public static final double maxAngularSpeedRadiansPerSecond = maxAngularSpeedRotPerSecond * 2 * Math.PI;
        //10.78 rad/s
        //617.56 deg/s

        public static final double driveBaseRadius = rotDiameter / 2.0;

        //FL, FR, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackwidth / 2.0, kTrackwidth / 2.0),
            new Translation2d(kTrackwidth / 2.0, -kTrackwidth / 2.0),
            new Translation2d(-kTrackwidth / 2.0, kTrackwidth / 2.0),
            new Translation2d(-kTrackwidth / 2.0, -kTrackwidth / 2.0));

        public static final double translationKP = 2.3; //2.3
        public static final double translationKI = 0;
        public static final double translationKD = 0;

        public static final double rotationKP = 2.4; //2.3
        public static final double rotationKI = 1.2;
        public static final double rotationKD = 0;

    }
    public static final class ShooterConstants {
        public static final int shooterPriority = 2;
        public static final double L1LeftSpeed = -0.03;
        public static final double L1RightSpeed = -0.18;
        public static final double L4Speed = -0.30;
        public static final double ShootSpeed = -0.23;
    }
    public static final class AlgaeConstants {
        public static final double PivotRatio = (1.0/36.0) * (16.0/38.0); //85.5:1 wtf kind of ratio is that
        public static final double intakeRatio = (1.0/4.0) * (18.0/42.0); //85.5:1 wtf kind of ratio is that
        public static final double slowModeSpeed = 0.2;
        public static final double pivotMotorRotToRad = PivotRatio * (2 * Math.PI);
        public static final double intakeMotorRotToRad = intakeRatio * (2 * Math.PI);
        public static final double voltageMin = 9.9;
        public static final double pivotMin = 0;
        public static final double pivotKick = 152;
        public static final double pivotOut = 30.0;
        public static final double pivotDanger = 90.0;
        public static final double pivotMax = 185;
        public static final double MinAlgaeHeight = 7.0;
        public static final double MaxAlgaeHeight = 117.5;
    }

    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    public static final class ElevatorConstants {
        public static final int motorRatio = 12; // 12:1
        public static final double maxSpeed = 50; // inches per second 47.19
        public static final double maxAcceleration = 40; // inches per second squared
        public static final double motorRotToIN = (motorRatio * (2 * Math.PI) / 0.8755); // * 2; // 1.751//ratio * 2pi / gear radius in inchs
        public static final double Min = 1; // highest point for min
        public static final double Drop = 4; // highest point for drop
        public static final double Feedforward = 0.40;
        public static final double slowModetrigger = 20;
        public static final double Max = 122; // lowest point for max
        public static final double TOFEdgeBuffer = 100;
        public static final double coralL1 = 12;
        public static final double coralL2 = 26;
        public static final double coralL3 = 62;
        public static final double coralL4H = 120.1;
        public static final double coralL4M = 119.1;
        public static final double coralL4L = 118.6;
        public static final double algaeL0 = 0.1;
        public static final double algaeL2 = 49;
        public static final double algaeL3 = 86;
        public static final double slowModeSpeed = 0.2;
        public static final int elevatorPriority = 1;
    }

    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    public static final class airlockConstants {
        /** array [min, max] */
        public static final double[] frontLCTrigger = {0, 60};
        /** array [min, max] */
        public static final double[] backLCTrigger = {0, 60};
    }

    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    public static final class falconFlareConstants {
        public static final Map<String, Boolean[]> colours = new HashMap<>();
        static {
            colours.put("white", new Boolean[]{false, true, false});
            colours.put("green", new Boolean[]{false, true, true});
            colours.put("purple", new Boolean[]{true, false, false});
            colours.put("yellow", new Boolean[]{true, false, true});
            colours.put("blue", new Boolean[]{true, true, false});
            colours.put("red", new Boolean[]{true, true, true});
        }
    }

    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    public static final class limelightConstants {
        public static final int[] redReef = {};
        public static final int[] blueReef = {};
        public static final double[] LLendoffset = {0.3175, 0, 0.5334, 0, -20, 0};
        public static final double[] LLfunneloffset = {0, -2.75, 0, 0, 0, 0};
    }
}   