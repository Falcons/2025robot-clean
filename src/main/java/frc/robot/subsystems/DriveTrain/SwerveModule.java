package frc.robot.subsystems.DriveTrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    public final String moduleName;
    private final SparkMax driveMotor, turningMotor;
    private SparkMaxConfig driveConfig, turningConfig;
    private final RelativeEncoder driveEncoder, turningEncoder;

    private final PIDController turningPID;

    private final SparkAnalogSensor absEncoder;
    private final AnalogEncoder absEncoderRIOAIPin;
    private final double absEncoderOffset;
    public Alert driveFaultAlert = new Alert("Faults", "", Alert.AlertType.kError);
    public Alert turningFaultAlert = new Alert( "Faults", "", Alert.AlertType.kError);
    public Alert driveWarningAlert = new Alert("Warnings", "", Alert.AlertType.kWarning);
    public Alert turningWarningAlert = new Alert("Warnings", "", Alert.AlertType.kWarning);
    public SwerveModule(String name, int driveMotorID, int turningMotorID, int absEncoderPort, boolean reversed, double offsetDegrees) {
        this.moduleName = name;
        
        this.driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.driveMotorRotToMetre);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.driveMotorRPMToMetresPerSecond);
        driveConfig.smartCurrentLimit(40);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        

        this.turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
        turningConfig = new SparkMaxConfig();
        turningConfig.idleMode(IdleMode.kBrake);
        turningConfig.inverted(reversed);
        turningConfig.analogSensor.positionConversionFactor(ModuleConstants.voltToRad);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.turningRotToWheelDegree);
        turningConfig.encoder.velocityConversionFactor(ModuleConstants.turningRPMToDegreePerSecond);
        turningConfig.smartCurrentLimit(40);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        
        this.driveEncoder = driveMotor.getEncoder();

        this.turningEncoder = turningMotor.getEncoder();

        if (absEncoderPort == -1) {
            absEncoderRIOAIPin = null;
            this.absEncoder = turningMotor.getAnalog();

        } else {
            absEncoder = null;
            absEncoderRIOAIPin = new AnalogEncoder(absEncoderPort);
        }

        this.absEncoderOffset = Units.degreesToRadians(offsetDegrees);

        //kI value is only this high due to max integrator range
        turningPID = new PIDController(0.004, 0.05, 0);
        turningPID.enableContinuousInput(-180, 180);
        turningPID.setTolerance(0.1);
        turningPID.setIntegratorRange(-0.01, 0.01);
        turningEncoder.setPosition(getAbsEncoderDeg());
        
        SmartDashboard.putData("swerve/TurningPID/" + this.moduleName, this.turningPID);

        resetEncoders();
    }

    /** Stops Drive and Turning Motors */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

// Integrated Drive Encoder

    /** @return Drive Encoder Postion in Metres */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Drive Encoder position in Metres per Second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

// Integrated Turning Encoder

    /** @return Raw Integrated Turning Encoder */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /** @return Integrated Turning Encoder in degrees (-180 to 180) CCW+ */
    public double getTurningEncoderDegree() {
        return Math.IEEEremainder(turningEncoder.getPosition(), 360);
      }
    

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

// Absolute Encoder

    public double getAbsEncoderRaw() {
        if (absEncoder != null) {
            return absEncoder.getPosition();
        } else {
            return absEncoderRIOAIPin.get() * 2 * Math.PI;
        }
    }

    /** @return Abs position in correct units, incorrect range */
    public double getRawPositionWithOffset() {
        return getAbsEncoderRaw() - absEncoderOffset;
    }

    /** @return Absolute Encoder in radians (-Pi to Pi) CCW+ */
    public double getAbsEncoderRad() {
        if (getRawPositionWithOffset() < -Math.PI) {
            return getRawPositionWithOffset() + 2 * Math.PI;

        } else if (getRawPositionWithOffset() > Math.PI){
            return getRawPositionWithOffset() - 2 * Math.PI;

        } else {
            return getRawPositionWithOffset();
        }
    }

    /** @return Absolute Encoder in degrees (-180 to 180) CCW+ */
    public double getAbsEncoderDeg() {
        return getAbsEncoderRad() * 180.0 / Math.PI;
    }

    /** Resets Drive encoders and matches Turning encoders with absolute */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsEncoderDeg());
    }

    public boolean hasActiveDriveFault() {
        return driveMotor.hasActiveFault();
    }
    public Faults getActiveDriveFaults() {
        return driveMotor.getFaults();
    }
    public boolean hasActiveDriveWarning() {
        return driveMotor.hasActiveWarning();
    }
    public Warnings getActiveDriveWarnings() {
        return driveMotor.getWarnings();
    }
    public boolean hasActiveTurningFault() {
        return turningMotor.hasActiveFault();
    }
    public Faults getActiveTurningFaults() {
        return turningMotor.getFaults();
    }
    public boolean hasActiveTurningWarning() {
        return turningMotor.hasActiveWarning();
    }
    public Warnings getActiveTurningWarnings() {
        return turningMotor.getWarnings();
    }
// SwerveModuleState

    /** @return Current Speed and Angle of Module */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsEncoderRad()));
    }

    /** Sets Module to specified Speed and Angle */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.driveMaxSpeedMPS);
        turningMotor.set(turningPID.calculate(getAbsEncoderDeg(), state.angle.getDegrees()));
    }

// SwerveModulePosition

    /** @return Current Position and Angle of Module */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

//PID

    /** Turning Motor to Setpoint */
    public void pidTuning() {
        turningMotor.set(turningPID.calculate(getAbsEncoderDeg()));
    }

    /** Sets PID setpoint from Smartdashboard value */
    public void setpoint() {
        double stpt = SmartDashboard.getNumber("swerve/Module Setpoint", 0);
        turningPID.setSetpoint(stpt);
    } 

    public void rawSetPoint(double setpoint){
        turningPID.setSetpoint(setpoint);
    }
    /** Resets the previous error and the integral term. */
    public void pidReset() {
        turningPID.reset();
    }
}
