// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANCoder encoder;
    private double encoderOffset;

    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;
    private double ticksPerRotation = 2048.0;

    private double wheelDiameter = 0.10033;

    private double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    private boolean driveInverted = true;
    private double steerReduction = (15.0 / 32.0) * (10.0 / 60.0);
    private boolean steerInverted = true;

    private double proportionalConstant = 0.2;
    private double integralConstant = 0.0;
    private double derivativeConstant = 0.1;

    private double driveSensorPositionCoefficient;
    private double driveSensorVelocityCoefficient;
    private double steerSensorPositionCoefficient;
    private double steerSensorVelocityCoefficient;

    private double referenceAngleRadians = 0.0;
    

    private int fixTick = 0;
    private int fixCount = 10;

    private int timeoutMS = 10;
    private int statusFrameMS = 255;

    private ErrorCode cancoderSettingsCode;
    private ErrorCode cancoderSensorCode;
    private ErrorCode cancoderMagOffsetCode;

    private ErrorCode driveSettingsCode;
    private ErrorCode driveGeneralCode;
    private ErrorCode driveQuadratureCode;
    private ErrorCode driveAinTempCode;
    private ErrorCode drivePulseWidthCode;
    private ErrorCode driveMotionMagicCode;
    private ErrorCode driveFeedback1Code;
    private ErrorCode driveBasePID0Code;
    private ErrorCode driveTurnPID1Code;

    private ErrorCode steerSettingsCode;
    private ErrorCode steerFeedbackDeviceCode;
    private ErrorCode steerSensorPositionCode;
    private ErrorCode steerGeneralCode;
    private ErrorCode steerQuadratureCode;
    private ErrorCode steerAinTempCode;
    private ErrorCode steerPulseWidthCode;
    private ErrorCode steerMotionMagicCode;
    private ErrorCode steerFeedback1Code;
    private ErrorCode steerBasePID0Code;
    private ErrorCode steerTurnPID1Code;
    

    private CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    private TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    
    public SwerveModule(int driveMotorPort, int steerMotorPort, int encoderPort, double encoderOffset) {
        this.driveMotor = new TalonFX(driveMotorPort);
        this.steerMotor = new TalonFX(steerMotorPort);
        this.encoder = new CANCoder(encoderPort);
        this.encoderOffset = encoderOffset;

        //#region CANCoder setup
        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        cancoderConfig.magnetOffsetDegrees = Math.toDegrees(encoderOffset);
        cancoderConfig.sensorDirection = false;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        cancoderSettingsCode = encoder.configAllSettings(cancoderConfig, timeoutMS);
        cancoderSensorCode = encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, timeoutMS);
        //#endregion

        //#region Drive motor setup
        driveSensorPositionCoefficient = Math.PI * wheelDiameter * driveReduction / ticksPerRotation;
        driveSensorVelocityCoefficient = driveSensorPositionCoefficient * 10.0;

        driveConfig.voltageCompSaturation = nominalVoltage;

        driveConfig.supplyCurrLimit.currentLimit = driveCurrentLimit;
        driveConfig.supplyCurrLimit.enable = true;


        driveSettingsCode = driveMotor.configAllSettings(driveConfig, timeoutMS);

        driveMotor.enableVoltageCompensation(true);
        
        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(driveInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        driveMotor.setSensorPhase(true);

        driveGeneralCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, statusFrameMS, timeoutMS);
        driveQuadratureCode =  driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFrameMS, timeoutMS);
        driveAinTempCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, statusFrameMS, timeoutMS);
        drivePulseWidthCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, statusFrameMS, timeoutMS);
        driveMotionMagicCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, statusFrameMS, timeoutMS);
        driveFeedback1Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, statusFrameMS, timeoutMS);
        driveBasePID0Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, statusFrameMS, timeoutMS);
        driveTurnPID1Code =  driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, statusFrameMS, timeoutMS);

        //#endregions

        //#region Steer motor setup

        steerSensorPositionCoefficient = 2.0 * Math.PI / ticksPerRotation * steerReduction;
        steerSensorVelocityCoefficient = steerSensorPositionCoefficient * 10.0;
    
        steerConfig.slot0.kP = proportionalConstant;
        steerConfig.slot0.kI = integralConstant;
        steerConfig.slot0.kD = derivativeConstant;
        
        steerConfig.voltageCompSaturation = nominalVoltage;

        steerConfig.supplyCurrLimit.currentLimit = steerCurrentLimit;
        steerConfig.supplyCurrLimit.enable = true;

        steerSettingsCode = steerMotor.configAllSettings(steerConfig, timeoutMS);

        steerMotor.enableVoltageCompensation(true);

        steerFeedbackDeviceCode = steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMS);
        
        steerMotor.setSensorPhase(steerInverted);
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        
        steerSensorPositionCode = steerMotor.setSelectedSensorPosition(encoder.getAbsolutePosition() / steerSensorVelocityCoefficient, 0, timeoutMS);

        steerGeneralCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, statusFrameMS, timeoutMS);
        steerQuadratureCode =  steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFrameMS, timeoutMS);
        steerAinTempCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, statusFrameMS, timeoutMS);
        steerPulseWidthCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, statusFrameMS, timeoutMS);
        steerMotionMagicCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, statusFrameMS, timeoutMS);
        steerFeedback1Code = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, statusFrameMS, timeoutMS);
        steerBasePID0Code = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, statusFrameMS, timeoutMS);
        steerTurnPID1Code =  steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, statusFrameMS, timeoutMS);

        //#endregion

    }

    public void resetSteerOffset() {
        steerMotor.setSelectedSensorPosition(encoder.getAbsolutePosition() / steerSensorPositionCoefficient);
    }

    public double getReferenceAngle() {
        return referenceAngleRadians;
    }

    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = steerMotor.getSelectedSensorPosition() * steerSensorPositionCoefficient;

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        steerMotor.set(TalonFXControlMode.Position, adjustedReferenceAngleRadians / steerSensorPositionCoefficient);


        this.referenceAngleRadians = referenceAngleRadians;
    }

    public double getStateAngle() {
        double motorAngleRadians = steerMotor.getSelectedSensorPosition() * steerSensorPositionCoefficient;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    public void setReferenceVoltage(double voltage) {
        driveMotor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
    }

    public double getStateVelocity() {
        return driveMotor.getSelectedSensorVelocity() * driveSensorVelocityCoefficient;
    }

    public double getDriveVelocity() {
        return getStateVelocity();
    }

    public double getSteerAngle() {
        return getStateAngle();
    }

    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        setReferenceVoltage(driveVoltage);
        setReferenceAngle(steerAngle);
    }

    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    public TalonFX getSteerMotor() {
        return steerMotor;
    }

    public CANCoder getEncoder() {
        return encoder;
    }

    public double getEncoderOffset() {
        return encoderOffset;
    }


    public void periodic(){ // This is called every frame that the robot is updated from the drive subsystem.
        
        
        
        switch (fixTick){
            case 0:
                if (cancoderSettingsCode != ErrorCode.OK){
                    cancoderSettingsCode = encoder.configAllSettings(cancoderConfig, timeoutMS);
                }

                if (driveSettingsCode != ErrorCode.OK){
                    driveSettingsCode = driveMotor.configAllSettings(driveConfig, timeoutMS);
                }
                break;

            case 1:
                if (steerSettingsCode != ErrorCode.OK){
                    steerSettingsCode = steerMotor.configAllSettings(steerConfig, timeoutMS);
                }

                if (steerFeedbackDeviceCode != ErrorCode.OK){
                    steerFeedbackDeviceCode = steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMS);
                }   
                break;

            case 2:
                if (cancoderMagOffsetCode != ErrorCode.OK || steerSensorPositionCode != ErrorCode.OK || encoder.configGetMagnetOffset(timeoutMS) != encoderOffset){
                    cancoderMagOffsetCode = encoder.configMagnetOffset(encoderOffset, timeoutMS);
                    steerMotor.setSelectedSensorPosition(encoder.getAbsolutePosition() / steerSensorVelocityCoefficient, 0, timeoutMS);
                }

                if (cancoderSensorCode != ErrorCode.OK){
                    cancoderSensorCode = encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, timeoutMS);
                }
                break;

            case 3:
                if (driveGeneralCode != ErrorCode.OK){
                    driveGeneralCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, statusFrameMS, timeoutMS);
                }
                
                if (steerGeneralCode != ErrorCode.OK){
                    steerGeneralCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, statusFrameMS, timeoutMS);
                }
                break;
            case 4:
                if (driveQuadratureCode != ErrorCode.OK){
                    driveQuadratureCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFrameMS, timeoutMS);
                }

                if (steerQuadratureCode != ErrorCode.OK){
                    steerQuadratureCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFrameMS, timeoutMS);
                }
                break;
            case 5:
                if (driveAinTempCode != ErrorCode.OK){
                    driveAinTempCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, statusFrameMS, timeoutMS);
                }

                if (steerAinTempCode != ErrorCode.OK){
                    steerAinTempCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, statusFrameMS, timeoutMS);
                }
                break;
            case 6:
                if (drivePulseWidthCode != ErrorCode.OK){
                    drivePulseWidthCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, statusFrameMS, timeoutMS);
                }

                if (steerPulseWidthCode != ErrorCode.OK){
                    steerPulseWidthCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, statusFrameMS, timeoutMS);
                }
                break;
            case 7:
                if (driveMotionMagicCode != ErrorCode.OK){
                    driveMotionMagicCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, statusFrameMS, timeoutMS);
                }

                if (steerMotionMagicCode != ErrorCode.OK){
                    steerMotionMagicCode = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, statusFrameMS, timeoutMS);
                }
                break;
            case 8:
                if (driveFeedback1Code != ErrorCode.OK){
                    driveFeedback1Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, statusFrameMS, timeoutMS);
                }

                if (steerFeedback1Code != ErrorCode.OK){
                    steerFeedback1Code = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, statusFrameMS, timeoutMS);
                }
                break;
            case 9:
                if (driveBasePID0Code != ErrorCode.OK){
                    driveBasePID0Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, statusFrameMS, timeoutMS);
                }

                if (steerBasePID0Code != ErrorCode.OK){
                    steerBasePID0Code = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, statusFrameMS, timeoutMS);
                }
           
                break;
            case 10:
                if (driveTurnPID1Code != ErrorCode.OK){
                    driveTurnPID1Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, statusFrameMS, timeoutMS);
                }

                if (steerTurnPID1Code != ErrorCode.OK){
                    steerTurnPID1Code = steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, statusFrameMS, timeoutMS);
                }
                break;
            
            default:
                break;
        }
        
        fixTick++;

        if (fixTick > fixCount){
            fixTick = 0;
        }
    }

}
