// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

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

    private double driveSensorVelocityCoefficient;
    private double steerSensorVelocityCoefficient;

    private int fixTick = 0;
    private int fixCount = 22;

    private int timeoutMS = 10;
    private int statusFrameMS = 255;

    private ErrorCode cancoderSettingsCode;
    private ErrorCode cancoderSensorCode;

    private ErrorCode driveSettingsCode;
    private ErrorCode driveGeneralCode;
    private ErrorCode driveQuadratureCode;
    private ErrorCode driveAinTempCode;
    private ErrorCode drivePulssWidthCode;
    private ErrorCode driveMotionMagicCode;
    private ErrorCode driveFeedback1Code;
    private ErrorCode driveBasePID0Code;
    private ErrorCode driveTurnPID1Code;

    private ErrorCode steerSettingsCode;
    private ErrorCode steerFeedbackDeviceCode;
    private ErrorCode steerGeneralCode;
    private ErrorCode steerQuadratureCode;
    private ErrorCode steerAinTempCode;
    private ErrorCode steerPulssWidthCode;
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

        double sensorPositionCoefficient;

        //#region Drive motor setup
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        sensorPositionCoefficient = Math.PI * wheelDiameter * driveReduction / ticksPerRotation;
        
        driveSensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        motorConfiguration.voltageCompSaturation = nominalVoltage;

        motorConfiguration.supplyCurrLimit.currentLimit = driveCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;


        driveSettingsCode = driveMotor.configAllSettings(motorConfiguration);

        driveMotor.enableVoltageCompensation(true);
        
        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(driveInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        driveMotor.setSensorPhase(true);

        driveGeneralCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, statusFrameMS, timeoutMS);
        driveQuadratureCode =  driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, statusFrameMS, timeoutMS);
        driveAinTempCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, statusFrameMS, timeoutMS);
        drivePulssWidthCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, statusFrameMS, timeoutMS);
        driveMotionMagicCode = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, statusFrameMS, timeoutMS);
        driveFeedback1Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, statusFrameMS, timeoutMS);
        driveBasePID0Code = driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, statusFrameMS, timeoutMS);
        driveTurnPID1Code =  driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, statusFrameMS, timeoutMS);

        //#endregion


        //#region Steer motor setup

        sensorPositionCoefficient = 2.0 * Math.PI / ticksPerRotation * steerReduction;
        steerSensorVelocityCoefficient = sensorPositionCoefficient * 10.0;
    
        steerConfig.slot0.kP = proportionalConstant;
        steerConfig.slot0.kI = integralConstant;
        steerConfig.slot0.kD = derivativeConstant;
        
        steerConfig.voltageCompSaturation = nominalVoltage;

        steerConfig.supplyCurrLimit.currentLimit = steerCurrentLimit;
        steerConfig.supplyCurrLimit.enable = true;

        steerSettingsCode = steerMotor.configAllSettings(steerConfig, timeoutMS);

        steerMotor.enableVoltageCompensation(true);

        steerFeedbackDeviceCode = steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMS);
        
        steerMotor.setInverted(TalonFXInvertType.CounterClockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);

.        motor.setSelectedSensorPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient, 0, CAN_TIMEOUT_MS), "Failed to set Falcon 500 encoder position");

        // Reduce CAN status frame rates
        CtreUtils.checkCtreError(
                motor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        STATUS_FRAME_GENERAL_PERIOD_MS,
                        CAN_TIMEOUT_MS
                ),
                "Failed to configure Falcon status frame period"
        );

        return new ControllerImplementation(motor,
                sensorPositionCoeffic
        //#endregion

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

    public void periodic(){
        
        // This is called every frame that the robot is updated from the drive subsystem
        
        switch (fixTick){
            case 0:
                if (cancoderSettingsCode != ErrorCode.OK){
                    cancoderSettingsCode = encoder.configAllSettings(cancoderConfig, timeoutMS);
                }
                break;

            case 1:
                if (cancoderSensorCode != ErrorCode.OK){
                    cancoderSensorCode = encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, timeoutMS);
                }
                break;

            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
            case 7:
                break;
            case 8:
                break;
            case 9:
                break;
            case 10:
                break;
            case 11:
                break;
            case 12:
                break;
            case 13:
                break;
            case 14:
                break;
            case 15:
                break;


            default:
                break;
        }
        
        fixTick++;

        if (fixTick >= fixCount){
            fixTick = 0;
        }
    }

}
