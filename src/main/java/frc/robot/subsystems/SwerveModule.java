// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule extends SubsystemBase {
private int turnMotorId;
private int driveMotorId;

  private PearadoxSparkMax driveMotor;
  private PearadoxSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;
  private SparkMaxConfig driveConfig;
  private SparkMaxConfig turnConfig;
  

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private double absoluteEncoderOffset;
  private Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;

      this.turnMotorId = turnMotorId;
      this.driveMotorId = driveMotorId;

      driveConfig = new SparkMaxConfig();
      turnConfig = new SparkMaxConfig();

      driveConfig
      .inverted(driveMotorReversed)
      .idleMode(IdleMode.kCoast);

      turnConfig
      .inverted(turnMotorReversed)
      .idleMode(IdleMode.kCoast);

      driveMotor = new PearadoxSparkMax(driveMotorId,MotorType.kBrushless,driveConfig);
      turnMotor = new PearadoxSparkMax(turnMotorId,MotorType.kBrushless,turnConfig);
      // driveMotor = new PearadoxSparkMax(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
      // turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      
      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      absoluteEncoder = new CANcoder(absoluteEncoderId);

      turnPIDController = new PIDController(Constants.SwerveConstants.KP_TURNING, 0, 0.001); //0.001);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
      //last angle in degrees
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber(turnMotorId+"", turnMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber(driveMotorId+" Current: ", driveMotor.getOutputCurrent() );
  } 

  public void setBrake(boolean brake){
    if(brake){
      driveConfig.idleMode(IdleMode.kBrake);
      driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //driveMotor.setIdleMode(IdleMode.kBrake);
      //turnMotor.setIdleMode(IdleMode.kCoast);
      //turnMotor.setIdleMode(IdleMode.kBrake);
      turnConfig.idleMode(IdleMode.kBrake);
      turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    else{
      //driveMotor.setIdleMode(IdleMode.kCoast);
      driveConfig.idleMode(IdleMode.kCoast);
      driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      turnConfig.idleMode(IdleMode.kCoast);
      turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public double getDriveMotorPosition(){
    return driveEncoder.getPosition() * Constants.SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity() * Constants.SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * Constants.SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * Constants.SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  
  //returning rotations
  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle-=absoluteEncoderOffset;
    angle *= (Math.PI*2);
    return angle;
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turnEncoder.setPosition((getAbsoluteEncoderAngle() / Constants.SwerveConstants.TURN_MOTOR_PCONVERSION));
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    //SmartDashboard.putNumber("Pre-optimized", desiredState.speedMetersPerSecond);
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    //SmartDashboard.putNumber("Post-optimized", desiredState.speedMetersPerSecond); 
    setAngle(desiredState);
    setSpeed(desiredState);
    //SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
  }

  public void setSpeed(SwerveModuleState desiredState){
    driveMotor.set(desiredState.speedMetersPerSecond / Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
  }

  public void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; 
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));

    lastAngle = angle;
  }

  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }
}