// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;




// import com.revrobotics.spark.*;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.drivers.WarriorSparkMax;
// import frc.robot.Constants;

// public class Shooter extends SubsystemBase {
  

//   private WarriorSparkMax pivotMotor;


//   private DutyCycleEncoder shooterEncoder;

//   // private ProfiledPIDController controller;
//   // private ArmFeedforward feedforward;

//   // private double setpoint_rad;
//   // private boolean homing = true;
 
//   //private CANSparkMax pivotMotor; not implemented yet

//   /** Creates a new Shooter. */
//   public Shooter() {
    

//     pivotMotor = new WarriorSparkMax(Constants.ShooterConstants.pivotCANSparkID, MotorType.kBrushless, false, IdleMode.kCoast, 50);
//     //pivotMotor.restoreFactoryDefaults();

//     Timer.delay(0.2);

//     //pivotMotor.setSmartCurrentLimit(50);
        
//     pivotMotor.setIdleMode(IdleMode.kBrake);


//     Timer.delay(0.2);

//     //check these parameters f
//     shooterEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterEncoderID, 1.0, Constants.ShooterConstants.shooterEncoderOffset); // 
//     // shooterEncoder.(Constants.ShooterConstants.shooterEncoderOffset);
    
//     // controller = new ProfiledPIDController(0.61, 0, 0, new Constraints(Units.degreesToRadians(180.0), Units.degreesToRadians(360.0)));

//     // controller.setTolerance(Units.degreesToRadians(2.0));
//     // controller.enableContinuousInput(-Math.PI, Math.PI);
//     // feedforward = new ArmFeedforward(0.09, 0.86, 1.95);


//   }

//   public SparkMax getShooterMotor(){
//     return pivotMotor;
//   }
  

//   public void setPivotSpeed(double speed){
//     pivotMotor.set(speed);
//   }

  
//   public void zeroEncoder(){
//     shooterEncoder.reset();
//   }

//   public double getPivotMotorVoltage(){
//     return pivotMotor.getBusVoltage();
//   }

//   public void setPivotVoltage(double voltage){
//     pivotMotor.setVoltage(voltage);
//   }

//   // public void setSetpoint(double setpoint){
//   //   this.setpoint_rad = setpoint;
//   //   controller.setGoal(new State(setpoint_rad, 0.0));
//   // }

//   // public void setHoming(boolean homing){
//   //   this.homing = homing;
//   // }

//   // public double getAngle(){
//   //   return Math.abs(shooterEncoder.getDistance()) * 2 * Math.PI;
//   // }

//   // public boolean getHoming(){
//   //   return homing;
//   // }

//   // public boolean atGoal(){
//   //   return controller.atGoal();
//   // }
 
//   // public double getWrappedAngle(){
//   //   return MathUtil.angleModulus(getAngle());
//   // }

//   // public void resetController(){
//   //   controller.reset(getWrappedAngle(), pivotMotor.getEncoder().getVelocity());
//   // }

//   public void stop(){
//     pivotMotor.setVoltage(0.0);
//   }

//   public DutyCycleEncoder getShooterEncoder(){
//     return shooterEncoder;
//   }
//   public double getAbsoluteDistance(){
//     return Math.abs(shooterEncoder.getDistance());
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("Shooter Encoder Position", getShooterEncoder().getAbsolutePosition());
//     SmartDashboard.putNumber("Shooter encoder distance: ", Math.abs(getShooterEncoder().getDistance()));
//     // This method will be called once per scheduler run
//   }
// }
