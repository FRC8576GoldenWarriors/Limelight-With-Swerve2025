// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.collections.ReadOnlyPrimitiveLongSet;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
  //PRACTICE MODULES
  // private SwerveModule leftFront = new SwerveModule(
  //   Constants.SwerveConstants.LEFT_FRONT_DRIVE_ID, 
  //   Constants.SwerveConstants.LEFT_FRONT_TURN_ID, 
  //   false, 
  //   true, 
  //   Constants.SwerveConstants.LEFT_FRONT_CANCODER_ID, 
  //   Constants.SwerveConstants.LEFT_FRONT_OFFSET);

  // private SwerveModule rightFront = new SwerveModule(
  //   Constants.SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
  //   Constants.SwerveConstants.RIGHT_FRONT_TURN_ID, 
  //   true, //used to be true, might have to change back - Om: 2/14/24
  //   true, 
  //   Constants.SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
  //   Constants.SwerveConstants.RIGHT_FRONT_OFFSET);

  // private SwerveModule leftBack = new SwerveModule(
  //   Constants.SwerveConstants.LEFT_BACK_DRIVE_ID, 
  //   Constants.SwerveConstants.LEFT_BACK_TURN_ID, 
  //   true, 
  //   true, 
  //   Constants.SwerveConstants.LEFT_BACK_CANCODER_ID, 
  //   Constants.SwerveConstants.LEFT_BACK_OFFSET);

  //   private SwerveModule rightBack = new SwerveModule(
  //   Constants.SwerveConstants.RIGHT_BACK_DRIVE_ID, 
  //   Constants.SwerveConstants.RIGHT_BACK_TURN_ID, 
  //   true, 
  //   true, 
  //   Constants.SwerveConstants.RIGHT_BACK_CANCODER_ID, 
  //   Constants.SwerveConstants.RIGHT_BACK_OFFSET);


  //COMPETITIOM MODULES
   private SwerveModule leftFront = new SwerveModule(
    Constants.SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    Constants.SwerveConstants.LEFT_FRONT_TURN_ID, 
    false, 
    true, 
    Constants.SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    Constants.SwerveConstants.LEFT_FRONT_OFFSET);

  private SwerveModule rightFront = new SwerveModule(
    Constants.SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    Constants.SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, //used to be true, might have to change back - Om: 2/14/24
    true, 
    Constants.SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    Constants.SwerveConstants.RIGHT_FRONT_OFFSET);

  private SwerveModule leftBack = new SwerveModule(
    Constants.SwerveConstants.LEFT_BACK_DRIVE_ID, 
    Constants.SwerveConstants.LEFT_BACK_TURN_ID, 
    true, 
    true, 
    Constants.SwerveConstants.LEFT_BACK_CANCODER_ID, 
    Constants.SwerveConstants.LEFT_BACK_OFFSET);

    private SwerveModule rightBack = new SwerveModule(
    Constants.SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    Constants.SwerveConstants.RIGHT_BACK_TURN_ID, 
    true, 
    true, 
    Constants.SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    Constants.SwerveConstants.RIGHT_BACK_OFFSET);

    


  private SlewRateLimiter frontLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);

  private static final Drivetrain drivetrain = new Drivetrain();

  public SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveConstants.DRIVE_KINEMATICS, getHeadingRotation2d(), getModulePositions(), new Pose2d());

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          Constants.SwerveConstants.DRIVE_KINEMATICS,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            leftFront.getPosition(),
            rightFront.getPosition(),
            leftBack.getPosition(),
            rightBack.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
          private StructPublisher<Pose2d> m_publisher;
  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    // AutoBuilder.configureHolonomic(
    //   this::getPose2d,
    //   this::resetPose2d,
    //   this::getRobotRelativeSpeeds,
    //   this::driveRobotRelative,
    //   Constants.SwerveConstants.AUTO_CONFIG,
    //   () -> isRedAlliance(),
    //   this
    // );

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> leftFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> leftFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> rightFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> rightFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> leftBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> leftBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> rightBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> rightBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> (getHeading() / 180 * Math.PI), null);
      }
    });
    m_publisher = NetworkTableInstance.getDefault().getStructTopic(Constants.VisionConstants.nameConstants.publishName, Pose2d.struct).publish();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //RobotContainer.poseEstimator.updateOdometry(getHeadingRotation2d(), getModulePositions());
    
    double yaw = gyro.getYaw().getValueAsDouble();
    SmartDashboard.putNumber("Robot Angle", getHeading());
    //rates 2 is yaw (XYZ in order )
    /*SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((yaw/ 180)) + "pi rad/s");
    // Logger.recordOutput("Robot Angle", getHeading());
    // Logger.recordOutput("Robot Pitch", getPitch());
    // Logger.recordOutput("Robot Roll", getRoll());
    // Logger.recordOutput("Pose", getPose().toString());
    // Logger.recordOutput("Angular Speed", new DecimalFormat("#.00").format((yaw / 180)) + "pi rad/s" );

    SmartDashboard.putString("Pose", getPose2d().toString());

    //new values
    SmartDashboard.putNumber("Left Front Module Velocity", leftFront.getDriveMotorVelocity());
    SmartDashboard.putNumber("Right Front Module Velocity", rightFront.getDriveMotorVelocity());
    SmartDashboard.putNumber("Left Back Module Velocity", leftBack.getDriveMotorVelocity());
    SmartDashboard.putNumber("Right Back Module Velocity", rightBack.getDriveMotorVelocity());
    SmartDashboard.putNumber("Left Front Module abs angle", leftFront.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Front Module abs angle", rightFront.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Left Back Module abs angle", leftBack.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Back Module abs angle", rightBack.getAbsoluteEncoderAngle());*/

    /*SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> leftFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> leftFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> rightFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> rightFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> leftBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> leftBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> rightBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> rightBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading(), null);
      }
    });// */
  

    // Logger.recordOutput("Drivetrain/Robot Angle", getHeadingRotation2d().getRadians());
    // Logger.recordOutput("Drivetrain/Pose", getPose());
    // Logger.recordOutput("Drivetrain/Angular Speed", yaw / 180);
    // Logger.recordOutput("Drivetrain/Module States", getModuleStates());

    odometry.update(getHeadingRotation2d(), getModulePositions());
    updatePose();
    m_publisher.set(m_poseEstimator.getEstimatedPosition());
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ 
      //Drive with rotational speed control w/ joystick
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.15 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.15 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.15 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }
    public void updatePose() {
    m_poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          leftFront.getPosition(),
          rightFront.getPosition(),
          leftBack.getPosition(),
          rightBack.getPosition()
        });


    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }


  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    System.out.println("resetAllEncoders()");
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return (Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360)); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  // public void setModuleZero(){ Not Called Anywhere
  //   leftFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   rightFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   leftBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   rightBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  // }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  } 

  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  public void drive (Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLooop){
    ChassisSpeeds chassisSpeeds;
    if(fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);

    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);

    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public void resetPose2d(Pose2d pose){
    odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public boolean isRedAlliance(){
    if (DriverStation.getAlliance().isPresent()){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}