package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight.AprilTagStatsLimelight;
//import frc.robot.subsystems.Limelight.SpeakerAllignment;

public class AlignToAprilTag extends Command {
    // private final SpeakerAllignment speakerAllignment;
    private AprilTagStatsLimelight aprilTagStatsLimelight;
    private Drivetrain drivetrain;

    private final PIDController rotationPID;
    private final PIDController forwardPID;
    private final PIDController sidePID;
    
    private double targetDistance;
    private CommandXboxController c = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);

    public AlignToAprilTag(AprilTagStatsLimelight aprilTagStatsLimelight, Drivetrain drivetrain){
        this.aprilTagStatsLimelight = aprilTagStatsLimelight;
        this.drivetrain = drivetrain;

        rotationPID = new PIDController(1, 0.0001, 0.001);
        rotationPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_ANGLE_ERROR);

        forwardPID = new PIDController(1, 0.0001, 0.001);
        forwardPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);
        
        sidePID = new PIDController(1, 0.0001, 0.001);
        addRequirements(aprilTagStatsLimelight, drivetrain);
    }

    // @Override
    // public void initialize(){
    //     speakerAllignment.configureAliance(alliance == Alliance.Blue);
    // }

    @Override
    public void execute(){

        //If possible, change to using PID for turn and forward
        //double turn = rotationPID.calculate(aprilTagStatsLimelight.getTY(), 0);
        //double forward = distancePID.calculate(aprilTagStatsLimelight.getTX(), Constants.VisionConstants.distanceConstants.goalMeterDistance);
        
        // double sideSpeed = -RobotContainer.driverController.getLeftX() * Math.abs(RobotContainer.driverController.getLeftX()) * 1.8;
        // sideSpeed = Math.abs(sideSpeed) > 0.15 ? sideSpeed : 0;
        // SmartDashboard.putNumber("turn speed",turn);
        // SmartDashboard.putNumber("forward speed",forward);
        // SmartDashboard.putNumber("side speed",sideSpeed);
        
        //drivetrain.swerveDrive(forward, sideSpeed, turn, true, new Translation2d(), false);
        // drivetrain.drive(
        //     new Translation2d(), //new Translation2d(aprilTagStatsLimelight.calculateDistance(aprilTagStatsLimelight.getID()), forward) 
        //     turn, 
        //     true,
        //      true);

        //match the angle of the tag
        double targetYaw = aprilTagStatsLimelight.getTX();
        double rotate = rotationPID.calculate(drivetrain.getHeading(), targetYaw);

        double sideDistance = aprilTagStatsLimelight.calculateDistance(aprilTagStatsLimelight.getID()) * Math.tan(Math.toRadians(targetYaw));
        double sideSpeed = sidePID.calculate(sideDistance,0);

        double forwardDistance = aprilTagStatsLimelight.calculateDistance(aprilTagStatsLimelight.getID());
        double forwardSpeed = forwardPID.calculate(forwardDistance, Constants.VisionConstants.distanceConstants.goalMeterDistance);

        drivetrain.swerveDrive(forwardSpeed, sideSpeed, rotate, true, new Translation2d(), false);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished(){
        //return (aprilTagStatsLimelight.hasValidTargets() && rotationPID.atSetpoint() && distancePID.atSetpoint()) || ((targetDistance == 0) || (c.leftTrigger().getAsBoolean()));
        return (rotationPID.atSetpoint() && sidePID.atSetpoint() && forwardPID.atSetpoint())|| !aprilTagStatsLimelight.hasValidTargets();
    }
}
