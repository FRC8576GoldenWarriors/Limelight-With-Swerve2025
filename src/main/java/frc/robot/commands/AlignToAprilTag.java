package frc.robot.commands;

import java.io.ObjectInputStream.GetField;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight.AprilTagStatsLimelight;
//import frc.robot.subsystems.Limelight.SpeakerAllignment;

public class AlignToAprilTag extends Command {
    // private final SpeakerAllignment speakerAllignment;
    private AprilTagStatsLimelight aprilTagStatsLimelight;
    private Drivetrain drivetrain;

    private final PIDController rotationPID;
    private final PIDController forwardPID;


    private final double focalLength = Constants.VisionConstants.limelightCameraDimensions.FOCAL_LENGTH;
    private final double pixelWidth = Constants.VisionConstants.limelightCameraDimensions.PIXEL_WIDTH;
    private final double realWidth = Constants.VisionConstants.limelightCameraDimensions.REAL_WIDTH;

    private double driveOutput;
    private double detectedWidth;
    

    private double rotationOutput;
    
    private double callibrationFactor = 1.2;

    private double currentDistance;
    private double tx;
    private double goalDistance;

    private CommandXboxController c = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);

    public AlignToAprilTag(AprilTagStatsLimelight aprilTagStatsLimelight, Drivetrain drivetrain){
        this.aprilTagStatsLimelight = aprilTagStatsLimelight;
        this.drivetrain = drivetrain;

        rotationPID = new PIDController(0.06, 0.0001, 0.001);
        rotationPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_ANGLE_ERROR);

        forwardPID = new PIDController(1.0, 0.0001, 0.001);
        forwardPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);


        addRequirements(aprilTagStatsLimelight, drivetrain);
    }

    // @Override
    // public void initialize(){
    //     speakerAllignment.configureAliance(alliance == Alliance.Blue);
    // }

    @Override
    public void execute(){


        detectedWidth = realWidth * Math.sqrt(aprilTagStatsLimelight.getArea()) / (pixelWidth * focalLength * callibrationFactor);

        tx = aprilTagStatsLimelight.getTX();
        currentDistance = (aprilTagStatsLimelight.calculateDistance(aprilTagStatsLimelight.getID(), focalLength, realWidth, detectedWidth) * 2) / 10000;
        goalDistance = Constants.VisionConstants.limeLightDistanceConstants.DESIRED_APRIL_TAG_DISTANCE;


        SmartDashboard.putNumber("Tag distance", currentDistance);


        rotationOutput = rotationPID.calculate(tx, 0);
        driveOutput = (currentDistance <= goalDistance) ? 0 : forwardPID.calculate(currentDistance, goalDistance);
        
        drivetrain.drive(new Translation2d(driveOutput, 0), rotationOutput, false, true);
        SmartDashboard.putNumber("Vision PID Drive output",driveOutput);

    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished(){
        return (aprilTagStatsLimelight.hasValidTargets() && rotationPID.atSetpoint() && forwardPID.atSetpoint()) ||
         ((currentDistance == 0) || (c.leftTrigger().getAsBoolean()));
    }
}
