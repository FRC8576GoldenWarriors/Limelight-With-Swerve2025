package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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
    private final PIDController distancePID;
    
    private double targetDistance;
    private CommandXboxController c = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);

    public AlignToAprilTag(AprilTagStatsLimelight aprilTagStatsLimelight, Drivetrain drivetrain){
        this.aprilTagStatsLimelight = aprilTagStatsLimelight;
        this.drivetrain = drivetrain;

        rotationPID = new PIDController(0.03, 0.0001, 0.001);
        rotationPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_ANGLE_ERROR);

        distancePID = new PIDController(0.5, 0.0001, 0.001);
        distancePID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);

        addRequirements(aprilTagStatsLimelight, drivetrain);
    }

    // @Override
    // public void initialize(){
    //     speakerAllignment.configureAliance(alliance == Alliance.Blue);
    // }

    @Override
    public void execute(){


        double tx = aprilTagStatsLimelight.getTX();
        targetDistance = aprilTagStatsLimelight.calculateDistance(aprilTagStatsLimelight.getID());

        double rotationOutput = rotationPID.calculate(tx, 0.0);
        // double distanceOutput = distancePID.calculate(targetDistance, Constants.VisionConstants.limeLightDistanceConstants.OPTIMAL_SHOOTING_DISTANCE);

        Translation2d translation = new Translation2d(targetDistance, 0.0);
        //double rotation = rotationOutput;

        drivetrain.drive(translation, rotationOutput, true, true);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished(){
        return (aprilTagStatsLimelight.hasValidTargets() && rotationPID.atSetpoint() && distancePID.atSetpoint()) || ((targetDistance == 0) || (c.leftTrigger().getAsBoolean()));
    }
}
