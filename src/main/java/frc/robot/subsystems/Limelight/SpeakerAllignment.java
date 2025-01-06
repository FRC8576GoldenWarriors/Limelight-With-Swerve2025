package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class SpeakerAllignment extends SubsystemBase {
    private final NetworkTable limelightNetworkTable;
    private final PIDController rotationPID;
    private final PIDController distancePID;
    public final Drivetrain swerveDrive;


    public SpeakerAllignment(Drivetrain swerveDrive){
        this.swerveDrive = swerveDrive;
        limelightNetworkTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.limelightNetworkTableKey.LIMELIGHT_NETWORKTABLE_KEY);

        rotationPID = new PIDController(0.03, 0.0001, 0.001);
        rotationPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_ANGLE_ERROR);

        distancePID = new PIDController(0.5, 0.0001, 0.001);
        distancePID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);
    }
    
    // Check if Limelight sees an april tag
    public boolean hasValidTargets() {
        return AprilTagStatsLimelight.hasValidTargets();
    }

    public boolean isAlignedWithSpeaker() {
        return hasValidTargets() && rotationPID.atSetpoint() && distancePID.atSetpoint();
    }

    public void alignWithSpeaker() {
        if(!hasValidTargets()) {
            swerveDrive.stopModules();
            return;
        }

        double tx = AprilTagStatsLimelight.getTX();
        double ty = AprilTagStatsLimelight.getTY();

        double targetDistance = calculateDistance(ty);

        double rotationOutput = rotationPID.calculate(tx, 0.0);
        double distanceOutput = distancePID.calculate(targetDistance, Constants.VisionConstants.limeLightDistanceConstants.OPTIMAL_SHOOTING_DISTANCE);

        Translation2d translation = new Translation2d(targetDistance, 0.0);
        double rotation = rotationOutput;

        swerveDrive.drive(translation, rotation, true, true);
    }

    public void configureAliance(boolean isBlue){
        int targetTagID = isBlue ? Constants.VisionConstants.aprilTagIDConstants.BLUE_SPEAKER_TAG_ID : Constants.VisionConstants.aprilTagIDConstants.RED_SPEAKER_TAG_ID;
        limelightNetworkTable.getEntry("pipline").setNumber(targetTagID);
    }

    private double calculateDistance(double verticalAngle){

        final double TARGET_HEIGHT = Constants.VisionConstants.limeLightDimensionConstants.TARGET_HEIGHT;
        final double CAMERA_HEIGHT = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_HEIGHT;
        final double CAMERA_PITCH = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_PITCH;


        double angleToSpeakerEntranceRadians = Math.toRadians( CAMERA_PITCH + verticalAngle);
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToSpeakerEntranceRadians);
    }

    @Override
    public void periodic(){
        updateLimelightTracking();
    }

    public void updateLimelightTracking() {
        limelightNetworkTable.getEntry("camMode").setNumber(0); // Sets the vision processing mode 
        limelightNetworkTable.getEntry("ledMode").setNumber(3); // Forces the LED to stay on always
    }
    
}
