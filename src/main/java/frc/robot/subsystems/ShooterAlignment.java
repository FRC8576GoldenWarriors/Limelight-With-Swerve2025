// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Limelight.AprilTagStatsLimelight;

// public class ShooterAlignment extends Command{
//     private final Shooter shooterSubsystem;
//     private final PIDController pidController;

//     public ShooterAlignment(Shooter shooterSubystem){
//         this.shooterSubsystem = shooterSubystem;
//         this.pidController = new PIDController(0.1, 0.0, 0.0); // Make sure to tune these values later

//         addRequirements(shooterSubystem);
//     }

//     @Override
//     public void execute(){
//         double tx = AprilTagStatsLimelight.getTX();
//         double adjustment = pidController.calculate(tx, 0); // When tx = 0 the robot is aligned to the tag
//         shooterSubsystem.setPivotSpeed(adjustment);
//     }

//     @Override
//     public boolean isFinished(){
//         return false; // This will keep adjusting the shooter until the command is interrupted 
//     }

//     @Override
//     public void end(boolean interrupted){
//         shooterSubsystem.stop();
//     }
    
// }
