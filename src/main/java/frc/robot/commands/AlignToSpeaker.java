package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight.SpeakerAllignment;

public class AlignToSpeaker extends Command {
    private final SpeakerAllignment speakerAllignment;
    

    public AlignToSpeaker(SpeakerAllignment speakerAllignment){
        this.speakerAllignment = speakerAllignment;
        addRequirements(speakerAllignment);
    }

    // @Override
    // public void initialize(){
    //     speakerAllignment.configureAliance(alliance == Alliance.Blue);
    // }

    @Override
    public void execute(){
        speakerAllignment.alignWithSpeaker();
    }

    @Override
    public void end(boolean interrupted){
        speakerAllignment.swerveDrive.stopModules();
    }

    @Override
    public boolean isFinished(){
        return speakerAllignment.isAlignedWithSpeaker();
    }
}
