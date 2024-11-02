package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class driveToRotation extends Command {
    double goalRotation;
    double tolerance;

    Drive drive = Drive.getInstance();

    public driveToRotation(double goalRotation, double tolerance){
        this.goalRotation = goalRotation;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        drive.driveToRotation(goalRotation);
    }

    @Override
    public void end(boolean interrupt){
        drive.setSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
    }

    @Override
    public boolean isFinished(){
        return Math.abs(goalRotation - drive.getPose().getRotation().getDegrees()) <= tolerance;
    }
}
