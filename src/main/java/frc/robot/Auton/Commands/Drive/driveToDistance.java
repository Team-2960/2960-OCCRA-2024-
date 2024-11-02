package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class driveToDistance extends Command {
    double leftDistance; //meters
    double rightDistance; //meters
    double leftInitial; //meters
    double rightInitial; //meters

    double tolerance; //meters

    Drive drive = Drive.getInstance();

    public driveToDistance(double leftDistance, double rightDistance, double tolerance){
        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.tolerance = tolerance;
    }

    @Override 
    public void initialize(){
        leftInitial = drive.getLeftPosition();
        rightInitial = drive.getRightPosition();
    }

    @Override
    public void execute(){
        drive.driveDistance(leftDistance, rightDistance, leftInitial, rightInitial);
    }

    @Override
    public void end(boolean interrupt){
        drive.setSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
    }

    @Override
    public boolean isFinished(){
        boolean isLeftFinished = Math.abs((leftInitial + leftDistance) - (leftInitial + drive.getLeftPosition())) <= tolerance;
        boolean isRightFinished = Math.abs((rightInitial + rightDistance) - (rightInitial + drive.getRightPosition())) <= tolerance;

        return isLeftFinished && isRightFinished;
        
    }


}
