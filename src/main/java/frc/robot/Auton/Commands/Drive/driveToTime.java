package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class driveToTime extends Command{
    double leftSpeed;
    double rightSpeed;
    double time;

    Timer driveTimer = new Timer();

    Drive drive = Drive.getInstance();

    public driveToTime(double leftSpeed, double rightSpeed, double time){
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.time = time;
    }

    @Override
    public void initialize(){
        driveTimer.restart();
    }

    @Override
    public void execute(){
        drive.setSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
    }

    @Override
    public void end(boolean interrupt){
        drive.setSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
    }

    @Override
    public boolean isFinished(){
        return driveTimer.get() >= time;
    }
}
