package frc.robot.Auton.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class shootBall extends Command {
    @Override
    public void initialize(){
        Shooter.getInstance().setShooter(1);
    }

    @Override 
    public boolean isFinished(){
        boolean finished = Shooter.getInstance().isBallShot();
        return finished;
    }
}
