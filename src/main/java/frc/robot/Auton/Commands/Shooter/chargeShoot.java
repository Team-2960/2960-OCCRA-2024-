package frc.robot.Auton.Commands.Shooter;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class chargeShoot extends Command{
    
    double shootVelocity;

    public chargeShoot(double shootVelocity){
        this.shootVelocity = shootVelocity;
    }

    @Override 
    public void execute(){
        Shooter.getInstance().setShooterSpeed(shootVelocity);;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
