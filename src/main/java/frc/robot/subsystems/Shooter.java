package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter shooter = null;

    private CANSparkMax shooterMotor;
    private DigitalInput shooterPhotoeye;

    private Shooter(){
        shooterMotor = new CANSparkMax(Constants.shooterMotorID, MotorType.kBrushless);
        shooterPhotoeye = new DigitalInput(Constants.shooterPhotoeye);
    }

    public void setShooter(double setSpeed){
        if (shooterPhotoeye.get() == false){
            shooterMotor.set(setSpeed);
        }else{
            shooterMotor.set(0);
        }
    }

    public boolean isBallShot(){
        return shooterPhotoeye.get();
    }


    public static Shooter getInstance(){
        if (shooter == null){
            shooter = new Shooter();
        }
        return shooter;
    }
}
