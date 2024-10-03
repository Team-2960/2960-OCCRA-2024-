package frc.robot.subsystems;



import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase { 
    private static Drive drive = null;
    private CANSparkMax leftMotor1;
    private CANSparkMax leftMotor2;
    private CANSparkMax rightMotor1;
    private CANSparkMax rightMotor2;

    private Drive(){
        leftMotor1 = new CANSparkMax(Constants.leftMotor1ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(Constants.leftMotor2ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(Constants.rightMotor1ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(Constants.rightMotor2ID, MotorType.kBrushless);

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        leftMotor1.setInverted(true);
    }

    public void setPower(double rightPower, double leftPower){
        rightMotor1.set(rightPower);
        leftMotor1.set(leftPower);
    }

    public void setVoltagef(double rightVoltage, double leftVoltage){
        rightMotor1.setVoltage(rightVoltage);
        leftMotor1.setVoltage(leftVoltage);
    }

    public static Drive getInstance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }
    
}