package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    public static Intake intake = null;

    private CANSparkMax intakeMotor;
    private CANSparkMax handoffMotor;
    private DoubleSolenoid intakeExtender;
    

    private Intake(){
        intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        handoffMotor = new CANSparkMax(Constants.handoffMotorID, MotorType.kBrushless);
        intakeExtender = new DoubleSolenoid(Constants.intakeExtModuleID ,PneumaticsModuleType.REVPH , Constants.intakeExtFor, Constants.intakeExtRev);
    }

    public void setIntake(double intakePower){
        intakeMotor.set(intakePower);
    }

    public void setHandoff(boolean setHandoff){
        if(setHandoff == true){
            handoffMotor.set(Constants.handoffPower);
        } else{
            handoffMotor.set(0);
        }
        
    }

    public void setIntakeExt(boolean setIntake){
        if(setIntake == true){
            intakeExtender.set(Value.kForward);
        }else{
            intakeExtender.set(Value.kReverse);
        }
    }

    
    
    public static Intake getInstance(){
        if (intake == null){
            intake = new Intake();
        }
        return intake;
    }
}
