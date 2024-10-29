// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auton.RobotContainer;
import frc.robot.Auton.Commands.AutonList;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   * 
   */
  Drive drive;
  OI oi;
  Shooter shooter;
  Intake intake;
  Pneumatics pneumatics;
  Optional<Command> autonomousCommand;
  RobotContainer robotContainer;

  Joystick controller1 = new Joystick(0);
  
  @Override
  public void robotInit() {
    drive = Drive.getInstance();
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    pneumatics = Pneumatics.getInstance();
    oi = OI.getInstance();
    robotContainer = new RobotContainer();
    autonomousCommand = AutonList.getDefaultCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    if (autonomousCommand.isPresent()) autonomousCommand.get().schedule();
    /*autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand == null){
      autonomousCommand.schedule();
    }*/
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
