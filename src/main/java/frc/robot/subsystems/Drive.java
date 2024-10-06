package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drive extends SubsystemBase { 
    private static Drive drive = null;
    private CANSparkMax leftMotor1;
    private CANSparkMax leftMotor2;
    private CANSparkMax rightMotor1;
    private CANSparkMax rightMotor2;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private Encoder encoder;

    private AHRS navx;

    private PIDController leftPID;
    private PIDController rightPID;

    private DifferentialDriveKinematics kinematics;

    private DifferentialDrivePoseEstimator poseEstimator;

    private SimpleMotorFeedforward feedforward;

    private PIDController leftPidController;
    private PIDController rightPidController;

    public AutoBuilder autoBuilder;

    

    private Drive(){
        leftMotor1 = new CANSparkMax(Constants.leftMotor1ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(Constants.leftMotor2ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(Constants.rightMotor1ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(Constants.rightMotor2ID, MotorType.kBrushless);

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        leftMotor1.setInverted(true);

        leftEncoder = leftMotor1.getEncoder(Type.kHallSensor, 42);
        rightEncoder = rightMotor1.getEncoder(Type.kHallSensor, 42);
        navx = new AHRS(SPI.Port.kMXP);
        navx.reset();

        kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
        
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, navx.getRotation2d(), 
            leftEncoder.getPosition() * Constants.wheelCirc, 
            rightEncoder.getPosition() * Constants.wheelCirc, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        feedforward = new SimpleMotorFeedforward(1, 3);
        leftPidController = new PIDController(1, 0, 0);
        rightPidController = new PIDController(1, 0, 0);

        autoBuilder = new AutoBuilder();
        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::presetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public void setPower(double rightPower, double leftPower){
        rightMotor1.set(rightPower);
        leftMotor1.set(leftPower);
    }

    public void setVoltage(double rightVoltage, double leftVoltage){
        rightMotor1.setVoltage(rightVoltage);
        leftMotor1.setVoltage(leftVoltage);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward =
            feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward =
            feedforward.calculate(speeds.rightMetersPerSecond);
    
        final double leftOutput =
            leftPidController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPidController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
        leftMotor1.setVoltage(leftOutput + leftFeedforward);
        rightMotor1.setVoltage(rightOutput + rightFeedforward);
    }

    public void drive(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        setSpeeds(wheelSpeeds);
    }

    public void presetPosition(Pose2d pose2d){
        poseEstimator.resetPosition(navx.getRotation2d(), 
            new DifferentialDriveWheelPositions(leftEncoder.getPosition() * Constants.wheelCirc, 
            rightEncoder.getPosition() * Constants.wheelCirc),
            pose2d);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds((leftEncoder.getVelocity() * Constants.wheelCirc)/60, rightEncoder.getVelocity()/60));
    }



    public static Drive getInstance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }
    
}