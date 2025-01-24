// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import java.lang.Math;
import com.ctre.phoenix6.hardware.CANcoder;

public class DriveBaseSwerve extends SubsystemBase {
  /** Creates a new DriveBaseSwerve. */
 
  private final SparkMax BackRightDrive = new SparkMax(41, MotorType.kBrushless);
  private final SparkMax BackRightTurn = new SparkMax(42, MotorType.kBrushless);
  private final SparkMax FrontRightDrive = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax FrontRightTurn = new SparkMax(12, MotorType.kBrushless);
  private final SparkMax FrontLeftDrive = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax FrontLeftTurn = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax BackLeftDrive = new SparkMax(31, MotorType.kBrushless);
  private final SparkMax BackLeftTurn = new SparkMax(32, MotorType.kBrushless);
  
   //create SparkClosedLoopControllers to fix Drive and Turn motors

   private SparkClosedLoopController m_BackLeftDrivePID = BackLeftDrive.getClosedLoopController();
   private SparkClosedLoopController m_BackRightDrivePID = BackRightDrive.getClosedLoopController();
   private SparkClosedLoopController m_FrontRightDrivePID = FrontLeftDrive.getClosedLoopController();
   private SparkClosedLoopController m_FrontLeftDrivePID = FrontRightDrive.getClosedLoopController();
   private SparkClosedLoopController m_BackLeftTurnPID = BackLeftTurn.getClosedLoopController();
   private SparkClosedLoopController m_BackRightTurnPID = BackRightTurn.getClosedLoopController();
   private SparkClosedLoopController m_FrontRightTurnPID = FrontLeftTurn.getClosedLoopController();
   private SparkClosedLoopController m_FrontLeftTurnPID = FrontRightTurn.getClosedLoopController();
   
     
  //create Relative encoders (from built-in motor encoders) for Turn and Drive motors
  private RelativeEncoder m_BackLeftTurnEncoder = BackLeftTurn.getEncoder();
  private RelativeEncoder m_BackRightTurnEncoder = BackRightTurn.getEncoder();
  private RelativeEncoder m_FrontRightTurnEncoder = FrontRightTurn.getEncoder();
  private RelativeEncoder m_FrontLeftTurnEncoder = FrontLeftTurn.getEncoder();
  
  //Bind CANcoders for Absolute Turn Encoding
  private static final String canBusName = "rio";
  private final CANcoder m_FrontRightTurnCancoder = new CANcoder(10, canBusName);
  private final CANcoder m_FrontLeftTurnCancoder = new CANcoder(20, canBusName);
  private final CANcoder m_BackLeftTurnCancoder = new CANcoder(30, canBusName);
  private final CANcoder m_BackRightTurnCancoder = new CANcoder(40, canBusName);

   //Bind Module controllers
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();

   //Create Variables to store PID coefficients and other config parameters
   public double maxRPM, setTurn;
   public double kdP, kdI, kdD, kdIz, kdFF, kdMaxOutput, kdMinOutput;
   public double ktP, ktI, ktD, ktIz, ktFF, ktMaxOutput, ktMinOutput;
   public double ksP, ksI, ksD, ksIz, ksFF, ksMaxOutput, ksMinOutput;
   public double xAxis, yAxis, kYawRate, maxVel, maxYaw, o_lyAxis, o_ryAxis;
   public double k_posConv = .048; // linear meters traveled at wheel, per motor rotation
   public double k_velConv = .0008106; // linear meters per second (m/s) speed at wheel, convert from motor RPM
   public double k_turnConv = 16.8; //wheel degrees per motor rotation from steering relative encoder
   public double rotations;
   
   //Create Swerve Kinematics modules
Translation2d m_BackLeftLocation = new Translation2d(-0.25 ,0.250);
Translation2d m_BackRightLocation = new Translation2d(-0.25 ,-.250);
Translation2d m_FrontLeftLocation = new Translation2d(.250,.250);
Translation2d m_FrontRightLocation = new Translation2d(.250,-.250);

// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

public final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_BackLeftLocation, m_BackRightLocation, m_FrontLeftLocation, m_FrontRightLocation);

//Create instance of fieldspeeds to store the ChassisSpeeds
ChassisSpeeds fieldspeeds = new ChassisSpeeds(0,0,0);

public DriveBaseSwerve() {
  //Create SparkMax Config for Drive motors
  SparkMaxConfig config_Drive = new SparkMaxConfig();
  
  //Declare Drive PID coefficients 

  
  kdP = 0.4; 
  kdI = 0.00000;
  kdD = 0.05; 
  kdIz = 0; 
  kdFF = 0.01; 
  kdMaxOutput = 1; 
  kdMinOutput = -1;

  config_Drive
    .smartCurrentLimit(40)
    .inverted(true)
    .idleMode(IdleMode.kCoast);
config_Drive.encoder
    .positionConversionFactor(k_posConv)
    .velocityConversionFactor(k_velConv);
config_Drive.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(kdP, kdI, kdD)
    .iZone(kdIz)
    .velocityFF(kdFF)
    .outputRange(kdMinOutput, kdMaxOutput);

      //Create SparkMax Config for Turn motors
   /**
    * The PID Controller can be configured to use the analog sensor as its feedback
    * device with the method SetFeedbackDevice() and passing the PID Controller
    * the SparkMaxAnalogSensor object. 
    */
  SparkMaxConfig config_Turn = new SparkMaxConfig();
  
  // Turn PID coefficients
  ktP = 0.07; 
  ktI = 0.0000;
  ktD = 0.001;
  ktIz = 0; 
  ktFF = 0.003; 
  ktMaxOutput = .9; 
  ktMinOutput = -.9;

config_Turn
    .smartCurrentLimit(40)
    .inverted(true)
    .idleMode(IdleMode.kCoast);
config_Turn.encoder
    .positionConversionFactor(k_turnConv)
    .velocityConversionFactor(k_turnConv);
config_Turn.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ktP, ktI, ktD)
    .iZone(ktIz)
    .velocityFF(ktFF)
    .outputRange(ktMinOutput, ktMaxOutput)
    .positionWrappingEnabled(true)
    .positionWrappingMinInput(0)
    .positionWrappingMaxInput(360);

    
    //Set Configuration for Drive motors
    BackLeftDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BackRightDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontLeftDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontRightDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Set Configuration for Turn motors
    BackLeftTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BackRightTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontLeftTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontRightTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    /* Speed up signals to an appropriate rate */
    m_FrontRightTurnCancoder.getPosition().setUpdateFrequency(100);
    m_FrontRightTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_FrontLeftTurnCancoder.getPosition().setUpdateFrequency(100);
    m_FrontLeftTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_BackLeftTurnCancoder.getPosition().setUpdateFrequency(100);
    m_BackLeftTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_BackRightTurnCancoder.getPosition().setUpdateFrequency(100);
    m_BackRightTurnCancoder.getVelocity().setUpdateFrequency(100);

     
    //Initialize RelativeEncoders to CANCoder Absolute Value
    m_FrontRightTurnEncoder.setPosition(m_FrontRightTurnCancoder.getAbsolutePosition().getValueAsDouble());//might want to add the .waitForUpdate() method to reduce latency?
    m_FrontLeftTurnEncoder.setPosition(m_FrontLeftTurnCancoder.getAbsolutePosition().getValueAsDouble());
    m_BackLeftTurnEncoder.setPosition(m_BackLeftTurnCancoder.getAbsolutePosition().getValueAsDouble());
    m_BackRightTurnEncoder.setPosition(m_BackRightTurnCancoder.getAbsolutePosition().getValueAsDouble());

    maxVel = 4; // m/s linear velocity of drive wheel
    maxYaw = 2*Math.PI;   // max rad/s for chassis rotation rate
      
    //Set Initial Swerve States
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState BackLeftSwerve = moduleStates[0];
    SwerveModuleState BackRightSwerve = moduleStates[1];
    SwerveModuleState FrontLeftSwerve = moduleStates[2];
    SwerveModuleState FrontRightSwerve = moduleStates[3];

       //Optimize Swerve Module States using Instance Method
       BackLeftSwerve.optimize(Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
       BackRightSwerve.optimize(Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
       FrontLeftSwerve.optimize(Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
       FrontRightSwerve.optimize(Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
       
       m_gyro.reset();

}

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public void DriveMotors(Boolean isRobotRelative, double xAxis, double yAxis, double rot) {

    if(m_controller.getLeftBumperButton()){
      fieldspeeds =  new ChassisSpeeds(xAxis, yAxis, rot);
    }
    else {
      fieldspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()+180));
    }
  SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(fieldspeeds);
  SwerveModuleState BackLeftSwerve = moduleStates[0];
  SwerveModuleState BackRightSwerve = moduleStates[1];
  SwerveModuleState FrontLeftSwerve = moduleStates[2];
  SwerveModuleState FrontRightSwerve = moduleStates[3];
  
  //Optimize Swerve Module States using Deprecated Static Method
  //var BackLeftOptimized = SwerveModuleState.optimize(BackLeftSwerve, Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
  //var BackRightOptimized = SwerveModuleState.optimize(BackRightSwerve, Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
  //var FrontLeftOptimized = SwerveModuleState.optimize(FrontLeftSwerve, Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
  //var FrontRightOptimized = SwerveModuleState.optimize(FrontRightSwerve, Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
  
  //Optimize Swerve Module States using Instance Method
  BackLeftSwerve.optimize(Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
  BackRightSwerve.optimize(Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
  FrontLeftSwerve.optimize(Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
  FrontRightSwerve.optimize(Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
  
  /**
   * PIDController objects are commanded to a set point using the 
   * SetReference() method and the reference value of the optimized swerve mfodule states
   * 
   * The first parameter is the value of the set point, whose units vary
   * depending on the control type set in the second parameter.
   * 
   * The second parameter is the control type, can be set to one of four 
   * parameters:
   * com.revrobotics.SparkMax.ControlType.kDutyCycle
   *  com.revrobotics.SparkMax.ControlType.kPosition
   *  com.revrobotics.SparkMax.ControlType.kVelocity
   *  com.revrobotics.SparkMax.ControlType.kVoltage
   */
  
  //Set actual motor output values to drive

  //angle and SpeedMeter not
  m_BackRightTurnPID.setReference(BackRightSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);
  m_FrontRightTurnPID.setReference(FrontRightSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);
  m_FrontLeftTurnPID.setReference(FrontLeftSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);
  m_BackLeftTurnPID.setReference(BackLeftSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);  
  m_BackLeftDrivePID.setReference(BackLeftSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
  m_BackRightDrivePID.setReference(BackRightSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
  m_FrontLeftDrivePID.setReference(FrontLeftSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
  m_FrontRightDrivePID.setReference(FrontRightSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);

  }
}
