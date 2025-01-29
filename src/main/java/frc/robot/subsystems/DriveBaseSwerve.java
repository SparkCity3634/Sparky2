// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import Constants.java;

public class DriveBaseSwerve extends SubsystemBase {
  /** Creates a new DriveBaseSwerve. */
 
 //constants
  private final SparkMax BackRightDrive = new SparkMax(Constants.BackRightDriveCANId, MotorType.kBrushless);
  private final SparkMax BackRightTurn = new SparkMax(constants.BackRightTurnCANId, MotorType.kBrushless);
  private final SparkMax FrontRightDrive = new SparkMax(constants.FrontRightDriveCANId, MotorType.kBrushless);
  private final SparkMax FrontRightTurn = new SparkMax(constants.FrontRightTurnCANId, MotorType.kBrushless);
  private final SparkMax FrontLeftDrive = new SparkMax(constants.FrontLeftDriveCANId, MotorType.kBrushless);
  private final SparkMax FrontLeftTurn = new SparkMax(constants.FrontLeftTurnCANId, MotorType.kBrushless);
  private final SparkMax BackLeftDrive = new SparkMax(constants.BackLeftDriveCANId, MotorType.kBrushless);
  private final SparkMax BackLeftTurn = new SparkMax(constants.BackLeftTurnCANId, MotorType.kBrushless);
  
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
  
  private final CANcoder m_FrontRightTurnCancoder = new CANcoder(constants.FrontRightTurnCancoderId, constants.canBusName);
  private final CANcoder m_FrontLeftTurnCancoder = new CANcoder(constants.FrontLeftTurnCancoderId, constants.canBusName);
  private final CANcoder m_BackLeftTurnCancoder = new CANcoder(constants.BackLeftTurnCancoderId, constants.canBusName);
  private final CANcoder m_BackRightTurnCancoder = new CANcoder(constants.BackRightTurnCancoderId, constants.canBusName);

   //Bind Module controllers
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();

public final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(constants.m_BackLeftLocation, constants.m_BackRightLocation, constants.m_FrontLeftLocation, constants.m_FrontRightLocation);

//Create instance of fieldspeeds to store the ChassisSpeeds
ChassisSpeeds fieldspeeds = new ChassisSpeeds(0,0,0);

public DriveBaseSwerve() {
  //Create SparkMax Config for Drive motors
  SparkMaxConfig config_Drive = new SparkMaxConfig();
  
  //Declare Drive PID coefficients 

  config_Drive
    .smartCurrentLimit(40)
    .inverted(true)
    .idleMode(IdleMode.kCoast);
config_Drive.encoder
    .positionConversionFactor(constants.k_posConv)
    .velocityConversionFactor(constants.k_velConv);
config_Drive.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(constants.kdP, constants.kdI, constants.kdD)
    .iZone(constants.kdIz)
    .velocityFF(constants.kdFF)
    .outputRange(constants.kdMinOutput, constants.kdMaxOutput);

   /**
    * The PID Controller can be configured to use the analog sensor as its feedback
    * device with the method SetFeedbackDevice() and passing the PID Controller
    * the SparkMaxAnalogSensor object. 
    */
   
  SparkMaxConfig config_Turn = new SparkMaxConfig();
 
config_Turn
    .smartCurrentLimit(40)
    .inverted(true)
    .idleMode(IdleMode.kCoast);
config_Turn.encoder
    .positionConversionFactor(contsants.k_turnConv)
    .velocityConversionFactor(contsants.k_turnConv);
config_Turn.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(contsants.ktP, contsants.ktI, contsants.ktD)
    .iZone(contsants.ktIz)
    .velocityFF(contsants.ktFF)
    .outputRange(contsants.ktMinOutput, contsants.ktMaxOutput)
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

 public ChassisSpeeds CalcFieldSpeeds (Boolean isRobotRelative, double xAxis, double yAxis, double kYawRate) {

  // Get the x speed or forward/backward speed. We are inverting this because
// we want a positive value when we pull up. Xbox controllers
// return positive values when you pull down by default.
  final var xSpeed = constants.m_xspeedLimiter.calculate(xAxis)* constants.maxVel;

// Get the y speed or sideways/strafe speed. We are inverting this because
// we want a positive value when we pull to the left. Xbox controllers
// return positive values when you pull to the right by default.
final var ySpeed = constants.-m_yspeedLimiter.calculate(yAxis)* constants.maxVel;

// Get the rate of angular rotation. We are inverting this because we want a
// positive value when we pull to the left (remember, CCW is positive in
// mathematics). Xbox controllers return positive values when you pull to
// the right by default.
final var rot = contsants.-m_rotLimiter.calculate(kYawRate)* constants.maxYaw;

/*Calculate Swerve Module States based on controller readings
if left bumper is pressed Drive will be Robot Relative
otherwise Drive is Field Centric
*/

  if(isRobotRelative) {
    fieldspeeds =  new ChassisSpeeds(xAxis, yAxis, rot);

  }
  else {
    fieldspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()+180));
  }
return fieldspeeds;

  }
  public void DriveMotors(ChassisSpeeds fieldspeeds) {

    
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

