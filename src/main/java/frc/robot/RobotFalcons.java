// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Talons
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
/**
 * 
 */
public class RobotFalcons extends TimedRobot {

  private final double HEIGHT_PER_PULSE = (1.562 * 1e-4); // (1/6400)

  private TalonSRX mTalon;
  private XboxController mController;

  private int desiredPosition;

  private double KStatic = 0.1;
  


  /**
   * 
   */
  @Override
  public void robotInit() {
    int port = 0;
    mTalon = new TalonSRX(port);

    // What is a PID index?
    mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    // Constants defined in example 
    double kF = 0, kP = 0, kI = 0, kD = 0;
    int motionMagicSlot = 0; //What??
    mTalon.config_kP(motionMagicSlot,kP,30);
    mTalon.config_kI(motionMagicSlot,kI,30);
    mTalon.config_kD(motionMagicSlot,kD,30);

    // Set max velocity (sensor units (1/4096th rotation per 100ms))
    final double SENSORY_UNITS_TO_ROTATIONS_PER_SECOND = ((1.0 / 4096.0) / 0.100);
    double maxVelocity = (SENSORY_UNITS_TO_ROTATIONS_PER_SECOND * 1500.0) / 60.0; // 1500 rpm
    double maxAcceleration = (SENSORY_UNITS_TO_ROTATIONS_PER_SECOND * 2000.0) / 60.0; // 2000 rp(m^2)

    // Smart motion is the PID over a longer period of time, which allows
    // a maxiumum velocity and maximum acceleration to make a gradual movement
    
    // set max velocity and max acceleration
    mTalon.configMotionCruiseVelocity(maxVelocity, 30);
    mTalon.configMotionAcceleration(maxAcceleration, 30);

    double start = 0;

    mTalon.setSelectedSensorPosition(((int) start), 0, 30);
    
   
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    desiredPosition = 5; // feet 

    if (mController.getBButton())
    {
      desiredPosition = (-1) * desiredPosition;
    }
    
    double desiredPosition_in_pulses = (int) Math.round(desiredPosition / HEIGHT_PER_PULSE);

    mTalon.set(ControlMode.MotionMagic, desiredPosition_in_pulses, 
               DemandType.ArbitraryFeedForward, KStatic);


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
