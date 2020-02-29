/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.Button;

//import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Mechanism;

// import java.io.IOException;
// import java.nio.file.Paths;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick m_stick = new Joystick(0);

//  private Button buttonA = new Button();

  private Mechanism intake = new Mechanism("button",0,5,6,1,2,50);
  private Mechanism conveyer = new Mechanism("button",0,7,8,3,4,50);
  private Mechanism shooter = new Mechanism("button",0,9,10,5,6,50);
  private Mechanism climb = new Mechanism("button", 0,11,12,7,8,50);
  /*
  BUTTONS:
  A - 1
  B - 2
  X - 3
  Y - 4
  L Shoulder Buttons - 5
  R Shoulder Button - 6
  Middle Button (L) - 7
  Middle Button ® - 8
  */

  // private Mechanism Intake = new Mechanism("button",0,5,6,1,2,50);
  //private final Timer m_timer = new Timer();
  //CANSparkMax l_motor1 = new CANSparkMax(1, MotorType.kBrushless);
  //CANSparkMax r_motor1 = new CANSparkMax(2, MotorType.kBrushless);
  //CANSparkMax l_motor2 = new CANSparkMax(3, MotorType.kBrushless);
  //CANSparkMax r_motor2 = new CANSparkMax(4, MotorType.kBrushless);
  //CANSparkMax l_flywheel = new CANSparkMax(5, MotorType.kBrushless);
  //CANSparkMax r_flywheel = new CANSparkMax(6, MotorType.kBrushless);
   //test
  
  
 /* public Spark getSpark(int motor)
  {
    switch(motor)
    {
      case 0:
        return l_motor1;
      case 1:
        return r_motor1;
      case 2:
        return l_motor2;
      default:
        return r_motor2;
      
    }
  }*/
 // DifferentialDrive flywheel = new DifferentialDrive(l_flywheel, r_flywheel);
 RobotContainer rc = new RobotContainer(); 
//  RamseteCommand c; 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    // try {
    //Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/YourPath.wpilib.json"));
    // }
    // catch (IOException e)
    // {}
    rc.getAutonomousCommand();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
   /* switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
         // Drive for 2 seconds
        if (m_timer.get() < 2.0) {
      //    m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
      } else {
       //   m_robotDrive.stopMotor(); // stop robot
      }
        break;
    }*/
  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {

    //establishes minimum and maximums of deadzone
    final double deadZone=0.5;
    final double minZone=0.07;

    //gets joystick values and creates curves
    double y = m_stick.getRawAxis(1);
    double yprime = -Math.pow(y,3);
      // double yprime=-y;
    double x = m_stick.getRawAxis(4);
    double xprime = Math.pow(x,3);
      // double xprime=x;
    
    //The % power used
    final double speedLimit = 0.8;
    final double turnLimit=0.5;

    //Reports joystick numbers
    DriverStation.reportWarning("Y,X: "+((Double)yprime).toString()+","+((Double)xprime).toString(),true);
    
    //Button stuff

      //Button A
      if(true)
      {
  
      }







    //Mathmomagic! For X and Y
    if(minZone<Math.abs(yprime)){
      // yprime=deadZone*Math.signum(yprime);
        yprime=Math.abs(yprime)/yprime*(deadZone+speedLimit*(1-deadZone)*(Math.abs(y)-0.1)/0.9);
        DriverStation.reportWarning("BANANA"+((Double)Math.abs(yprime)).toString(),true);
    }
    else{
      yprime=0;
    }
    if(minZone<Math.abs(xprime)){
      xprime=Math.abs(xprime)/xprime*(deadZone+turnLimit*(1-deadZone)*(Math.abs(x)-0.1)/0.9);
        DriverStation.reportWarning("KIWI"+((Double)Math.abs(xprime)).toString(),true);
    }
    else{
      xprime=0;
    }

    //Actual drive part
    rc.m_robotDrive.arcadeDrive(yprime, xprime);

    






    // if(0<m_stick.getRawAxis(4) && m_stick.getRawAxis(4)<-deadZone){
    //   x=deadZone;
    // }
    // else if(0>m_stick.getRawAxis(4) && m_stick.getRawAxis(4)>-deadZone){
    //   x=-deadZone;
    // }
    //if(m_stick.getRawAxis(4)>10)
    // flywheel.arcadeDrive(1,0);
     intake.run();
     conveyer.run();
     shooter.run();
     climb.run();

    //  if(m_stick.getRawButton(1)){
    //    if(Limelight.isTarget())
    //    rc.m_robotDrive.arcadeDrive(0,Limelight.getTx()*.1);
    //    else
    //   rc.m_robotDrive.arcadeDrive(0, 0);
    //  }
    // else
    // rc.m_robotDrive.arcadeDrive(-Math.pow(y,3)*.8, Math.pow(x,3)*.8);

    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}