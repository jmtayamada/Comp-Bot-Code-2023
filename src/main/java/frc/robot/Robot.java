// ||  |\      /||  |:==\\    //==\\   ||===\\   ====||====      /\      |\   ||  ====||====
// ||  ||\    //||  |    ||  //    \\  ||    ||      ||         //\\     ||\  ||      ||    
// ||  ||\\  // ||  |:==//   ||    ||  ||\\=//       ||        //==\\    ||\\ ||      ||    
// ||  || \\//  ||  ||       \\    //  || \\         ||       //    \\   || \\||      ||    
// ||  ||  \/   ||  ||        \\==//   ||  \\        ||      //      \\  ||  \||      ||    

// Please keep this organized, put objects in the right sections
// use c_name for constants, and m_name for objects, if your object uses a port or string, 
// or something else in the create (ex: new WPI_TalonSRX(port)) use a constant for the port value


// for school wifi github changes, run command in terminal
// shutdown: tailscale down
// setup: tailscale up --exit-node 100.109.252.124


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class Robot extends TimedRobot {
  // basic constants
  final int c_DriverStickPort = 2; // port for driver stick
  final int c_HelperStickPort = 0; // port for helper stick
  final int c_LeftMotorPort = 1; // port for left motor
  final int c_LeftMotor2Port = 2; // port for left motor2
  final int c_RightMotorPort = 3; // port for right motor
  final int c_RightMotor2Port = 4; // port for right motor2
  // basic motors
  private final WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(c_LeftMotorPort);
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(c_LeftMotor2Port);
  private final WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(c_RightMotorPort);
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(c_RightMotor2Port);
  private final DifferentialDrive m_motors1 = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final DifferentialDrive m_motors2 = new DifferentialDrive(m_leftMotor2, m_rightMotor2);
  // sensors
  Timer m_timer = new Timer();
  // controllers
  private final Joystick m_driverStick = new Joystick(c_DriverStickPort);
  private final Joystick m_helperStick = new Joystick(c_HelperStickPort);
  // limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rock");     // this name may be different
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  // autonomous values
  double a_turnValue;
  double a_driveValue;
  boolean a_scored = false;
  // other
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_timer.start();
    if (a_scored == false) {
      if (tv.getDouble(0) == 1) {
        double RobotVerticalAngle = ty.getDouble(0);
        a_turnValue = -tx.getDouble(0)/54;
        if (ty.getDouble(0) > 19.25) {
          a_driveValue = (ty.getDouble(0)-19.5)/5.35;
        } else if(ty.getDouble(0) < 18.75) {
          a_driveValue = -(9.25+(-(ty.getDouble(0)-9.25)))/18.5;
        } else {
          a_driveValue = 0;
          // arm code goes here
          a_scored = true;
        }
        if (a_driveValue > 1) {
          a_driveValue = 1;
        } else if(a_driveValue < -1) {
          a_driveValue = -1;
        } 
        SmartDashboard.putNumber("drive", a_driveValue);
        SmartDashboard.putNumber("turn", a_turnValue);
        System.out.println("turn: "+a_turnValue);
        System.out.println("drive: "+a_driveValue);
        m_motors1.arcadeDrive(a_driveValue, a_turnValue, false);
        m_motors2.arcadeDrive(a_driveValue, a_turnValue, false);
        // leftMotor.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
        // leftMotor2.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
        // rightMotor.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
        // rightMotor2.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
      } else {
        if (m_timer.get() < .5) {
          m_motors1.arcadeDrive(0, -.5);  //.5
          m_motors2.arcadeDrive(0, -.5);   //.5
        } 
        if (m_timer.get() < 1.5 && m_timer.get() >= .5) {
          m_motors1.arcadeDrive(0, 0);
          m_motors2.arcadeDrive(0, 0);
        }
        if (m_timer.get() >= 1.5) {
          m_timer.reset();
        }
      }
    } else {

    }
  }
  

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
