package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Xbox.buttons;



public class Robot extends TimedRobot {
  public static final double MAX_METERS_PER_SEC = 0.5;
  
  private double drive, translate, rotate; 
  private DriveTrain dt = new DriveTrain();
  private Xbox driver = new Xbox(0);

  @Override
  public void robotInit() {
  dt.init();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
   
    drive = driver.deadzone(-driver.getY(GenericHID.Hand.kLeft)) * MAX_METERS_PER_SEC; 
    translate = driver.deadzone(-driver.getX(GenericHID.Hand.kLeft)) * MAX_METERS_PER_SEC;
    rotate = (driver.deadzone(-driver.getX(GenericHID.Hand.kRight))) * 360;
    
    dt.drive(drive, translate, rotate);
    //Common.dashNum("Deadzone Value", drive);

  }

  @Override
  public void testPeriodic() {
  }

  private void update() {
    dt.setTarget(drive);
  }

  @Override
  public void disabledInit() {
    //dt.calcTurnAngle(30, (5000/360.0)*4096);
  }
}
