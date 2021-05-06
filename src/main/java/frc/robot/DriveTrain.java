package frc.robot;

//import java.beans.VetoableChangeListenerProxy;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.math.*;

public class DriveTrain {
    private final static CANSparkMax motorBL = new CANSparkMax(Constants.BACKLEFT, MotorType.kBrushless);
    private final static CANSparkMax motorBR = new CANSparkMax(Constants.BACKRIGHT, MotorType.kBrushless);
    private final static CANSparkMax motorFL = new CANSparkMax(Constants.FRONTLEFT, MotorType.kBrushless);
    private final static CANSparkMax motorFR = new CANSparkMax(Constants.FRONTRIGHT, MotorType.kBrushless);
    private CANEncoder motor_encoderBL, motor_encoderBR, motor_encoderFR, motor_encoderFL;
	
	private CANPIDController motor_pidControllerBR, motor_pidControllerBL, motor_pidControllerFR, motor_pidControllerFL;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    //private static double velocity, RPM;
    private final double  WHEEL_DIAMETER = 4; // Inches
    private final double GEAR_RATIO = 8.16; // 8.16:1 
    private final double WHEEL_CIRC_INCH = WHEEL_DIAMETER * 3.141592;
    private final double WHEEL_CIRC_METERS = WHEEL_CIRC_INCH / 39.37;
    private final double VELOCITY_CONVERSION_FACTOR = (1/GEAR_RATIO) * WHEEL_CIRC_METERS/60;
    //private final double MAX_POWER = 0.5; 
    public final double DISTANCE_FROM_CENTER = 5/39.37;                     // Turn motors are situated 5 inches up/back and to the sides from the center 
    private final double ENCODER_COUNT_PER_DEGREE = 4096.0/360.0;               //Conversion factor for turning


    TalonSRX steerMotorFL = new TalonSRX(12);
    TalonSRX steerMotorFR = new TalonSRX(3);
    TalonSRX steerMotorBL = new TalonSRX(14);
    TalonSRX steerMotorBR = new TalonSRX(1);
    SwerveDriveKinematics kinematics;

    private Xbox xbox = new Xbox(0); 
    double targetPos = 0;
    double steerCalibrationMotorBL = 164;  //Motor #14
    double steerCalibrationMotorFL = 728;  //Motor #12
    double steerCalibrationMotorBR = -201;  //Motor #01
    double steerCalibrationMotorFR = 1094;  //Motor #03
    // PID parameter setup for Position Closed-Loop control
    // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#ch16-closedloop
    private double steerMotorKP = 5.0 * 1023 / 4096;  //100% power when error is 4096 (ie one full rotation) :  Math is motorpct * 1023 pid power range / 4096 encoder counts per rev
    private double steerMotorKI = 0.000002;
    private double steerMotorKD = 0.001;
    private double steerMotorKF = 0.0;
    private double reverseDrive = 0.0;      //Start as either a positive or negative 1


    public void init() {
        
        steerMotorFL.configFactoryDefault();             // Clear all motor controller settings
        //optional configurations examples
        steerMotorFL.setInverted(false);                 // Set to true if you need reverse motor direction (note: this also inverts sensor direction)
        steerMotorFL.setSensorPhase(false);              // Set to true if motor and sensor are running out of phase (ie opposite directions from each other)
        steerMotorFL.setNeutralMode(NeutralMode.Brake);  // Set to Brake or Coast
        steerMotorFL.configNeutralDeadband(0.0);        // Motor stops at power values lower than this value.  (Default is 0.04)
        //ramping for closed and open loop control
        steerMotorFL.configOpenloopRamp(0.0);              // Seconds from neutral to full power  (openloop is non-PID control)
        steerMotorFL.configClosedloopRamp(0.0);            // Seconds from neutral to full power  (this is for PID control)
        steerMotorFL.configPeakCurrentLimit(30);           // Set maximum current draw
        steerMotorFL.enableCurrentLimit(false);            // Must set to true to enable current limiting
        //
        steerMotorFL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // Choose the absolute sensor for PID feedback on PID 0, timeout of 10ms. 
        // 
        steerMotorFL.set(ControlMode.PercentOutput,0);   // Motor output can be between +/-1.0.  0=stop.
        // set PID
        steerMotorFL.config_kP(0,steerMotorKP);   //Set kP for PID0
        steerMotorFL.config_kI(0,steerMotorKI);   //Set kI for PID0
        steerMotorFL.config_kD(0,steerMotorKD);   //Set kD for PID0
        steerMotorFL.config_kD(0,steerMotorKF);   //Set kF for PID0
        
        steerMotorFR.configFactoryDefault();             // Clear all motor controller settings
        //optional configurations examples
        steerMotorFR.setInverted(false);                 // Set to true if you need reverse motor direction (note: this also inverts sensor direction)
        steerMotorFR.setSensorPhase(false);              // Set to true if motor and sensor are running out of phase (ie opposite directions from each other)
        steerMotorFR.setNeutralMode(NeutralMode.Brake);  // Set to Brake or Coast
        steerMotorFR.configNeutralDeadband(0.0);        // Motor stops at power values lower than this value.  (Default is 0.04)
        //ramping for closed and open loop control
        steerMotorFR.configOpenloopRamp(0.0);              // Seconds from neutral to full power  (openloop is non-PID control)
        steerMotorFR.configClosedloopRamp(0.0);            // Seconds from neutral to full power  (this is for PID control)
        steerMotorFR.configPeakCurrentLimit(30);           // Set maximum current draw
        steerMotorFR.enableCurrentLimit(false);            // Must set to true to enable current limiting
        //
        steerMotorFR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // Choose the absolute sensor for PID feedback on PID 0, timeout of 10ms. 
        // 
        steerMotorFR.set(ControlMode.PercentOutput,0);   // Motor output can be between +/-1.0.  0=stop.
        // set PID
        steerMotorFR.config_kP(0,steerMotorKP);   //Set kP for PID0
        steerMotorFR.config_kI(0,steerMotorKI);   //Set kI for PID0
        steerMotorFR.config_kD(0,steerMotorKD);   //Set kD for PID0
        steerMotorFR.config_kD(0,steerMotorKF);   //Set kF for PID0
        
        steerMotorBL.configFactoryDefault();             // Clear all motor controller settings
        //optional configurations examples
        steerMotorBL.setInverted(false);                 // Set to true if you need reverse motor direction (note: this also inverts sensor direction)
        steerMotorBL.setSensorPhase(false);              // Set to true if motor and sensor are running out of phase (ie opposite directions from each other)
        steerMotorBL.setNeutralMode(NeutralMode.Brake);  // Set to Brake or Coast
        steerMotorBL.configNeutralDeadband(0.0);        // Motor stops at power values lower than this value.  (Default is 0.04)
        //ramping for closed and open loop control
        steerMotorBL.configOpenloopRamp(0.0);              // Seconds from neutral to full power  (openloop is non-PID control)
        steerMotorBL.configClosedloopRamp(0.0);            // Seconds from neutral to full power  (this is for PID control)
        steerMotorBL.configPeakCurrentLimit(30);           // Set maximum current draw
        steerMotorBL.enableCurrentLimit(false);            // Must set to true to enable current limiting
        //
        steerMotorBL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // Choose the absolute sensor for PID feedback on PID 0, timeout of 10ms. 
        // 
        steerMotorBL.set(ControlMode.PercentOutput,0);   // Motor output can be between +/-1.0.  0=stop.
        // set PID
        steerMotorBL.config_kP(0,steerMotorKP);   //Set kP for PID0
        steerMotorBL.config_kI(0,steerMotorKI);   //Set kI for PID0
        steerMotorBL.config_kD(0,steerMotorKD);   //Set kD for PID0
        steerMotorBL.config_kD(0,steerMotorKF);   //Set kF for PID0
        
        steerMotorBR.configFactoryDefault();             // Clear all motor controller settings
        //optional configurations examples
        steerMotorBR.setInverted(false);                 // Set to true if you need reverse motor direction (note: this also inverts sensor direction)
        steerMotorBR.setSensorPhase(false);              // Set to true if motor and sensor are running out of phase (ie opposite directions from each other)
        steerMotorBR.setNeutralMode(NeutralMode.Brake);  // Set to Brake or Coast
        steerMotorBR.configNeutralDeadband(0.0);        // Motor stops at power values lower than this value.  (Default is 0.04)
        //ramping for closed and open loop control
        steerMotorBR.configOpenloopRamp(0.0);              // Seconds from neutral to full power  (openloop is non-PID control)
        steerMotorBR.configClosedloopRamp(0.0);            // Seconds from neutral to full power  (this is for PID control)
        steerMotorBR.configPeakCurrentLimit(30);           // Set maximum current draw
        steerMotorBR.enableCurrentLimit(false);            // Must set to true to enable current limiting
        //
        steerMotorBR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // Choose the absolute sensor for PID feedback on PID 0, timeout of 10ms. 
        // 
        steerMotorBR.set(ControlMode.PercentOutput,0);   // Motor output can be between +/-1.0.  0=stop.
        // set PID
        steerMotorBR.config_kP(0,steerMotorKP);   //Set kP for PID0
        steerMotorBR.config_kI(0,steerMotorKI);   //Set kI for PID0
        steerMotorBR.config_kD(0,steerMotorKD);   //Set kD for PID0
        steerMotorBR.config_kD(0,steerMotorKF);   //Set kF for PID0
        
        motorBL.restoreFactoryDefaults();						//clear settings on controller
        motorBR.restoreFactoryDefaults();
        motorFL.restoreFactoryDefaults();
        motorFR.restoreFactoryDefaults();
        
           	                                                    
		motorBL.setIdleMode(CANSparkMax.IdleMode.kCoast);						//set motors to coast (ie non-braking)
        motorBR.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motorFL.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motorFR.setIdleMode(CANSparkMax.IdleMode.kCoast);

        motorBL.setClosedLoopRampRate(0.35);
        motorBR.setClosedLoopRampRate(0.35);
        motorFL.setClosedLoopRampRate(0.35);
        motorFR.setClosedLoopRampRate(0.35);

        motor_pidControllerBR = motorBR.getPIDController();
        motor_pidControllerBL = motorBL.getPIDController();
        motor_pidControllerFR = motorFR.getPIDController();
        motor_pidControllerFL = motorFL.getPIDController();		//get PID objects

        motor_encoderBL = motorBL.getEncoder();
        motor_encoderBR = motorBR.getEncoder();
        motor_encoderFL = motorFL.getEncoder();
        motor_encoderFR = motorFR.getEncoder();					//get Encoder objects
		
        motor_encoderBL.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);			//set Velocity conversion factor
        motor_encoderBR.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        motor_encoderFL.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        motor_encoderFR.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        kP = 0.25; // Increasing this makes the interval between error correction different. It also gets stronger. = 0.0001
        kI = 0;
        kD = 0; // = 10
        kIz = 0; 
        kFF = 0.28; //0.00018 original value > Power = kFF x Target
        kMaxOutput = 1.0; 
        kMinOutput = -1.0;

        motor_pidControllerBL.setP(kP);
        motor_pidControllerBL.setI(kI);
        motor_pidControllerBL.setD(kD);
        motor_pidControllerBL.setIZone(kIz);
        motor_pidControllerBL.setFF(kFF);
        motor_pidControllerBL.setOutputRange(kMinOutput, kMaxOutput);
    
        motor_pidControllerBR.setP(kP);
        motor_pidControllerBR.setI(kI);
        motor_pidControllerBR.setD(kD);
        motor_pidControllerBR.setIZone(kIz);
        motor_pidControllerBR.setFF(kFF);
        motor_pidControllerBR.setOutputRange(kMinOutput, kMaxOutput);

        motor_pidControllerFR.setP(kP);
        motor_pidControllerFR.setI(kI);
        motor_pidControllerFR.setD(kD);
        motor_pidControllerFR.setIZone(kIz);
        motor_pidControllerFR.setFF(kFF);
        motor_pidControllerFR.setOutputRange(kMinOutput, kMaxOutput);

        motor_pidControllerFL.setP(kP);
        motor_pidControllerFL.setI(kI);
        motor_pidControllerFL.setD(kD);
        motor_pidControllerFL.setIZone(kIz);
        motor_pidControllerFL.setFF(kFF);
        motor_pidControllerFL.setOutputRange(kMinOutput, kMaxOutput);
  
        Translation2d translateFL = new Translation2d(DISTANCE_FROM_CENTER, DISTANCE_FROM_CENTER);
        Translation2d translateFR = new Translation2d(DISTANCE_FROM_CENTER, -DISTANCE_FROM_CENTER);
        Translation2d translateBL = new Translation2d(-DISTANCE_FROM_CENTER, DISTANCE_FROM_CENTER);
        Translation2d translateBR = new Translation2d(-DISTANCE_FROM_CENTER, -DISTANCE_FROM_CENTER);

        kinematics = new SwerveDriveKinematics(translateFL,translateFR,translateBL,translateBR);
    }
   
	private double getVel() {
        return motor_encoderBL.getVelocity();   // * 10; // Returns m/s after converting from tenths to seconds
    }
    
    public void setTarget(double metersPerSecond) {
        //Nigates the speed of the wheel by muiltiplying by -1.0
        motor_pidControllerBL.setReference(metersPerSecond, ControlType.kVelocity);
        motor_pidControllerFL.setReference(metersPerSecond, ControlType.kVelocity);
        motor_pidControllerBR.setReference(metersPerSecond, ControlType.kVelocity);
        motor_pidControllerFR.setReference(metersPerSecond, ControlType.kVelocity);
    }

    public void update() {

    }

    public void drive(double forwardVel,double translateVel,double rotateVel) {
        ChassisSpeeds speeds = new ChassisSpeeds(forwardVel, translateVel, Math.toRadians(rotateVel));
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState moduleFL = moduleStates[0];
        SwerveModuleState moduleFR = moduleStates[1];
        SwerveModuleState moduleBL = moduleStates[2];
        SwerveModuleState moduleBR = moduleStates[3];

        /*
        SmartDashboard.putNumber("Forward Vel", forwardVel);
        SmartDashboard.putNumber("Translate Vel", translateVel);
        SmartDashboard.putNumber("Rotate Vel (d/s)", rotateVel);
        SmartDashboard.putNumber("Front Left Speed", moduleFL.speedMetersPerSecond);
        SmartDashboard.putNumber("Front Right Speed", moduleFR.speedMetersPerSecond);
        SmartDashboard.putNumber("Back Left Speed", moduleBL.speedMetersPerSecond);
        SmartDashboard.putNumber("Back Right Speed", moduleBR.speedMetersPerSecond);
        SmartDashboard.putNumber("Front Left Angle", moduleFL.angle.getDegrees());
        SmartDashboard.putNumber("Front Right Angle", moduleFR.angle.getDegrees());
        SmartDashboard.putNumber("Back Left Angle", moduleBL.angle.getDegrees());
        SmartDashboard.putNumber("Back Right Angle", moduleBR.angle.getDegrees());
        SmartDashboard.putNumber("Front Right Encoder", steerMotorFR.getSelectedSensorPosition());
        SmartDashboard.putNumber("Front Left Encoder", steerMotorFL.getSelectedSensorPosition());
        SmartDashboard.putNumber("Front Right Target", steerMotorFR.getClosedLoopTarget());
        SmartDashboard.putNumber("Front Left Target", steerMotorFL.getClosedLoopTarget());
        */
        if (forwardVel == 0 && translateVel == 0 && rotateVel == 0) {
            motor_pidControllerBL.setReference(0, ControlType.kVelocity);           
            motor_pidControllerFL.setReference(0, ControlType.kVelocity);
            motor_pidControllerBR.setReference(0, ControlType.kVelocity);
            motor_pidControllerFR.setReference(0, ControlType.kVelocity);
        } else {
            steerMotorBL.set(ControlMode.Position, calcTurnAngle(moduleBL.angle.getDegrees(), steerMotorBL.getSelectedSensorPosition() - steerCalibrationMotorBL) + steerCalibrationMotorBL);
            motor_pidControllerBL.setReference(moduleBL.speedMetersPerSecond * reverseDrive, ControlType.kVelocity);
            steerMotorFL.set(ControlMode.Position, calcTurnAngle(moduleFL.angle.getDegrees(), steerMotorFL.getSelectedSensorPosition() - steerCalibrationMotorFL) + steerCalibrationMotorFL);
            motor_pidControllerFL.setReference(moduleFL.speedMetersPerSecond * reverseDrive, ControlType.kVelocity);
            steerMotorBR.set(ControlMode.Position, calcTurnAngle(moduleBR.angle.getDegrees(), steerMotorBR.getSelectedSensorPosition() - steerCalibrationMotorBR) + steerCalibrationMotorBR);
            motor_pidControllerBR.setReference(moduleBR.speedMetersPerSecond * reverseDrive, ControlType.kVelocity);
            steerMotorFR.set(ControlMode.Position, calcTurnAngle(moduleFR.angle.getDegrees(), steerMotorFR.getSelectedSensorPosition() - steerCalibrationMotorFR) + steerCalibrationMotorFR);
            motor_pidControllerFR.setReference(moduleFR.speedMetersPerSecond * reverseDrive, ControlType.kVelocity);
        }
    /**
        steerMotorBL.set(ControlMode.Position, (moduleBL.angle.getDegrees() + 180) * ENCODER_COUNT_PER_DEGREE + steerCalibrationMotorBL);
        steerMotorFL.set(ControlMode.Position, (moduleFL.angle.getDegrees() + 180) * ENCODER_COUNT_PER_DEGREE + steerCalibrationMotorFL);
        steerMotorBR.set(ControlMode.Position, (moduleBR.angle.getDegrees() + 180) * ENCODER_COUNT_PER_DEGREE + steerCalibrationMotorBR);
        steerMotorFR.set(ControlMode.Position, (moduleFR.angle.getDegrees() + 180) * ENCODER_COUNT_PER_DEGREE + steerCalibrationMotorFR);
    */
        
    

    }
/**
    public double efficiency(double currAngle, double tarAngle) {
        if (abs(currAngle - tarAngle) > abs(tarAngle - (currAngle + 360))) { // I don't think this is going to work but I wrote it anyways
            return (tarAngle - (currAngle + 360));
        } else {
            return (tarAngle);
        }
        
    }

    public double abs(double x) {
        if (x < 0) {
            return -x;
        } else {
            return x;
        }
    }*/

    public double calcTurnAngle(double targetSignedHeading, double currentEncoderAngle) {
        double encoderCompassHeading = (currentEncoderAngle % 4096) / ENCODER_COUNT_PER_DEGREE; 
        //System.out.println("Current Compass Heading: ");
        //System.out.println(encoderCompassHeading);
        double currentSignedHeading = 0;
        if(encoderCompassHeading > 180) {
            currentSignedHeading = encoderCompassHeading-360; 
            //System.out.println("Current Signed Heading: ");
            //System.out.println(currentSignedHeading);
        } else {
            currentSignedHeading = encoderCompassHeading;
            //System.out.println("Current Signed Heading: ");
            //System.out.println(currentSignedHeading);
        }
        
        double headingChange = targetSignedHeading - currentSignedHeading;
        if (headingChange > 180) {
            headingChange = headingChange - 360; 
        }
        if (headingChange < -180) {
            headingChange = -360 - headingChange; 
        }
        //now constrain heading change to no more than 90 degrees.
        if (Math.abs(headingChange) > 90) {
            if (headingChange >= 0) {
                headingChange = headingChange - 180;
                reverseDrive = -1.0;
            } else {
                headingChange = headingChange + 180;
                reverseDrive = -1.0;
            }
        } else {
            reverseDrive = 1.0;
        }

        //System.out.println("Heading Change: ");
        //System.out.println(headingChange);

        double angleChange = headingChange * ENCODER_COUNT_PER_DEGREE;
        double newEncoderAngle = currentEncoderAngle + angleChange;
        //System.out.println("Calculated Encoder Angle: ");
        //System.out.println(newEncoderAngle);
        return newEncoderAngle;
    }

}