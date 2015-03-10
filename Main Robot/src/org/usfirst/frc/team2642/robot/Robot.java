
package org.usfirst.frc.team2642.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2642.robot.commands.ExampleCommand;
import org.usfirst.frc.team2642.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	RobotDrive myRobot;
	Joystick stick;
	ADXL345_I2C accel;
	ADXL345_I2C.Allaxes accelerations;
	Timer myTimer;
	Compressor compress;
	Solenoid solenoid;
	Gyro gyro;
	int autoLoopCounter;
	double[] accel = new double[]{0,0,0,0};
    	double dtime;
	public boolean calcdist1 = true;
	final int frontLeftChannel = 2;
    final int rearLeftChannel	= 3;
    final int frontRightChannel	= 1;
    final int rearRightChannel	= 0;
    double Kp = .03;
    int crabcount = 0;
    double gyroset;
	
	
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

    Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		oi = new OI();
        // instantiate the command used for the autonomous period
        autonomousCommand = new ExampleCommand();
        
        myRobot = new RobotDrive(2,3,1,0);
        myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
    	myRobot.setInvertedMotor(MotorType.kRearLeft, true);
    	stick = new Joystick(0);
    	accel = new BuiltInAccelerometer();
    	accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
    	myTimer = new Timer();
    	compress = new Compressor(0);
    	solenoid = new Solenoid(0);
    	gyro = new Gyro(0);
    	
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();

        myTimer.start();
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        while (accel[3]<1 || accel[3]>-1) {
        	myTimer.reset();
	    	if (accel.getAcceleration(ADXL345_I2C.Axes.kX)>0.04 || accel.getAcceleration(ADXL345_I2C.Axes.kX)<-0.04) {
	    		accel[1] = accel.getAcceleration(ADXL345_I2C.Axes.kX);
	    	}else{
	    		accel[1] = 0;
	    	}
	    	Timer.delay(0.1);
        	dtime = 0.1;
        	accel[3] = accel[2]*dtime+(.25)*(accel[0]+accel[1])*dtime*dtime;
        	accel[2] = (0.5)*(accel[0]+accel[1])*dtime;
	    	accel[0] = accel[1];
		    
		    myRobot.mecanumDrive_Cartesian(0,0.3,0,0);
	    	
		    SmartDashboard.putNumber("PastAcceleration", accel[0]);
		    SmartDashboard.putNumber("Acceleration", accel[1]);
		    SmartDashboard.putNumber("Velocity", accel[2]);
		    SmartDashboard.putNumber("Displacement", accel[3]);
	    	    SmartDashboard.putNumber("Timer", dtime);
		    
        }// end while loop
        myRobot.mecanumDrive_Cartesian(0,0,0,0);
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        
        
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        
        if(!stick.getRawButton(11)){
    		if (stick.getRawButton(1)){
    		myRobot.mecanumDrive_Cartesian(stick.getAcceleration(ADXL345_I2C.Axes.kX), stick.getAcceleration(ADXL345_I2C.Axes.kY), stick.getTwist(), gyro.getAngle());
    		}else if(stick.getRawButton(2) && crabcount <=1){
    		gyroset = gyro.getAngle();
    		crabcount++;
    		}else if(stick.getRawButton(2) && (crabcount >= 2)){
    		myRobot.mecanumDrive_Cartesian(stick.getAcceleration(ADXL345_I2C.Axes.kX)/2, stick.getAcceleration(ADXL345_I2C.Axes.kY)/2, (gyro.getAngle() - gyroset) * Kp, gyro.getAngle()/2);
    		}else if(crabcount >= 1 && !stick.getRawButton(2)){
    		crabcount = 0;
    		}else {
    		myRobot.mecanumDrive_Cartesian(stick.getAcceleration(ADXL345_I2C.Axes.kX) /2, stick.getAcceleration(ADXL345_I2C.Axes.kY) /2, stick.getTwist() /2, gyro.getAngle());}
    	}else{
    		myRobot.mecanumDrive_Cartesian(stick.getAcceleration(ADXL345_I2C.Axes.kX), stick.getAcceleration(ADXL345_I2C.Axes.kY), stick.getTwist(), 0.0);

    	}
        
        
        
        
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
