
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
	Accelerometer accel;
	Timer myTimer;
	Compressor compress;
	Solenoid mySolenoid;
	Gyro myGyro;
	int autoLoopCounter;
	double[] accelx = new double[]{0,0,0,0,0};
    double[] accely = new double[]{0,0,0,0,0};
    
    double dtime;
    public boolean calcdist1 = true;
	
	
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
    	mySolenoid = new Solenoid(0);
    	myGyro = new Gyro(0);
    	
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
        while (accelx[4]<2 || accelx[4]>-2 || accely[4]<2 || accely[4]>-2) {
        	
        	myTimer.reset();
	    	accelx[0] = (Math.floor((accel.getX()*100)+5)/100)-0.055;
	    	accely[0] = (Math.floor((accel.getX()*100)+5)/100)-0.055;
	    	//Timer.delay(0.1);
        	dtime = myTimer.get();
        	//dtime = 0.001;
        	
        	//accelx[2] = ((accelx[0]-accelx[1])/2)*dtime+accelx[3];
	    	//accelx[4] = ((accelx[2]-accelx[3])/2)*dtime+accelx[4];
	    	
	    	accelx[2] = accelx[0]*dtime+accelx[1];
	    	accelx[4] = accelx[2]*dtime+accelx[3];
	    	
	    	//dtime = myTimer.get();
	    	
	    	//accely[2] = ((accely[0]-accely[1])/2)*dtime+accely[3];
	    	//accely[4] = ((accely[2]-accely[3])/2)*dtime+accely[4];
	    	
	    	accely[2] = accely[0]*dtime+accely[1];
	    	accely[4] = accely[2]*dtime+accely[3];
	    	
	    	accelx[1] = accelx[2];
	    	accelx[3] = accelx[4];
	    	
	    	accely[1] = accely[2];
	    	accely[3] = accely[4];
	    	
		    //System.out.println(accelx[4]);
		    //System.out.println(accely[4]);
		    
		    myRobot.mecanumDrive_Cartesian(0,0.3,0,0);
	    	
		    SmartDashboard.putNumber("x Acceleration", accelx[0]);
		    SmartDashboard.putNumber("y Acceleration", accely[0]);
		    SmartDashboard.putNumber("x PastVelocity", accelx[1]);
		    SmartDashboard.putNumber("y PastVelocity", accely[1]);
		    SmartDashboard.putNumber("x Velocity", accelx[2]);
		    SmartDashboard.putNumber("y Velocity", accely[2]);
		    SmartDashboard.putNumber("x PastDisplacement", accelx[3]);
		    SmartDashboard.putNumber("y PastDisplacement", accely[3]);
		    SmartDashboard.putNumber("x Displacement", accelx[4]);
		    SmartDashboard.putNumber("y Displacement", accely[4]);
	    	SmartDashboard.putNumber("Timer", dtime);
		    
		    /*if (stick.getRawButton(1) == true) {
		    	calcdist1 = false;
		    }*/
        }// end while loop
        myRobot.mecanumDrive_Cartesian(0,0 ,0,0);
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
        
        myRobot.mecanumDrive_Cartesian(-stick.getRawAxis(1),-stick.getRawAxis(2),stick.getRawAxis(4),0);
        
        
        
        
        
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
