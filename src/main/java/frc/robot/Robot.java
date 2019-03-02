package frc.robot;  // stuff..

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.cameraserver.CameraServer ;
//import edu.wpi.first.wpilibj.Servo;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogInput;
import java.util.ArrayList ;
import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.command.Command ;
import edu.wpi.first.wpilibj.PWMVictorSPX ;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;


import io.github.pseudoresonance.pixy2api.*;
//import io.github.pseudoresonance.pixy2api.links.I2CLink; ;
import io.github.pseudoresonance.pixy2api.links.* ;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block ;






/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name 
 * of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  boolean tiltingLeft = false ;
  boolean tiltingRight = false;
  boolean tippingForward = false ; 
  boolean tippingBackward = false ;
  double scootForwardclock = 0 ;
  double scootStart_time  = 0 ;
  double scootForwardclock2 = 0 ;
  double scootStart_time2  = 0 ;
  double raiseLegsClock = 0;
  double raiseLegsStart_time = 0 ;
  int climbphase = 0 ;
  double raisebackclock = 0;
  double raisebackStart_time = 0;
  double climbphase0timer = 0 ;
  double climbphase0Start_time =  0;
  boolean abort_climb = false ;
  double climbphase5timer = 0 ;
  double climbphase5Start_time =  0;
  boolean backLiftAtRaiseLimit = false ;
  double tiltingDegree = 0 ;
  double tippingDegree = 0 ;
  boolean climbprogram_running = false ;
  boolean climb_six_running = false ;
  boolean fully_raised = false ;
  double doRaiseTogetherStartTime = 0 ;
  double doRaiseTogetherLapTime = 0 ;
  boolean in_evening_phase = false ;
  double previous_navx_check = 0 ;


    // lowering phase:
    double lower_phase = 10 ;
    double lower_phase_10_timer = 0 ;
    double lower_phase_10_start_time = 0 ;
    double lower_phase_20_timer = 0 ;
    double lower_phase_20_start_time = 0 ;
    double lower_phase_30_timer = 0 ;
    double lower_phase_30_start_time = 0 ;
    double lower_phase_40_timer = 0 ;
    double lower_phase_40_start_time = 0 ;
    double lower_phase_50_timer = 0 ;
    double lower_phase_50_start_time = 0 ;
    double lower_phase_60_timer = 0 ;
    double lower_phase_60_start_time = 0 ;

 // climbing to 6" platform:
  double climb_six_phase = 10 ;
  double climb_six_phase_10_timer = 0 ;
  double climb_six_phase_10_start_time = 0 ;
  double climb_six_phase_20_timer = 0 ;
  double climb_six_phase_20_start_time = 0 ;
  double climb_six_phase_30_timer = 0 ;
  double climb_six_phase_30_start_time = 0 ;
  double climb_six_phase_40_timer = 0 ;
  double climb_six_phase_40_start_time = 0 ;



  double nudge_value = .5 ;
  double distance = 100 ;
  double throttle ;
  double turn_throttle ;
  boolean thumb = false ;
  double middleHeight ;

  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new Spark(0), new Spark(1));
  private final Joystick m_stick = new Joystick(0);  // EXTREME 3D
  private final XboxController xbox = new XboxController(1); // DUAL ACTION
  private final Timer m_timer = new Timer();
  Spark hatchmotor = new Spark(2);
  Spark bucketelevator = new Spark(3);


  Pixy2 pixy = Pixy2.createInstance(new SPILink());
  //Pixy2CCC pccc = Pixy2CCC.createInstance(new SPILink()) ;



  AnalogInput frontHeightSensor = new AnalogInput(3) ;
  AnalogInput backHeightSensor = new AnalogInput(2) ;
  AnalogInput middleHeightSensor = new AnalogInput(0) ;

  // analog 1
  //AnalogInput pixyA = new AnalogInput(1) ;

  AnalogInput myRangeFinder = new AnalogInput(1) ;


  //Spark compressor = new Spark(4);
  Relay compressorRelay = new Relay(0);
  //DigitalInput limitSwitch ;
  DigitalInput compressorSwitch = new DigitalInput(0); 
  DigitalInput backLiftLimitSwitch = new DigitalInput(1); 

  boolean bucket_zero_has_been_set = false ;
  boolean time_has_been_reset = false ;

  // ## RS7 encoder
  /*
  DigitalInput EncoderA = new DigitalInput(8);
  DigitalInput EncoderB = new DigitalInput(9);
  double countA = 0 ; double countB = 0 ;
  boolean encoderAlastvalue = false ; 
  boolean encoderBlastvalue = true ; */
  Encoder motorEnc = new Encoder (8,9);

  double R = 1 ;  // reverse mode tracker

  Spark frontLiftRight = new Spark(5);
  Spark frontLiftLeft = new Spark(6);
  

  PWMVictorSPX backlift = new PWMVictorSPX(7);
  PWMVictorSPX backdrive = new PWMVictorSPX(8);


  //Servo myservo = new Servo(2) ;
  AHRS ahrs = new AHRS(SerialPort.Port.kUSB);
  double previous_ahrs_bytecount ;
  boolean navx_online = false ;
  boolean climbmode = false ;
  double targetangle ;
  boolean turning ;
  double startangle ;
  boolean pointreached ;
  
  boolean hatch_up ;

    boolean reversemode  = false ; // drive backwards.

  DoubleSolenoid piston = new DoubleSolenoid(0,1);
  DoubleSolenoid piston2 = new DoubleSolenoid(2,3);

  //DigitalInput pix = new DigitalInput(1) ; 

  UsbCamera RightCamera ;
  UsbCamera LeftCamera ;

  double basket_piston_fired_start_time = 0 ;

  float FLATpitch ;
  float FLATroll ;
  float FLATyaw ;
  boolean flat_has_been_set = false ;

  // * straight line driving variables
  boolean straightlinedrivingmode = false ;
  double straightangle = 0 ; 
  double steervalue = 0 ;
  boolean turningright = false ;
  boolean turningleft = false ;

  NetworkTableEntry sometablevalue  ;
  NetworkTableEntry blah ;
  NetworkTableEntry camLeftStream ;
  NetworkTableEntry camRightStream ;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // load up all 4 cameras
    CameraServer.getInstance().startAutomaticCapture("Left",2);
    CameraServer.getInstance().startAutomaticCapture("Right",3);
    CameraServer.getInstance().startAutomaticCapture("Left",0);
    CameraServer.getInstance().startAutomaticCapture("Right",1);

    /* network tables playing around:
    NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
    NetworkTable table = inst.getTable("datatable");
    sometablevalue = table.getEntry("MMYTEST");
    sometablevalue.setDouble(5);
    NetworkTable tableSD = inst.getTable("SmartDashboard");
    blah = tableSD.getEntry("TIMER:");
    */

    NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
    NetworkTable CamPub = inst.getTable("CameraPublisher");
    camLeftStream = CamPub.getEntry("Left/streams") ;
    camRightStream = CamPub.getEntry("Right/streams") ;

    pixy.init(); 


  }


  @Override
  public void autonomousInit() {     
    m_timer.reset();  
    m_timer.start();   
    time_has_been_reset = true ;
    climbmode = false ;

    // assume we start on flat ground.
    flat_has_been_set = true ;
    FLATpitch = ahrs.getPitch(); 
    FLATroll = ahrs.getRoll();  
    FLATyaw = ahrs.getYaw();

    bucket_zero_has_been_set = true ;
    motorEnc.reset();

    basket_piston_fired_start_time = 0 ;

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

        //  button 4 on xbox control determines if climb mode is on or off.
        if (xbox.getRawButton(4)) {  
          if ( ! climbmode) { L("A: switched into climbmode.");}
          climbmode = true ;  
        } else { 
          if ( climbmode) { L("A: switched out of climbmode.");}
          climbmode = false ; 
          backlift.set(0) ;
          backdrive.set(0);
          frontLiftLeft.set(0);
          frontLiftRight.set(0) ;
        }
        SmartDashboard.putBoolean("CLIMB MODE:", climbmode);



    thumb = m_stick.getRawButton(2);
    do_airCompressor();
    do_bucket_encoder() ;
    navx_check() ;
    do_distance_sensors() ;
    do_reverse_mode() ;
    get_throttle_value() ;

    if (! climbmode) {
      SLD() ; // straight line driving.
      normal_driving() ;
      pixy_driving() ;
      pov_driving() ;
      shoot_ball_from_bucket() ;
      do_hatch_piston() ;
      do_hatch_motor() ;
      do_bucket_elevator() ;
    }


    // if we are IN climbmode, since this is autonomous, it can only mean we are climbing 
    // off of the 6" platoform.
    if (climbmode) {
      lower_from_six() ;
    }
    


        // -----------------------------------------------------
    // show the timers:
    SmartDashboard.putNumber("TIMER:",m_timer.get());
    double timeleft = 150 - Math.round(m_timer.get());
    SmartDashboard.putNumber("TIME LEFT:",timeleft);

  }

  // ******************************************************************** //
  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    try { ahrs.resetDisplacement() ; } finally {};
    hatch_up = false ;
    if (! time_has_been_reset) { m_timer.reset();  m_timer.start();}
    piston.set(DoubleSolenoid.Value.kReverse);


    if (! flat_has_been_set) {
        FLATpitch = ahrs.getPitch(); 
        FLATroll = ahrs.getRoll();  
        FLATyaw = ahrs.getYaw();
    }
    if (! bucket_zero_has_been_set) {
        motorEnc.reset();
    }


    // climbing stuff
    climbmode = false ;
    scootForwardclock = 0 ;
    scootStart_time  = 0 ;
    scootForwardclock2 = 0 ;
    scootStart_time2  = 0 ;
    raiseLegsClock = 0;
    raiseLegsStart_time = 0 ;
    climbphase = 0 ;
    raisebackclock = 0;
    raisebackStart_time = 0;
    climbphase0timer = 0 ;
    climbphase0Start_time =  0;
    abort_climb = false ;
    climbphase5timer = 0 ;
    climbphase5Start_time =  0;
    doRaiseTogetherStartTime = 0 ;
    doRaiseTogetherLapTime = 0 ;
    in_evening_phase = false ;
    navx_check() ;

    basket_piston_fired_start_time = 0 ;

    previous_navx_check = m_timer.get() ;

  }



  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    // -----------------------------------------------------
    // show the timers:
    SmartDashboard.putNumber("TIMER:",m_timer.get());  double timeleft = 150 - Math.round(m_timer.get());
    SmartDashboard.putNumber("TIME LEFT:",timeleft);

    //  button 4 on xbox control determines if climb mode is on or off.
    if (xbox.getRawButton(4)) {  
      if ( ! climbmode) { L("switched into climbmode.");}
      climbmode = true ;  
    } else { 
      if ( climbmode) { L("switched out of climbmode.");}
      climbmode = false ; 
    }
    SmartDashboard.putBoolean("CLIMB MODE:", climbmode);


    thumb = m_stick.getRawButton(2);


    // Check if navigation is possible.
    navx_check() ;

    // AIR compressor  ----------------------------------------------
    do_airCompressor();


    // ULTRA sensors, under the robot, used to see if we've moved over after climbing.
    do_distance_sensors() ;


    if (! climbmode) {
      do_reverse_mode() ;
      get_throttle_value() ;
      SLD() ;   // straight line driving.
      do_bucket_encoder() ;
      normal_driving() ;
      pixy_driving() ;
      pov_driving() ;
      shoot_ball_from_bucket() ;
      do_hatch_piston() ;
      do_hatch_motor() ;
      do_bucket_elevator() ;
    }

    if (! climbmode) {  // NOT in CLIMBING MODE
      backlift.set(0) ;
      backdrive.set(0) ;
      frontLiftRight.set(0) ;
      frontLiftLeft.set(0) ;
      climbprogram_running = false ;
      climb_six_running = false ;
      return ;  //  return since everything that follows is about climbing.
    }


    //  CLIMBING ---------------------------------

    gather_climb_data ();



  // Climbing Program.
  double Xpov = xbox.getPOV() ;
  if (Xpov == 0 && navx_online && ! abort_climb){
    if (! climbprogram_running) { L("climb program turned on.");}
    climbprogram_running = true ;
  } else if (Xpov == 180 ){
    if (climb_six_running) { L("climb_six program turned on.");}
    climb_six_running = true ;
  
  }

  if (climbprogram_running && ! abort_climb) {
    climbingprogram() ;
    return ;
  }
  if (climb_six_running && ! abort_climb) {
    climb_six();
    return ;
  }


  // Gyro raise or lower:
  boolean raiseALL = xbox.getRawButton(5); 
  boolean lowerALL = xbox.getRawButton(7);
  if (! raiseALL && ! lowerALL) {
    manual_climbing();
    return ;
  }

  if (! navx_online) { 
    backlift.set(0) ;
    backdrive.set(0) ;
    frontLiftRight.set(0) ;
    frontLiftLeft.set(0) ;
    return ;
  }  // can't do anything w/out the gyro from here down.

  if (raiseALL) { 
    doRaiseTogether() ;
  } else if (lowerALL) {
    doLowerTogether() ;
  }

} // END teleoPeriodic

// ***********************************************************************

public void gather_climb_data () {
  // gathering gyro data.
  backLiftAtRaiseLimit = backLiftLimitSwitch.get();
  SmartDashboard.putBoolean("CL: backLiftAtRaiseLimit", backLiftAtRaiseLimit);
  SmartDashboard.putBoolean("CL: climb aborted:", abort_climb);
  float pitch = ahrs.getPitch(); 
  float roll = ahrs.getRoll();  
  if (pitch > FLATpitch) { SmartDashboard.putString("CL pitch:", "tilting LEFT");}
    else {SmartDashboard.putString("CL pitch:", "tilting RIGHT");}
  if (roll > FLATroll) { SmartDashboard.putString("CL roll:", "tipping forward");}
    else { SmartDashboard.putString("CL roll:", "tipping BACK");}
  tiltingLeft = false ;
  if (pitch > FLATpitch) { tiltingLeft = true ;} 
  tiltingRight = false;
  if (pitch < FLATpitch) { tiltingRight = true ;} 
  tippingForward = false ; 
  if (roll > FLATroll  ) { tippingForward = true ;}
  tippingBackward = false ;
  if (roll < FLATroll) { tippingBackward = true ;}
  tiltingDegree = FLATpitch - pitch;
  tippingDegree = FLATroll - roll;
  SmartDashboard.putNumber("CL: tiltingDegree:", tiltingDegree);
  SmartDashboard.putNumber("CL: tippingDegree:", tippingDegree);
  tiltingDegree = Math.abs(tiltingDegree);
  tippingDegree = Math.abs(tippingDegree);

}

// --------------------------------------------------

public void doLowerTogether() {

  L("LOWERING: tilted by " + tiltingDegree + " degrees, evening out left/right...");

    double timeINT = Math.round(m_timer.get()) ;
    double divi = timeINT % 2 ;
    

    if (divi == 0 && tiltingDegree > 3) {
      backlift.set(0);
      even_out_left_right(false); // 'true' means even out by going up first.
      return ;
    } 

    
    L("LOWERING: tipping by " + tippingDegree + " degrees, evening out front/back...");
    if (divi == 0 && tippingDegree > 2) {
      frontLiftLeft.set(0);
      frontLiftRight.set(0);
      if (tippingForward) { L("L: tipping forward") ;
        backlift.set(-.3);
      } else { L("L: tipping back") ;
        backlift.set(.4) ;
      }
      return ;
    }

  // getting this far means, apply power to all motors, have the back raise slightly more than the front.
  backlift.set(-.3) ;
  frontLiftRight.set(-.5) ;
  frontLiftLeft.set(-.5) ;


}
// -------------------------------------------------------------
public void doRaiseTogether() {
  if (backLiftAtRaiseLimit) { // the back is high enough.
    L("back lift has reached limit.");
    backlift.set(0);
    if (tiltingDegree > 2) {
      even_out_left_right(false); // 'true' means even out by going up first.
      return ;
    }  else {
      frontLiftLeft.set(0);
      frontLiftRight.set(0) ;
      fully_raised = true ;
      return ;
    /*  if (tippingForward && tippingDegree > 2) {
        L("back lift at limit, left and right are even, doing tip, current tip at: " + tippingForward);
        frontLiftLeft.set(.5);
        frontLiftRight.set(.5) ;
        return ;
      } else {
        frontLiftLeft.set(0);
        frontLiftRight.set(0) ;
        L("back lift at limit, no longer tipping, finished full raise.");
        fully_raised = true ;
        return ;
      } */
    }
  }

  // not yet raised, so put some power on all 3 motors
  // for some amount of time, then stop and even them out again, then do all three again, etc.

  if (doRaiseTogetherStartTime == 0) {
    doRaiseTogetherStartTime = m_timer.get() ;
    doRaiseTogetherLapTime = doRaiseTogetherStartTime ;
    L("starting doRaiseTogether") ;
  }


    if (tiltingDegree > 5 || tippingDegree > 5) {
      L("aborting!  TILT:" + tiltingDegree + " and TIP:" + tippingDegree );
      SmartDashboard.putString("CLIMBING:", "aborted! falling over.");
      frontLiftLeft.set(0);
      frontLiftRight.set(0);
      backlift.set(0);
      return ;
    }


     // even out every X seconds...
     double LAPTIME = m_timer.get() - doRaiseTogetherLapTime ;
     SmartDashboard.putNumber("MC: Laptime:", LAPTIME);
     L("laptime: " + LAPTIME + " tilt:" + tiltingDegree + " tip:" + tippingDegree);
  if ( LAPTIME > .5) {
    if (! in_evening_phase) { 
      L("entering evening phase at " + m_timer.get() );
      frontLiftLeft.set(0);
      frontLiftRight.set(0) ;
      backlift.set(0);
    }
    in_evening_phase = true ;
    // EVEN OUT
    L("tilted by " + tiltingDegree + " degrees");
    if (tiltingDegree > 3) {
      backlift.set(0);
      even_out_left_right(true); // 'true' means even out by going up first.
      return ;
    } 
    
    L("tipping by " + tippingDegree + " degrees.");
    frontLiftLeft.set(0);
    frontLiftRight.set(0);
    if (tippingDegree > 2) {
      if (tippingForward) { L("tipping forward") ;
        backlift.set(-.5);
      } else { L("tipping back") ;
        backlift.set(1) ;
      }
      L("tip settings: backlift:" + backlift.get() + " frontright:" + frontLiftRight.get() + " frontleft:" + frontLiftLeft.get() );
      return ;
    }
    backlift.set(0);
     // Once we are even, reset LAP time.
     doRaiseTogetherLapTime = m_timer.get() ;
     L("all evened out, continuing up... at  " + doRaiseTogetherLapTime);
     in_evening_phase = false ;
  }

  // getting this far means, apply power to all motors, have the back raise slightly more than the front.
  double M = 1.4 ;
  backlift.set(.38 * M) ;
  frontLiftRight.set(.525 * M) ;
  frontLiftLeft.set(.525 * M) ;

}


// ------------------------------------------------- 


public void even_out_left_right(boolean go_up) {
  // adjust left and right heights until they are mostly even.
  double speed = .5 ;
  if (go_up) {
    if (tiltingLeft) {
      frontLiftRight.set(0);
      frontLiftLeft.set(speed);
    } else if (tiltingRight) {
      frontLiftRight.set(speed);
      frontLiftLeft.set(0);
    }
  } else {  // adjust by going down.
    if (tiltingLeft) {
      frontLiftRight.set(-speed);
      frontLiftLeft.set(0);
  } else if (tiltingRight) {
      frontLiftRight.set(0);
      frontLiftLeft.set(-speed);
    }
  }
  L("in even_out. left motor speed:" + frontLiftLeft.get() + " right motor speed:" + frontLiftRight.get()) ;


}



// -------------------------------------------------
// -------------------------------------------------
public void lower_from_six() {

  // sensor check:
  if (lower_phase == 10 && ! (middleHeight > 2 ) ) {
    SmartDashboard.putString("LOWERING:", "sensor malfunction - just drive off of it.");
    L("middleheight sensor reading bad, aborting lowering. sensor = " + middleHeight) ;
    return ;
  }



  // lower back leg for X seconds.
  if (lower_phase == 10) {
    if (lower_phase_10_timer > 2){
      backlift.set(0) ;
      L("lower phase 10 time elapsed.");
      lower_phase = 20 ;
      return ;
    }
    if (lower_phase_10_timer == 0){
      L("entering lower phase 10."); 
      SmartDashboard.putString("LOWERING:", "phase 10 - lowering wheels...");
      lower_phase_10_start_time = m_timer.get() - .1 ;
    }
    lower_phase_10_timer = m_timer.get() - lower_phase_10_start_time ;
    // lower the back leg 7 inches.
    backlift.set(.5) ;
  }


  // drive backwards at full speed for X seconds.
  if (lower_phase == 20) {
    if (lower_phase_20_timer > 3){
      backdrive.set(0) ;
      L("lower phase 20 time elapsed.");
      lower_phase = 30 ;
      return ;
    }
    if (lower_phase_20_timer == 0){
      L("entering lower phase 20."); 
      SmartDashboard.putString("LOWERING:", "phase 20 - quick backward drive...");
      lower_phase_20_start_time = m_timer.get() - .1 ;
    }
    lower_phase_20_timer = m_timer.get() - lower_phase_20_start_time ;
    // drive backward:
    backdrive.set(-1) ;
  }

  // drive backwards at lower speed until sensor says the legs are clear. 
  if (lower_phase == 30) {
    if (lower_phase_30_timer > 8){
      backdrive.set(0) ;
      L("lower phase 30 aborted, time elapsed, sensor not activated.");
      SmartDashboard.putString("LOWERING:", "ABORTED in phase 30. Not sure what to do. Good luck!");
      return ;
    }
    if (lower_phase_30_timer == 0){
      L("entering lower phase 30.");
      SmartDashboard.putString("LOWERING:", "phase 30 - slow backward drive..."); 
      lower_phase_30_start_time = m_timer.get() - .1 ;
    }
    lower_phase_30_timer = m_timer.get() - lower_phase_30_start_time ;
    if (middleHeight < 2) {
      L("lower phase 30 finished, sensor reads: " + middleHeight) ;
      backdrive.set(0) ;
      lower_phase = 40 ;
      return ;
    }
    // drive backward slowly
    backdrive.set(-.5) ;
  }


    // lower front legs.
    if (lower_phase == 40) {
      if (lower_phase_40_timer > 4){
        frontLiftLeft.set(0);
        frontLiftRight.set(0) ;
        lower_phase = 50 ;
        L("lower phase 40 finished, time elapsed.");
        return ;
      }
      if (lower_phase_40_timer == 0){
        L("entering lower phase 40.");
        SmartDashboard.putString("LOWERING:", "phase 40 - lowering front legs 7 inches..."); 
        lower_phase_40_start_time = m_timer.get() - .1 ;
      }
      lower_phase_40_timer = m_timer.get() - lower_phase_40_start_time ;
      frontLiftLeft.set(.6) ;
      frontLiftRight.set(.6) ;
    }


    // drive backward full speed X seconds.
    if (lower_phase == 50) {
      if (lower_phase_50_timer > 3){
        backdrive.set(0);
        lower_phase = 60 ;
        L("lower phase 50 finished, time elapsed.");
        return ;
      }
      if (lower_phase_50_timer == 0){
        L("entering lower phase 50.");
        SmartDashboard.putString("LOWERING:", "phase 50 - driving away from platform..."); 
        lower_phase_50_start_time = m_timer.get() - .1 ;
      }
      lower_phase_50_timer = m_timer.get() - lower_phase_50_start_time ;
      backdrive.set(-1) ;
    }


    // raise all the legs
    if (lower_phase == 60) {
      if (lower_phase_60_timer > 3){
         backlift.set(0);
         frontLiftLeft.set(0);
         frontLiftRight.set(0);
         L("lower phase 60 finished, time elapsed.");
         SmartDashboard.putString("LOWERING:", "COMPLETE"); 
         return ;
      }
      if (lower_phase_60_timer == 0){
        L("entering lower phase 60.");
        SmartDashboard.putString("LOWERING:", "phase 60 - lowering robot."); 
        lower_phase_60_start_time = m_timer.get() - .1 ;
      }
      lower_phase_60_timer = m_timer.get() - lower_phase_60_start_time ;
      backlift.set(-.35);
      frontLiftLeft.set(-.9);
      frontLiftRight.set(-.9);
    }



}


// -------------------------------------------------
// -------------------------------------------------
public void climb_six() {

  if (abort_climb) { return ;}

  // Phase 10 - raise the whole thing 7 inches.
  if (climb_six_phase  == 10 ) { 
    if (climb_six_phase_10_timer > 4) {
      L("finished climb_six phase 10");
      climb_six_phase = 20 ;
      frontLiftLeft.set(0);
      frontLiftRight.set(0);
      backlift.set(0);
      return ;
    }
    if (climb_six_phase_10_timer == 0){
      L("entering climb_six phase 10."); 
      climb_six_phase_10_start_time = m_timer.get() - .1 ;
      SmartDashboard.putString("CLIMBING:", "phase 10 - raising robot 7 in. ...");
    }
    climb_six_phase_10_timer = m_timer.get() - climb_six_phase_10_start_time;
    doRaiseTogether();
    return;
  }


  // scoot forward until front sensor says we're over the platoform.
  if (climb_six_phase == 20) { 
  /*  double frontHeight = frontHeightSensor.getAverageVoltage();
    if (frontHeight < .25) {
      if (scootForwardclock < 1) {
        L("that was too fast for phase 20, aborting."); // sensor error.
        SmartDashboard.putString("CLIMBING:", "Aborted in phase 20, sensor malfunction. height: " + frontHeight);
        abort_climb = true ;
        backdrive.set(0);
        return ;
      }
      L("finished phase 20.");
      climb_six_phase = 30 ;
      backdrive.set(0);
    } else { */
      if (scootForwardclock > 4) { 
        L("phase 20, scooting, finished.");
        //SmartDashboard.putString("CLIMBING:", "Aborted in phase 20, scoot took too long.");
        backdrive.set(0);
        climb_six_phase = 30 ;
        //abort_climb = true;
        return ;
      }
      if (scootForwardclock == 0) {
        scootStart_time = m_timer.get() - .1 ;
        L("starting phase 20.");
        SmartDashboard.putString("CLIMBING:", "in phase 20, scooting forward.");
      }
       scootForwardclock = m_timer.get() - scootStart_time ;      
       backdrive.set(1) ;
     //} 
  }

    // raising both front legs. 
    if (climb_six_phase == 30) {
      if (raiseLegsClock > 5) {
        climb_six_phase = 40 ;
        frontLiftLeft.set(0);
        frontLiftRight.set(0);
        L("finished phase 30, time elapsed.");
      } else {
        if (raiseLegsClock == 0) {
          raiseLegsStart_time = m_timer.get() - .1 ;
          L("starting phase 30");
          SmartDashboard.putString("CLIMBING:","in phase 30, raising front legs...");
        }
        raiseLegsClock = m_timer.get() - raiseLegsStart_time ;
        frontLiftLeft.set(-.6) ; frontLiftRight.set(-.6) ;
      }
    }
  

    if (climb_six_phase == 40){
      /*double backHeight = backHeightSensor.getAverageVoltage();
      if (backHeight < .2) {
        if (scootForwardclock2 < 1) {
          L("that was too fast for phase 40, aborting."); // sensor error.
          SmartDashboard.putString("CLIMBING:", "Aborted in phase 40, sensor malfunction.  height: " + backHeight);
          abort_climb = true ;
          return ;
        }
        climb_six_phase = 50 ;
        backdrive.set(0);
        m_robotDrive.arcadeDrive(0, 0 );
        L("finished phase 40.");
      } else { */
        if (scootForwardclock2 > 6) { 
          L("phase 40, scooting, finished..");
          //SmartDashboard.putString("CLIMBING:", "Aborted in phase 40, scooting took too long.");
          backdrive.set(0); m_robotDrive.arcadeDrive(0, 0 );
          //abort_climb = true;
          climb_six_phase = 50 ;
          return ;
        }
        if (scootForwardclock2 == 0) {
           scootStart_time2 = m_timer.get() - .1 ;
           L("beginning phase 40.");
           SmartDashboard.putString("CLIMBING:", "in phase 40, driving forward....");
        }
        scootForwardclock2 = m_timer.get() - scootStart_time2 ;      
        backdrive.set(1) ;
        m_robotDrive.arcadeDrive(.4, 0 );
      //} 
    }


  // lift up the back leg just a little.
  if (climb_six_phase == 50) {
    if (raisebackclock >= .7) { 
      climb_six_phase = 60 ;
      backlift.set(0);
      L("finished phase 60, time elapsed.");
      SmartDashboard.putString("CLIMBING:", "COMPLETE") ;
    } else {
      if (raisebackclock == 0) {
         raisebackStart_time = m_timer.get() - .1 ;
         L("entered phase 50.");
         SmartDashboard.putString("CLIMBING:", "in phase 50, raising wheels.");
      }
      raisebackclock = m_timer.get() - raisebackStart_time ;      
      backlift.set(-.9) ;
    } 
  }



}

// -------------------------------------------------
// -------------------------------------------------



  public void climbingprogram() {
   // L("in climbing program...");

    if (climbphase == 0 && distance < 10) {
      L("can't start climbphase, distance is too large, distance = " + distance) ;
      SmartDashboard.putString("CLIMBING:", "can't start, too far from wall, get closer!");
      return ;
    }



    if (climbphase == 0 && fully_raised) { 
      L("CL: finished climbphase 0, going to phase 10.");
      climbphase = 10 ;
      frontLiftRight.set(0);
      frontLiftLeft.set( 0);
      backlift.set(0);
    }

    // Phase 0 - raise the whole thing:
    if (climbphase == 0 ) { 
      if (climbphase0timer > 30) {
        // ABORT!
        L("aborting climbing in phase 0, ran out of time");
        SmartDashboard.putString("CLIMBING:", "aborted in phase 0, didn't get raised in enough time.");
        abort_climb = true ;
        return ;
      }
      if (climbphase0timer == 0){
        L("entering climb phase 0."); 
        SmartDashboard.putString("CLIMBING:", "phase 0. raising...");
        climbphase0Start_time = m_timer.get() - .1 ;
      }
      climbphase0timer = m_timer.get() - climbphase0Start_time ;
      doRaiseTogether();
    }



    // end phase 0
    // start phase 10

    // scoot forward until front sensor says we're over the platoform.
    if (climbphase == 10) { 
      double frontHeight = frontHeightSensor.getAverageVoltage();
      if (frontHeight < .3) {
        if (scootForwardclock < 1) {
          L("that was too fast for phase 10, aborting."); // sensor error.
          SmartDashboard.putString("CLIMBING:", "aborted in phase 10, sensor malfunction.");
          abort_climb = true ;
          return ;
        }
        L("finished phase 10.");
        climbphase = 20 ;
        backdrive.set(0);
      } else {
        if (scootForwardclock > 5) { 
          L("phase 10, scooting, took too long, aborting.");
          SmartDashboard.putString("CLIMBING:", "aborted in phase 10, took too long.");
          backdrive.set(0);
          abort_climb = true;
          return ;
        }
        if (scootForwardclock == 0) {
          scootStart_time = m_timer.get() - .1 ;
          SmartDashboard.putString("CLIMBING:", "phase 10. scooting forward.");
          L("starting phase 10.");
        }

        scootForwardclock = m_timer.get() - scootStart_time ;      
        backdrive.set(1) ;

      } 
    }
    L("first scoot time was: " + scootForwardclock) ;
    

      // end phase 10
      // start phase 20


      // raising both front legs.  full speed, for X seconds.
    if (climbphase == 20) {
      if (raiseLegsClock > 5) {
        climbphase = 25 ;
        frontLiftLeft.set(0);
        frontLiftRight.set(0);
        L("finished phase 20, time elapsed.");
        raiseLegsClock  = 0 ;
        return ;
      } else {
        if (raiseLegsClock == 0) {
          raiseLegsStart_time = m_timer.get() - .1 ;
          L("starting phase 20");
          SmartDashboard.putString("CLIMBING:", "in phase 20, raising front legs quickly...");
        }
        raiseLegsClock = m_timer.get() - raiseLegsStart_time ;
        frontLiftLeft.set(-1) ; frontLiftRight.set(-1) ;
      }
    }

     // end phase 20
     // start phase 25
     if (climbphase == 25) {
      if (raiseLegsClock > 5) {
        climbphase = 30 ;
        frontLiftLeft.set(0);
        frontLiftRight.set(0);
        L("finished phase 25, time elapsed.");
      } else {
        if (raiseLegsClock == 0) {
          raiseLegsStart_time = m_timer.get() - .1 ;
          L("starting phase 25");
          SmartDashboard.putString("CLIMBING:", "in phase 25, raising front legs slowly...");
        }
        raiseLegsClock = m_timer.get() - raiseLegsStart_time ;
        frontLiftLeft.set(-.5) ; frontLiftRight.set(-.5) ;
      }
    }



     // end phase 25
     // start phase 30


      // drive it all forward with other drive wheels.
    if (climbphase == 30){
    double backHeight = backHeightSensor.getAverageVoltage();
    if (backHeight < .3) {
      if (scootForwardclock2 < 1) {
        L("that was too fast for phase 30, aborting."); // sensor error.
        SmartDashboard.putString("CLIMBING:", "phase 30 aborted, sensor malfunction.");
        abort_climb = true ;
        return ;
      }
      climbphase = 40 ;
      backdrive.set(0);
      m_robotDrive.arcadeDrive(0, 0 );
      L("finished phase 30.");
    } else {
      if (scootForwardclock2 > 6) { 
        L("phase 30, scooting, took too long, aborting.");
        SmartDashboard.putString("CLIMBING:", "phase 30 aborted, took too long.");
        backdrive.set(0); m_robotDrive.arcadeDrive(0, 0 );
        abort_climb = true;
        return ;
      }
      if (scootForwardclock2 == 0) {
         scootStart_time2 = m_timer.get() - .1 ;
         L("beginning phase 30.");
         SmartDashboard.putString("CLIMBING:", "in phase 30, driving forward....");
      }
      scootForwardclock2 = m_timer.get() - scootStart_time2 ;      
      backdrive.set(1) ;
      m_robotDrive.arcadeDrive(.4, 0 );
    } 
    L("scoot time 2: " + scootForwardclock2);
   }


    // end phase 30
    // start phase 40

    // lift up the back leg just a little.
    if (climbphase == 40) {
    if (raisebackclock >= 3) { 
      climbphase = 50 ;
      backlift.set(0);
      L("finished phase 40, time elapsed.");
      SmartDashboard.putString("CLIMBING:", "COMPLETE");
    } else {
      if (raisebackclock == 0) {
         raisebackStart_time = m_timer.get() - .1 ;
         L("entered phase 40.");
         SmartDashboard.putString("CLIMBING:", "in phase 40, raising back wheels.");
      }
      raisebackclock = m_timer.get() - raisebackStart_time ;      
      backlift.set(-.3) ;
    } 
    }
    L("raise legs clock: " + raisebackclock);

  } // end climbing program


  // ------------------------------------------------
  public void manual_climbing() {
    SmartDashboard.putString("MC A DOING MANUAL CLIMB", "YES") ;
    SmartDashboard.putBoolean("MC climbmode", climbmode ) ;
    double frontLiftRValue = xbox.getRawAxis(2);
    double frontLiftLValue = xbox.getRawAxis(0);
    // FRONT  RIGHT  LIFT
    if (Math.abs(frontLiftRValue) > .2 ) {
       frontLiftRight.set(frontLiftRValue * .7) ;
    } else {
      frontLiftRight.set(0) ;
    } 
       
    // FRONT LEFT  LIFT
    if (Math.abs(frontLiftLValue) > .2 ) {
       frontLiftLeft.set(frontLiftLValue * .7 ) ;
    } else {
       frontLiftLeft.set(0) ;
    }

    // BACK LIFT, uses button 4 plus the two right front buttons
    boolean RaiseRobotRear = xbox.getRawButton(6);
    boolean LowerRobotRear = xbox.getRawButton(8);
    if (RaiseRobotRear && ! backLiftAtRaiseLimit) {
        backlift.set(.7) ;
    } else if (LowerRobotRear){
        backlift.set(-.7) ;
    } else {
        backlift.set(0) ;
    }

    // DRIVE:
    if (xbox.getRawButton(3)) { 
      backdrive.set(1);
    } else if (xbox.getRawButton(1)){
      backdrive.set(-1);
    } else {
      backdrive.set(0);
    }
    SmartDashboard.putNumber("backdrive value:", backdrive.get());

    
  }


  public void L (String whattolog)  {
    System.out.println("6580log " + whattolog);
    System.out.println(m_timer.get()) ;
  }

  public void do_airCompressor() {
      // AIR compressor  ----------------------------------------------
      SmartDashboard.putBoolean("COMPRESSORSWITCH:",compressorSwitch.get());
      //if (! compressorSwitch.get() ) {
          // turn off compressor during last 30 seconds of a match, regardless of switch.
      if (! compressorSwitch.get() && (m_timer.get() < 120 || m_timer.get() > 160)   && ! climbmode ) {
       // comment the next line with // if you don't want it to run.  remove them to have it run again.
        //compressorRelay.set(Relay.Value.kForward) ;  
      } else {
        compressorRelay.set(Relay.Value.kOff) ;  
        //compressor.set(0);  
      }
  }


  public void navx_check() {
    if ( (m_timer.get() - previous_navx_check) > .3 ) {
      previous_navx_check = m_timer.get() ;
      if (ahrs.getByteCount() > 0 && ahrs.getByteCount() > previous_ahrs_bytecount) {
          if (! navx_online) { L("navx has come online");  }
          navx_online = true ;
          SmartDashboard.putString("CL: NAVX online:","Navx is ONline." ) ;
      } else {
          if (navx_online) { L("navx has gone OFFline"); }
          navx_online = false ;
          SmartDashboard.putString("CL: NAVX online:","Navx is OFFline." ) ;
      }
      previous_ahrs_bytecount = ahrs.getByteCount() ;
    }
  }




  public void do_bucket_encoder() {
    // ENCODER:  how far as the bucket been raised/lowered?   
    // the zero value is taken from wherever it was at the start of teleop.
    double motorDistance = motorEnc.getDistance();
    SmartDashboard.putNumber("Bucket Lowered Distance:", motorDistance);
  }


  public void do_distance_sensors() {
    // DISTANCE SENSOR:  distance from wall on bucket/front side. 
  	distance = myRangeFinder.getAverageVoltage();
		SmartDashboard.putNumber("DISTANCE VOLTAGE: ",distance);
    //distance = distance * 2.4 ;
    if (distance > 1.8) { distance = 5 ;}
    else if (distance > 1.5) { distance = 7 ;}
    else if (distance > .9) { distance = 10 ;}
    else if (distance > .6) { distance = 15 ;}
    else if (distance > .5) { distance = 20 ;}
    else if (distance > .45) { distance = 25 ;}
    else if (distance > .4) { distance = 30 ;}
    else {distance = 100;}
    SmartDashboard.putNumber("DISTANCE: ",distance);

    middleHeight = middleHeightSensor.getAverageVoltage();
    SmartDashboard.putNumber("Middle Height Sensor: ",middleHeight);

    double Urange = frontHeightSensor.getAverageVoltage(); 
    SmartDashboard.putNumber("Front height sensor voltage:", Urange);
    double UrangeB = backHeightSensor.getAverageVoltage(); 
    SmartDashboard.putNumber("BACK height sensor voltage:", UrangeB);





  }






  public void do_reverse_mode () {
    // REVERSE MODE: --------------------------------------------
    // change direction of joystick and which cameras to show.
    boolean button7 = m_stick.getRawButtonPressed(7);
    
    if (reversemode && button7){
      reversemode = false ;
      // change view
      String[] newstream = new String[1] ;
       //      newstream[0] = "mjpg:http://roboRIO-6580-FRC.local:1183/?action=stream" ;
      newstream[0] = "mjpg:http://10.65.80.2:1183/?action=stream" ;
      camLeftStream.setStringArray(newstream);
      newstream[0] = "mjpg:http://10.65.80.2:1184/?action=stream" ;
      camRightStream.setStringArray(newstream);
  
    } else if (! reversemode && button7){
      reversemode = true ;
        // change camera view:
      String[] newstream = new String[1] ;
      newstream[0] = "mjpg:http://10.65.80.2:1181/?action=stream" ;
      camLeftStream.setStringArray(newstream);
      newstream[0] = "mjpg:http://10.65.80.2:1182/?action=stream" ;
      camRightStream.setStringArray(newstream);
    }
    
    if (reversemode) { R = -1 ; } else { R = 1 ; }
      SmartDashboard.putBoolean("REVERSE MODE:", reversemode);
    // ---------------------------------------
  }


  public void get_throttle_value () {
    // ----------------------------------------------------
    // *** GET THROTTLE value:
    throttle = (1 - m_stick.getThrottle()) /  2.0 ;
    turn_throttle = throttle + .2 ;
    SmartDashboard.putNumber("Throttle", throttle);
    if (throttle > .9) { throttle = .95 ;}
    else if (throttle > .8) { throttle = .89 ;}
    else if (throttle > .7) { throttle = .83 ;}
    else if (throttle > .6) { throttle = .77 ;}
    else if (throttle > .5) { throttle = .72 ;}
    else if (throttle > .4) { throttle = .66 ;}
    else if (throttle > .3) { throttle = .6 ;}
    else if (throttle > .2) { throttle = .54 ;}
    else if (throttle > .1) { throttle = .49 ;}
    else if (throttle > 0) { throttle = .43 ;}
    else { throttle= 0; }
    //SmartDashboard.putNumber("Actual tHrottle", throttle);
    //SmartDashboard.putNumber("turn tHrottle", turn_throttle);
  }





  public void SLD() {
    // -----------------------------------------------------------------------------------
    // straight line driving, using THUMB button on joy stick.
    if (! navx_online) { return ;}
    if (m_stick.getRawButton(8)) { return; } // don't overlap with pixy driving.
    SmartDashboard.putBoolean("SLD THUMB", thumb);
    double currentangle = ahrs.getAngle() ; 
    SmartDashboard.putNumber("SLD ANGLE", currentangle);
    SmartDashboard.putNumber("SLD SANGLE", straightangle);
    SmartDashboard.putNumber("SLD STEER", steervalue);
    if (thumb && ! straightlinedrivingmode ) {
        straightangle = currentangle ;
        straightlinedrivingmode = true ;
        steervalue = 0;
    } else if (thumb && straightlinedrivingmode) {
      if (currentangle > straightangle){
        if (turningleft) { steervalue = 0 ;}
        turningright = true ; turningleft = false ;
        steervalue =  steervalue - 0.01 ;
          SmartDashboard.putString("SLD data:"," going right so turning left");
      } else if (currentangle < straightangle ){
        if (turningright) { steervalue = 0 ;}
        turningleft = true ; turningright = false ;
        SmartDashboard.putString("SLD data:"," going left so turning right");
          steervalue = steervalue + 0.01 ;
      } else {
        turningright = false; turningleft = false ;
        steervalue = 0 ;      
      }
      m_robotDrive.arcadeDrive( -throttle * R, steervalue);
  
    } else if (! thumb && straightlinedrivingmode){
        straightlinedrivingmode = false ;
    }
  }
    





  public void normal_driving() {
    //  normal DRIVING *********************************
    //m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
    double joyy = m_stick.getY();
    double joyz = m_stick.getZ();
    if (thumb) { return ;}  // straight line driving.
    if (m_stick.getRawButton(8)) { return ;} // pixy
    m_robotDrive.arcadeDrive(joyy * throttle * R, joyz * turn_throttle );
    
}






public void pixy_driving () {
  // PIXY ------------------------------------------------------
  if (m_stick.getRawButton(8) || xbox.getRawButton(3)) {
    //double mypixyversion = pixy.getVersion();
    pixy.setLamp((byte)1, (byte) 1 );
    //pixy.setServos(5,5);   // what numbers to use here??
    try { pixy.getCCC().getBlocks(false,Pixy2CCC.CCC_SIG1,25) ; } finally {} ;
    ArrayList<Block> blocks = pixy.getCCC().getBlocks() ;
    int blockSignature = 1 ;
    Block largestBlock = null ;
    boolean gotpixyx = false ;
    for (Block block : blocks) {
      SmartDashboard.putNumber("PIXY: blocksig", block.getSignature());
      if (block.getSignature() == blockSignature){
        largestBlock = block ;
        SmartDashboard.putNumber("PIXY: X-VALUe:", largestBlock.getX());
        gotpixyx = true ;
        break;
      }
    }
    // if value of x is smaller than 150, turn left. greater, turn right.
    if ( gotpixyx && largestBlock.getX() < 150 && largestBlock.getX() > 10){
         m_robotDrive.arcadeDrive(0, 1 * turn_throttle );
    } else if (gotpixyx && largestBlock.getX() > 150 && largestBlock.getX() < 400){
         m_robotDrive.arcadeDrive(0, -1 * turn_throttle );
    } 
    
  }
}




public void pov_driving () {
  // *******************************************************
  // use POV to nudge:
  if (m_stick.getRawButtonPressed(5)){
    nudge_value = nudge_value + .03 ;
  } 
  if (m_stick.getRawButtonPressed(3)){
    nudge_value = nudge_value - .03 ;
  }
  SmartDashboard.putNumber("POV nudge value:", nudge_value);
  double pov = m_stick.getPOV() ;
  SmartDashboard.putNumber("POV value:", pov);
  double Xpov = xbox.getPOV() ;
  SmartDashboard.putNumber("POV xbox:", Xpov);
  if (! thumb && ! climbmode) {
  if (m_stick.getButtonCount() > 0 &&  pov > -1 && (pov < 45 || pov >  310) ) { //  FORWARD
    // m_robotDrive.arcadeDrive(-throttle * R , 0);
    m_robotDrive.arcadeDrive(-nudge_value * R , 0);

  } else if (pov > 230) { // LEFT
    //m_robotDrive.arcadeDrive(0, -turn_throttle );
    m_robotDrive.arcadeDrive(0, -nudge_value - .15);
  } else if (pov > 140) { // BACK
    //m_robotDrive.arcadeDrive( throttle * R, 0);
    m_robotDrive.arcadeDrive( nudge_value * R, 0);
  } else if (pov > 50) { // RIGHT
    //m_robotDrive.arcadeDrive(0,turn_throttle  );
    m_robotDrive.arcadeDrive(0,nudge_value + .15 );
  } else { 
    if (xbox.getButtonCount() > 0 &&  Xpov > -1 && (Xpov < 45 || Xpov >  310) ) { //  FORWARD
      m_robotDrive.arcadeDrive(-throttle * R, 0);
    } else if (Xpov > 230) { // LEFT
      m_robotDrive.arcadeDrive(0, -turn_throttle);
    } else if (Xpov > 140) { // BACK
      m_robotDrive.arcadeDrive(throttle * R, 0);
    } else if (Xpov > 50) { // RIGHT
      m_robotDrive.arcadeDrive(0,turn_throttle);
    } 
  } 
  }
}





public void shoot_ball_from_bucket() {
  // --------------------------------------------------------------
    // piston 2 = SHOOT THE BALL from BASKET piston
    //boolean triggerpressed = m_stick.getTrigger();
    //SmartDashboard.putBoolean("TRIGGER", triggerpressed);
    boolean xbutton2 = xbox.getRawButtonPressed(2);
   // if ( triggerpressed || xbutton2){
    // keep it open for at least 1 second.
  if (xbutton2){
    basket_piston_fired_start_time = m_timer.get() ;
    piston2.set(DoubleSolenoid.Value.kReverse);
  } 
  if (basket_piston_fired_start_time > 0) {
    if ( (m_timer.get() - basket_piston_fired_start_time) > .5) {
      piston2.set(DoubleSolenoid.Value.kForward);
      basket_piston_fired_start_time = 0 ;
    }
  } else {  // basket_piston_fired_start_time is at zero:
    piston2.set(DoubleSolenoid.Value.kForward);
  }
}






public void do_hatch_piston () {
  // ---------------------------------------------------------
  //  HATCH PISTON
  boolean Xtopleft = xbox.getRawButton(5);  // extend button
  boolean Xbottomleft = xbox.getRawButton(7); // retract button
  if (Xtopleft){
    piston.set(DoubleSolenoid.Value.kForward);
  } else if (Xbottomleft) {
    piston.set(DoubleSolenoid.Value.kReverse);
  }
}





public void do_hatch_motor() {
  /// HATCH MOTOR  ----------------------------------------
  // boolean button6 = m_stick.getRawButton(6); // UP
  // boolean button4 = m_stick.getRawButton(4);  // DOWN
  double Xyaxis = xbox.getRawAxis(1);
  if (Xyaxis > .8) { // DOWN
    hatch_up = false ;
    hatchmotor.set(1);
  } else if (Xyaxis < -.8){ // UP
    hatch_up = true ;
    hatchmotor.set(-1);
  } else {
    // keep a little upward force on it to keep it from falling.
    if ( hatch_up) {
      double idlevalue = -.4 ;
      hatchmotor.set(idlevalue) ;
    } else {
      hatchmotor.set(0);
    }
  } 
  SmartDashboard.putNumber("Hatch motor:",hatchmotor.get() );
  SmartDashboard.putBoolean("Hatch Hook is UP:",hatch_up );
}






public void do_bucket_elevator() {
  // BUCKET ELEVATOR  -------------------------------------
    double Xzrotate = xbox.getRawAxis(3);
    SmartDashboard.putNumber("ZRaxis:",Xzrotate);
    if (Math.abs(Xzrotate) > .2) {
      bucketelevator.set(-Xzrotate * .2 );
    } else {
      bucketelevator.set(0);
    }
}





  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

/*

notes:


    // log something.
    System.out.println("something here");



*/


    
// NAV X  experiments here:
//float Xdisp = ahrs.getDisplacementX() ;
//SmartDashboard.putString("XDISP:",Float.toString(Xdisp));
//double Ydisp = ahrs.getDisplacementY() ;
//SmartDashboard.putNumber("YDISP:",Ydisp);
//double Zdisp = ahrs.getDisplacementZ() ;
//SmartDashboard.putNumber("ZDISP:",Zdisp);
//boolean motionDetected = ahrs.isMoving();
//SmartDashboard.putBoolean("MotionDetected", motionDetected);



  // network table stuff...
  // set a value in the network table.
  // sometablevalue.setNumber(3);