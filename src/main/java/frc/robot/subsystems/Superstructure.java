package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends SubsystemBase {

  private IntakeSubsystem mIntake = new IntakeSubsystem();
  private ShooterSubsystem mShooter = new ShooterSubsystem();
  private Climb mClimb = new Climb();
  //private static boolean goingToNextBar = false;
  private final PigeonIMU m_pig = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);
  
  private int climbState = 0;
  private double gyroPitch;
  private double angleOffset;
  private double tele1Offset;
  private double tele2Offset;
  private boolean autoAdvanceEnabled = true; //true;
  
  public void startClimb(){ //climb start position - doesnt get used 10-26-22
    mShooter.stopShooter();
    mIntake.retractIntake();

    //mArm.setArm(10);
    
    if(climbState < 1){
      climbState = 1;
      mClimb.setTele1(0); // climber 1 
      mClimb.setTele2(0); // climber 2  
      angleOffset = m_pig.getPitch();
      tele1Offset = mClimb.getTele1Pos();
      tele2Offset = mClimb.getTele2Pos();
    }
  }


  public void climbControl(int state){
    climbState += state;
    gyroPitch = m_pig.getPitch();
    
    if(climbState == 1){   
        mClimb.setTele1(0);
        mClimb.setTele2(0);
        angleOffset = m_pig.getPitch();}

    else if (climbState == 2){
      mClimb.setTele1(-200000); //full extension to grab mid rung
      mClimb.setTele2(30000);} //half extension to prepare for handoff
      //advance manually to start climb
      
    else if (climbState == 3){
      mClimb.setTele1(-000);
      mClimb.setTele2(240000); //full extension to grab high rung
    // advance climb for angle dependent grab
    //  if(gyroPitch - angleOffset < -28 && autoAdvanceEnabled && mClimb.getTele2Pos() - tele2Offset > 230000)climbState++;
      if(gyroPitch - angleOffset < -35 && autoAdvanceEnabled) climbState++;
    } 
/* else if (climbState == 4){ //
      mClimb.setTele1(-000);// lift off of high run to maintain momentum 10-26-22
      mClimb.setTele2(0000); }*/

    else if (climbState == 4){
      if(gyroPitch - angleOffset > -28 && autoAdvanceEnabled && mClimb.getTele2Pos() - tele2Offset > 230000){
      mClimb.setTele1(-000);// lift off of high run to maintain momentum 10-26-22
      mClimb.setTele2(0000); }
      else if (autoAdvanceEnabled && mClimb.getTele2Pos() - tele2Offset < 220000) climbState++;

      if(!autoAdvanceEnabled){
        mClimb.setTele1(-000);// lift off of high run to maintain momentum 10-26-22
        mClimb.setTele2(0000);
      }
      //INTENTIONALLY EMPTY CLIMB STATE
      //Swing through high bar grab location
      // advance climb for angle dependent grab
      /*if(gyroPitch - angleOffset < -35 && autoAdvanceEnabled){ //if angle is within deadband
        if(mClimb.getTele2Pos() > 230000)//if telescope is mast the min grab point
            {climbState++;}}    //if telescope position is within deadband
        
        //if(mClimb.getTele2Pos() - tele2Offset < 200000 && autoAdvanceEnabled){ //if angle is within deadband -- INTENTIONALLY DOESNT INCLUDE autoAdvanceEnabled
        //  climbState++;}    //if telescope position is within deadband
        */
       } //if angle is within deadband
      
    else if (climbState == 5){ //FOR BEHIND TRAV BAR GRAB (4-6-22 CLIMBER HOOK CONFIG)
      //mClimb.setTele1(-150000); //changed for manual mode
      if(autoAdvanceEnabled) mClimb.setTele1(-170000); //10-26-22 -> Owen did a comment! Good job Owen. Changed from -180000
      else  mClimb.setTele1(-240000);
      mClimb.setTele2(0000);
      // advance climb for angle dependent grab
      if(gyroPitch - angleOffset < -35 && mClimb.getTele1Pos() - tele1Offset < -160000 && mClimb.getTele2Pos() - tele2Offset < 4000 && autoAdvanceEnabled){ //if angle is within deadband     
        mClimb.setTele1(-240000); //10-26-22 -> Owen did a comment! Good job Owen. Changed from -180000
          
        climbState++;
      }
      
    } 
    else if (climbState == 6){//FOR BEHIND TRAV BAR GRAB (4-6-22 CLIMBER HOOK CONFIG)
    //  mClimb.setTele1(-240000); //full extension for trav 
    if(gyroPitch - angleOffset > -29.5 && autoAdvanceEnabled && mClimb.getTele2Pos() - tele2Offset < 1500 &&  mClimb.getTele1Pos() - tele1Offset < -235000 ){
      mClimb.setTele1(-180000);
      mClimb.setTele2(0000); }

      if(!autoAdvanceEnabled){
      mClimb.setTele1(-180000); //changed for manual mode
      mClimb.setTele2(0000);}
        // advance climb for angle dependent grab
       /* if( gyroPitch - angleOffset > -27 && autoAdvanceEnabled){ //if angle is within deadband
          if(mClimb.getTele2Pos() - tele2Offset < 10000){//if carriage position is within deadband
              //climbState++;
          }    
        } */
      //if telescope position is within deadband
        }  //if angle is within deadband
      
    else if (climbState == 7){
      mClimb.setTele1(-100000); //changed for manual mode
//      mClimb.setTele1(-180000);
      mClimb.setTele2(00000);
    } 
    else if (climbState == 8){ //end of climb
      mClimb.setTele1(-000);
      mClimb.setTele2(0);
  /*
    else if (climbState == 5){ //FOR AHEAD OF TRAV BAR GRAB (4-6-22 Not yet setup)
      mClimb.setTele1(140000); //Full extension to grab trav
      mClimb.setTele2(1000);
      // advance at top of swing
      if(gyroPitch - angleOffset < 25){      
              climbState++;}} 

    else if (climbState == 6){ //FOR AHEAD OF TRAV BAR GRAB (4-6-22 Not yet setup)
      // advance for final handoff
      if(gyroPitch - angleOffset > 267{      
              climbState++;}}    

    else if (climbState == 5){ //FOR AHEAD OF TRAV BAR GRAB (4-6-22 Not yet setup)
      mClimb.setTele1(100000); //Grab Trav
      mClimb.setTele2(30000);} //let go of high                     
  */     
    }  
    SmartDashboard.putNumber("ClimbState", climbState);
    SmartDashboard.putNumber("tele1", mClimb.getTele1Pos() - tele1Offset);
    SmartDashboard.putNumber("tele2", mClimb.getTele2Pos() - tele2Offset);
    SmartDashboard.putNumber("GyroPitch - Offset", gyroPitch - angleOffset);
    }
  }

/*OLD PROTO CLIMB CODE
  public void climb() {
    if (!mClimb.tele1Extended()) {
      mClimb.setTele1(Constants.EXTENDED_CLIMB_POSITION);
    } else {
      mClimb.setTele1(0);

      while(!goingToNextBar) {
        if(m_pig.getRoll() == Constants.CLIMB_ROLL_POS_1) {
          goingToNextBar = true;
          mClimb.setTele2(Constants.EXTENDED_CLIMB_POSITION);
          Timer.delay(5.0);
          mClimb.setTele2(Constants.EXTENDED_CLIMB_POSITION/2);
          Timer.delay(0.75);
          mClimb.setTele2(0);
        }
      }

      Timer.delay(2.0);

      while(!goingToNextBar) {
        if(m_pig.getRoll() == Constants.CLIMB_ROLL_POS_2) {
          goingToNextBar = true;
          mClimb.setTele2(Constants.EXTENDED_CLIMB_POSITION);
          Timer.delay(5.0);
          mClimb.setTele2(Constants.EXTENDED_CLIMB_POSITION/2);
          Timer.delay(0.75);
          mClimb.setTele2(0);
        }
      }
    }
  }
*/
