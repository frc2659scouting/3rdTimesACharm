package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {

    enum AutonMode {
        JUST_SHOOT, DO_NOTHING, JUST_DRIVE_BACK, SHOOT_DRIVE_BACK, RIGHT_SIDE_3_BALL, RIGHT_SIDE_MULTI, RIGHT_SIDE_4_BALL, DISCO, LEFT_SIDE_3_BALL
    }

    private AutonMode mCachedAutoMode = null;

 //    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<AutonMode> mAutoMode;
 //   private AutonMode autoModeReturn = null;
    private String autoChoiceReturn;

    public AutoModeSelector() {
        mAutoMode = new SendableChooser<>();
        mAutoMode.setDefaultOption("JUST SHOOT", AutonMode.JUST_SHOOT);
        mAutoMode.addOption("DO NOTHING", AutonMode.DO_NOTHING);
        mAutoMode.addOption("JUST DRIVE BACK", AutonMode.JUST_DRIVE_BACK);
        mAutoMode.addOption("SHOOT AND DRIVE BACK", AutonMode.SHOOT_DRIVE_BACK);
        mAutoMode.addOption("RIGHT SIDE 3 BALL", AutonMode.RIGHT_SIDE_3_BALL);
        mAutoMode.addOption("RIGHT SIDE 6 BALL", AutonMode.RIGHT_SIDE_MULTI);
        mAutoMode.addOption("RIGHT SIDE 4 BALL", AutonMode.RIGHT_SIDE_4_BALL);
        mAutoMode.addOption("LEFT SIDE 3 BALL", AutonMode.LEFT_SIDE_3_BALL);
        mAutoMode.addOption("DISCO", AutonMode.DISCO);
        SmartDashboard.putData("Auto Mode", mAutoMode);
    }

    public void updateModeCreator() {
        AutonMode desiredMode = mAutoMode.getSelected();
         if (mCachedAutoMode != desiredMode ) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
          //  autoModeReturn = desiredMode;
        }
        mCachedAutoMode = desiredMode;
    }

    public String returnAutoMode(){
       autoChoiceReturn = mAutoMode.getSelected().toString();
       return(autoChoiceReturn);
    }

    public void outputToSmartDashboard() {
       // SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("Auto Mode Selected", mCachedAutoMode.name());
    }

    public void reset() {
        mCachedAutoMode = null;
    }
}