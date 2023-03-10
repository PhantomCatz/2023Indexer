package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.DataLogger.DataCollection;

import com.revrobotics.ColorSensorV3;


public class CatzIndexer{
    //          SPARKMAX DEFS
    public CANSparkMax indexerMtrCtrlFrnt;

    private final int INDEXER_FRNT_MC_CAN_ID        = 40;//1;
    private final int INDEXER_FRNT_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FRNT_SPEED = 0.4;
    public final double INDEXER_FRNT_SPEED_FAST = 1;

    //
    public final double INDEXER_SPEED_OFF = 0.0;
    
    //          SECOND MOTOR DEFS
    public CANSparkMax indexerMtrCtrlBack;

    private final int INDEXER_BACK_MC_CAN_ID        = 56;//2;
    private final int INDEXER_BACK_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_BACK_SPEED = 0.2;


//              RIGHT WHEELS
    public CANSparkMax indexerMtrCtrlRGT_WHEELS;

    private final int INDEXER_MC_CAN_ID_RGT       = 37;//2;
    private final int INDEXER_MC_CURRENT_LIMIT_RGT = 60;

    public final double INDEXER_SIDE_WHEEL_SPEED_INJEST = 1.0;

    public CANSparkMax indexerMtrCtrlLFT_WHEELS;

    private final int INDEXER_MC_CAN_ID_LFT       = 38;//2;
    private final int INDEXER_MC_CURRENT_LIMIT_LFT = 60;

    public final double INDEXER_SIDE_WHEEL_SPEED_EJECT = 0.5;


    
    //          Flipper Defs
    public CANSparkMax indexerMtrCtrlFlipper;

    private final int INDEXER_FLIPPER_MC_CAN_ID        = 3;
    private final int INDEXER_FLIPPER_MC_CURRENT_LIMIT = 60;

    private final double FLIPPER_SPEED_ACTIVE = -0.1;
    private final double FLIPPER_SPEED_DEACTIVE = 0.4;
    private final double FLIPPER_SPEED_STOP = 0.0;

    //          BEAMBREAK DEFS
  
    public DigitalInput beamBreak;
  
    private final int BEAM_BREAK_DIO_PORT_LFT = 4;

    private boolean beamBreakContinuity = false;


    
    //            COLOR SENSOR DEFS
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private final int PROXIMITY_DISTANCE_THRESHOLD = 85;

    //          COLOR SENSOR COLOR CONSTANTS
    private final double CUBE_RED_THRESHOLD     = 0.25;
    private final double CUBE_GREEN_THRESHOLD   = 0.45;
    private final double CUBE_BLUE_THRESHOLD    = 0.28;

    private final double CONE_BLUE_THRESHOLD    = 0.2;

    //          SENSOR VARIABLES
    private double RedValue     = -999.0;
    private double GreenValue = -999.0;
    private double BlueValue = -999.0;

    Color detectedColor;
    double IR;
    int proximity;

    public DigitalInput LimitSwitchRGT;
    public DigitalInput LimitSwitchLFT;

//    private final int LIMIT_SWITCH_DIO_PORT_RGT = 5; 
//    private final int LIMIT_SWITCH_DIO_PORT_LFT = 6;

    boolean limitSwitchPressedLFT;
    boolean limitSwitchPressedRGT;

    
    

    //          objectdetection and flags
    public boolean objectReady            = false; // needs to be set back to false in elevator class
    
    private boolean objectDetected; 

    public static boolean objectCurrentlyIntaking;

    private boolean clawActive; // needs to be set off in elevator class

    public boolean flipperActive;

    private boolean flipperDeactivating;

    public static boolean isConeDetected = false;

    public static boolean isCubeDetected = false;
    

    //
    private Thread Indexer;

    private Timer indexerTimer;
    private double indexerTime;

    private Timer flipperTimer;
    private double flipperTime;

    int finalStateINT;

    private String colorSelected = "None";


    //          CONSTANTS

    private double INDEXER_MAX_BELT_TIME                = 7.0;
    private double FLIPPER_ENCODER_MAX_VALUE_ACTIVE     = -100;
    private double FLIPPER_ENCODER_MAX_VALUE_DEACTIVE   = -20;
   

    public CatzLog data;

    private RelativeEncoder m_encoder;




    //                      End of definitions

    public CatzIndexer() 
    {
        indexerMtrCtrlFrnt = new CANSparkMax(INDEXER_FRNT_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlFrnt.restoreFactoryDefaults();
        indexerMtrCtrlFrnt.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlFrnt.setSmartCurrentLimit(INDEXER_FRNT_MC_CURRENT_LIMIT);

      
        indexerMtrCtrlBack = new CANSparkMax(INDEXER_BACK_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlBack.restoreFactoryDefaults();
        indexerMtrCtrlBack.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlBack.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);


        indexerMtrCtrlFlipper = new CANSparkMax(INDEXER_FLIPPER_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlFlipper.restoreFactoryDefaults();
        indexerMtrCtrlFlipper.setIdleMode(IdleMode.kCoast);
        indexerMtrCtrlFlipper.setSmartCurrentLimit(INDEXER_FLIPPER_MC_CURRENT_LIMIT);

    
        indexerMtrCtrlRGT_WHEELS = new CANSparkMax(INDEXER_MC_CAN_ID_RGT, MotorType.kBrushless); 

        indexerMtrCtrlRGT_WHEELS.restoreFactoryDefaults();
        indexerMtrCtrlRGT_WHEELS.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlRGT_WHEELS.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);
 

        indexerMtrCtrlLFT_WHEELS = new CANSparkMax(INDEXER_MC_CAN_ID_LFT, MotorType.kBrushless); 

        indexerMtrCtrlLFT_WHEELS.restoreFactoryDefaults();
        indexerMtrCtrlLFT_WHEELS.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlLFT_WHEELS.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);
        indexerMtrCtrlLFT_WHEELS.follow(indexerMtrCtrlRGT_WHEELS, true);
    
        indexerTimer = new Timer();

        beamBreak = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);

     //   LimitSwitchLFT = new DigitalInput(LIMIT_SWITCH_DIO_PORT_LFT);
     //   LimitSwitchRGT = new DigitalInput(LIMIT_SWITCH_DIO_PORT_RGT);

        
        m_encoder = indexerMtrCtrlFlipper.getEncoder();
        m_encoder.setPositionConversionFactor(60);  //needs to be in correct position when contructor is called.
        m_encoder.setPosition(0);
        

        
        startIndexerThread();

      
    }

    public void startIndexerThread() 
    {
        Indexer = new Thread(() ->
        {
            while(true)
            {   
                 
                if(objectReady = false) //needs to be set to be set back to false in the elevator class when object is on the elevator
                {
                collectColorValues();
                collectBeamBreakValues();
                collectColorSensorDistanceValues();
                collectLimitSwitchValues();
                }
                 
                if(Robot.intake.intakeActive == true) //needs intakeActive from intake class set on when the rollers are active
                {
                    runIndexerBelt();
                    runSideWheelsFwd();
                    indexerTime = indexerTimer.get();
                    indexerTimer.reset();
                    indexerTimer.start();
                }
                else
                {
                
                }
                if(indexerTime > INDEXER_MAX_BELT_TIME)
                {
                    stopbelt();
                    stopSideWheels();
                }
                
                if(limitSwitchPressedLFT && limitSwitchPressedRGT)
                {
                    stopbelt();
                    stopSideWheels();
                    
                    objectReady = true; 
                }
                if(CubeDetectedMethod())
                {
                    colorSelected = "cube";
                    objectReady = true;
                }
                if(ConeDetectedMethod())
                {
                    colorSelected = "cone";
                }
                if(beamBreakContinuity == false)
                {
                    flipperActive = true;
                }
                if(flipperActive == true)
                {
                    if(m_encoder.getPosition() > FLIPPER_ENCODER_MAX_VALUE_ACTIVE)
                    {
                        indexerMtrCtrlFlipper.set(FLIPPER_SPEED_ACTIVE);
                    }
                    else
                    {
                        indexerMtrCtrlFlipper.set(FLIPPER_SPEED_STOP);
                        flipperActive = false;
                        flipperDeactivating = true;
                    }
                }
                if(flipperDeactivating == true)
                {
                    if(m_encoder.getPosition() < FLIPPER_ENCODER_MAX_VALUE_DEACTIVE)
                    {
                        indexerMtrCtrlFlipper.set(FLIPPER_SPEED_DEACTIVE);
                    }
                    else
                    {
                        indexerMtrCtrlFlipper.set(FLIPPER_SPEED_STOP);
                        flipperDeactivating = false;
                        objectReady = true;
                        stopbelt();
                        stopSideWheels();
                    }
                }

                DataCollectionIndexer();
                
            Timer.delay(0.02);
            }
            
         });
        Indexer.start();
    }
    
    /*-----------------------------------------------------------------------------------------
    *  
    *  Sensor Data Collection
    *
    *----------------------------------------------------------------------------------------*/


   public void collectColorValues() 
   {

        RedValue    = detectedColor.red;
        GreenValue  = detectedColor.green;
        BlueValue   = detectedColor.blue;

        detectedColor    = m_colorSensor.getColor();
    }

    public void collectColorSensorDistanceValues()
    {
        IR               = m_colorSensor.getIR();
        proximity        = m_colorSensor.getProximity();
    }

   public void collectBeamBreakValues() 
   {
        beamBreakContinuity = beamBreak.get();

    }
    public void collectLimitSwitchValues() 
    {

        limitSwitchPressedRGT = LimitSwitchRGT.get();
        limitSwitchPressedLFT = LimitSwitchLFT.get();
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Indexer Belt Controls
    *
    *----------------------------------------------------------------------------------------*/

    public void runIndexerBeltRev() 
    {
        indexerMtrCtrlFrnt.set(INDEXER_FRNT_SPEED);
        indexerMtrCtrlBack.set(INDEXER_BACK_SPEED);
    }

    public void runIndexerBelt() 
    {
        indexerMtrCtrlFrnt.set(-INDEXER_FRNT_SPEED);
        indexerMtrCtrlBack.set(-INDEXER_BACK_SPEED);
    }

    public void runIndexerBeltFast()
    {
        indexerMtrCtrlFrnt.set(INDEXER_FRNT_SPEED_FAST);
        indexerMtrCtrlBack.set(INDEXER_BACK_SPEED);
    }

    public void stopbelt() 
    {
        indexerMtrCtrlFrnt.stopMotor();
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Side Wheel Controls
    *
    *----------------------------------------------------------------------------------------*/

    public void runSideWheelsFwd()
    {
        indexerMtrCtrlRGT_WHEELS.set( INDEXER_SIDE_WHEEL_SPEED_EJECT);

    } 
    
    public void runSideWheelsFev()
    {
        indexerMtrCtrlRGT_WHEELS.set(-INDEXER_SIDE_WHEEL_SPEED_EJECT);

    }

    public void stopSideWheels()
    {
    
        indexerMtrCtrlRGT_WHEELS.set(FLIPPER_SPEED_STOP);

    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Front Indexer Controls
    *
    *----------------------------------------------------------------------------------------*/


    public void indexerFrntIngest() 
    {
        indexerMtrCtrlRGT_WHEELS.set( INDEXER_SIDE_WHEEL_SPEED_EJECT);
        indexerMtrCtrlFrnt.set(INDEXER_FRNT_SPEED);
    }

    public void indexerFrntEject() 
    {
        indexerMtrCtrlRGT_WHEELS.set(-INDEXER_SIDE_WHEEL_SPEED_EJECT);
        indexerMtrCtrlFrnt.set(INDEXER_BACK_SPEED);
    }

    public void indexerFrntStop() 
    {
        indexerMtrCtrlRGT_WHEELS.set(FLIPPER_SPEED_STOP);
        indexerMtrCtrlFrnt.set(FLIPPER_SPEED_STOP);

    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Back Indexer Controls
    *
    *----------------------------------------------------------------------------------------*/
    public void indexerBackIngest() 
    {
        indexerMtrCtrlBack.set(-1.0);
    }

    public void indexerBackEject() 
    {
        indexerMtrCtrlBack.set(0.5);
    }

    public void indexerBackStop() 
    {
        indexerMtrCtrlBack.set(FLIPPER_SPEED_STOP);
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  flipper control
    *
    *----------------------------------------------------------------------------------------*/
    public void flipperActiveMethod() 
    {
        if(m_encoder.getPosition() > FLIPPER_ENCODER_MAX_VALUE_ACTIVE)
        {
            indexerMtrCtrlFlipper.set(FLIPPER_SPEED_ACTIVE);
        }
        else
        {
            indexerMtrCtrlFlipper.set(FLIPPER_SPEED_STOP);
            flipperActive = false;
            flipperDeactivating = true;
        }
    }

    public void flipperDeactivatingMethod()
    {
        if(m_encoder.getPosition() < FLIPPER_ENCODER_MAX_VALUE_DEACTIVE)
        {
            indexerMtrCtrlFlipper.set(FLIPPER_SPEED_DEACTIVE);
        }
        else
        {
            indexerMtrCtrlFlipper.set(FLIPPER_SPEED_STOP);
            flipperDeactivating = false;
            objectReady = true;
            stopbelt();
            stopSideWheels();
        }
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Xbox control
    *
    *----------------------------------------------------------------------------------------*/

    public void procCmdIndexerFrnt(boolean xboxValueIn, boolean xboxValueOut)
    {
      if (xboxValueIn == true)
      {
        indexerFrntIngest();
      }
      else if (xboxValueOut == true)
      { 
        indexerFrntEject();
      }
      else
      {
        indexerFrntStop();
      }
    }

    public void DataCollectionIndexer()
    {
        if (DataCollection.LOG_ID_INDEXER == DataCollection.chosenDataID.getSelected())
        {
            data = new CatzLog(Robot.currentTime.get(),detectedColor.red,detectedColor.blue,detectedColor.green,proximity,finalStateINT,0,0,0,0,0,0,0,0,0,1);  
            Robot.dataCollection.logData.add(data);
        }
    }


    /*-----------------------------------------------------------------------------------------
    *  
    *  Sensor Methods
    *
    *----------------------------------------------------------------------------------------*/
    public boolean objectDetectedMethod()
    {

        if (proximity >= PROXIMITY_DISTANCE_THRESHOLD){
            objectDetected = true;
        }
        else
        {
            objectDetected = false;
        }
        return objectDetected;
    }
    public boolean ConeDetectedMethod()
    {
        if (RedValue <= CUBE_RED_THRESHOLD && GreenValue <= CUBE_GREEN_THRESHOLD && BlueValue >= CUBE_BLUE_THRESHOLD)
        {
            isConeDetected = true;
        }
        else
        {
            isConeDetected = false;
        }
        return isConeDetected;
    }

    public boolean CubeDetectedMethod()
    {
        if (BlueValue <= CONE_BLUE_THRESHOLD)
        {
            isCubeDetected = true;
        }
        else
        {
            isCubeDetected = false;
        }
        return isCubeDetected;
    }
    
        
    public Color getColor() {
        double r = (double)getColor().red;
        double g = (double)getColor().green;
        double b = (double)getColor().blue;
        double mag = r + g + b;
        return new Color(r / mag, g / mag, b/ mag);
    }
    /*-----------------------------------------------------------------------------------------
    *  
    *  Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void SmartDashboardIndexer()
    {
        SmartDashboard.putString("object detected", colorSelected);


    }
    public void SmartDashboardIndexer_Debug()
    {
        SmartDashboard.putNumber("TraceID", finalStateINT);
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Encoder getpositionfactor", m_encoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putBoolean("BeamBreak", beamBreakContinuity);
    }

}