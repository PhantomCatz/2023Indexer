package frc.Mechanisms;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.DataLogger.DataCollection;

import com.revrobotics.ColorSensorV3;


public class CatzIndexer {
    //          SPARKMAX DEFS
    public CANSparkMax inDexerMtrCtrlFRNT;

    private final int INDEXER_FRNT_MC_CAN_ID        = 30;
    private final int INDEXER_FRNT_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FRNT_SPEED = 0.4;


    //
    public final double INDEXER_SPEED_OFF = 0.0;
    
    //          SECOND MOTOR DEFS
    public CANSparkMax inDexerMtrCtrlBACK;

    private final int INDEXER_BACK_MC_CAN_ID        = 31;
    private final int INDEXER_BACK_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_BACK_SPEED = 0.2;
    
    //          Flipper Defs
    public CANSparkMax inDexerMtrCtrlFLIPPER;

    private final int INDEXER_FLIPPER_MC_CAN_ID        = 32;
    private final int INDEXER_FLIPPER_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FLIPPER_SPEED = 0.2;

    //          BEAMBREAK DEFS
    public DigitalInput beamBreakRGT;
    public DigitalInput beamBreakLFT;
    private final int BEAM_BREAK_DIO_PORT_RGT = 3;
    private final int BEAM_BREAK_DIO_PORT_LFT = 4;//up and down?
    
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
    private double RedValue = -999.0;
    private double GreenValue = -999.0;
    private double BlueValue = -999.0;

    private boolean beamBreakLFTContinuity = false;
    private boolean beamBreakRGTContinuity = false;

    Color detectedColor;
    double IR;
    int proximity;

    //          objectdetection and flags
    public boolean objectReady = false;
    
    private boolean objectDetected; 

    private boolean objectNotInPosition;

    private boolean coneTestActive;

    private boolean flipperActive;

    private boolean colorTestActive;

    private boolean isConeDetected = false;

    private boolean isCubeDetected = false;
    

    //
    private Thread Indexer;
    private Thread Shuffle;

    private Timer indexerTimer;
    private double indexerTime;

    private Timer flipperTimer;
    private double flipperTime;

    int finalStateINT;

    private String colorSelected = "None";


    //          CONSTANTS
    private double MAX_BELT_TIME = 4;
    private double CONE_TEST_DELAY_THRESHOLD = 4.8;
    private double FLIPPER_DELAY_THRESHOLD = 7;

    public CatzLog data;
    //                      End of definitions

    public CatzIndexer() 
    {
        inDexerMtrCtrlFRNT = new CANSparkMax(INDEXER_FRNT_MC_CAN_ID, MotorType.kBrushless); 

        inDexerMtrCtrlFRNT.restoreFactoryDefaults();
        inDexerMtrCtrlFRNT.setIdleMode(IdleMode.kBrake);
        inDexerMtrCtrlFRNT.setSmartCurrentLimit(INDEXER_FRNT_MC_CURRENT_LIMIT);

        
        inDexerMtrCtrlBACK = new CANSparkMax(INDEXER_BACK_MC_CAN_ID, MotorType.kBrushless); 

        inDexerMtrCtrlBACK.restoreFactoryDefaults();
        inDexerMtrCtrlBACK.setIdleMode(IdleMode.kBrake);
        inDexerMtrCtrlBACK.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);


        inDexerMtrCtrlFLIPPER = new CANSparkMax(INDEXER_FLIPPER_MC_CAN_ID, MotorType.kBrushless); 

        inDexerMtrCtrlFLIPPER.restoreFactoryDefaults();
        inDexerMtrCtrlFLIPPER.setIdleMode(IdleMode.kBrake);
        inDexerMtrCtrlFLIPPER.setSmartCurrentLimit(INDEXER_FLIPPER_MC_CURRENT_LIMIT);



        indexerTimer = new Timer();

        flipperTimer = new Timer();


        beamBreakLFT = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);
        beamBreakRGT = new DigitalInput(BEAM_BREAK_DIO_PORT_RGT);

        startIndexerThread();

      
    }

    void startIndexerThread() 
    {
        Indexer = new Thread(() ->
        {
            while(true)
            {
                collectColorSensorDistanceValues();


                if (objectDetectedMethod())//if ir detects object
                {
                    resetFlags();
                    while(colorTestActive = true)
                    {
                        collectColorValues(); 
                        
                        if(CubeDetectedMethod())//test color values for cube
                        {
                            colorSelected = "cube";
                            indexerTimer.reset();
                            indexerTimer.start();
                            indexerTime = indexerTimer.get();//reset timer

                            runIndexerBelt();
                            while(objectNotInPosition == true)
                            {
                                collectBeamBreakValues();

                                if(((beamBreakLFTContinuity == false) || (beamBreakRGTContinuity == false)) || indexerTime > MAX_BELT_TIME)//if a beambreak is broken or if the time limit is reached
                                {
                                    stopbelt();
                                    objectReady = true;
                                    finalStateINT = 3;
                                    exitFlags(); //turns the while statments to false to breakout of the while loops
                                }
                            }
                        }
                    
                        if(ConeDetectedMethod())//test color values for cone
                        {
                            colorSelected = "CONE";
                            indexerTimer.reset();
                            indexerTimer.start();
                            indexerTime = indexerTimer.get();//reset timer
                    
                            runIndexerBelt();
                            while(objectNotInPosition == true)
                            {
                                if(indexerTime >= MAX_BELT_TIME) //once the cone reaches the end, stop belt
                                {
                                    stopbelt();
                                   
                                    while (coneTestActive = true)
                                    {
                                        if(indexerTime >= CONE_TEST_DELAY_THRESHOLD)//once the delay is over, start testing for orientation
                                        {
                                            
                                            collectBeamBreakValues();

                                            if((beamBreakLFTContinuity == true) || (beamBreakRGTContinuity == true))//if not in correct position
                                            {
                                                runFlipper();
                                                
                                                resetFlipper();

                                                collectBeamBreakValues();
                                                if ((beamBreakRGTContinuity == false) && (beamBreakLFTContinuity == false))//if the cone is in the correct position
                                                {
                                                    objectReady = true;
                                                    finalStateINT = 1;
                                                    exitFlags(); //turns the while statments to false to breakout of the while loops
                                                }
                                                
                                            }      
                                            else if((beamBreakRGTContinuity == false) && (beamBreakLFTContinuity == false))//if already in correct position
                                            {
                                                objectReady = true;
                                                finalStateINT = 2;
                                                exitFlags(); //turns the while statments to false to breakout of the while loops
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                    
                DataCollectionINDEXER();
            }
        });
        Indexer.start();
    }




   public void collectColorValues() {

        RedValue    = detectedColor.red;
        GreenValue  = detectedColor.green;
        BlueValue   = detectedColor.blue;

        detectedColor    = m_colorSensor.getColor();
    }

    public void collectColorSensorDistanceValues(){
        IR               = m_colorSensor.getIR();
        proximity        = m_colorSensor.getProximity();
    }

   public void collectBeamBreakValues() {
        beamBreakLFTContinuity = beamBreakLFT.get();
        beamBreakRGTContinuity = beamBreakRGT.get();
    }

    void runIndexerBeltREV() {
        inDexerMtrCtrlFRNT.set(-INDEXER_FRNT_SPEED);
        inDexerMtrCtrlBACK.set(-INDEXER_BACK_SPEED);
    }

    void runIndexerBelt() {
        inDexerMtrCtrlFRNT.set(INDEXER_FRNT_SPEED);
        inDexerMtrCtrlBACK.set(INDEXER_BACK_SPEED);

    }

    void stopbelt() {
        inDexerMtrCtrlFRNT.stopMotor();
    }

    void runFlipper(){
        flipperTimer.reset();
        flipperTimer.start();
        flipperTime = flipperTimer.get();

        inDexerMtrCtrlFLIPPER.set(INDEXER_FLIPPER_SPEED);

        while (flipperActive = true)
        {
            if(flipperTime > FLIPPER_DELAY_THRESHOLD)
            {
                inDexerMtrCtrlFLIPPER.stopMotor();
                flipperActive = false;
            }
        }

    }

    void resetFlipper(){
        flipperTimer.reset();
        flipperTimer.start();
        flipperTime = flipperTimer.get();

        inDexerMtrCtrlFLIPPER.set(-INDEXER_FLIPPER_SPEED);

        while (flipperActive = true)
        {
            if(flipperTime > FLIPPER_DELAY_THRESHOLD)
            {
                inDexerMtrCtrlFLIPPER.stopMotor();
                flipperActive = false;
            }
        }
    }

    void resetFlags(){
        coneTestActive = true;
        objectNotInPosition = true;
        flipperActive = true;
        colorTestActive = true;
    }

    void exitFlags(){
        coneTestActive = false;
        objectNotInPosition = false;
        flipperActive = false;
        colorTestActive = false;
    }

    public void SmartDashboardIndexer()
    {
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putBoolean("Beam1", beamBreakLFTContinuity);
        SmartDashboard.putBoolean("Beam2", beamBreakRGTContinuity);
        SmartDashboard.putString("object detected", colorSelected);
        SmartDashboard.putNumber("TraceID", finalStateINT);

    }

    void DataCollectionINDEXER()
    {
        if (DataCollection.LOG_ID_INDEXER == DataCollection.chosenDataID.getSelected())
        {
            data = new CatzLog(Robot.currentTime.get(),0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);  
            Robot.dataCollection.logData.add(data);
        }
    }

    boolean objectDetectedMethod()
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
    boolean ConeDetectedMethod()
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

    boolean CubeDetectedMethod()
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
}