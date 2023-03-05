package frc.Mechanisms;


import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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


public class CatzIndexerNew {
    //          SPARKMAX DEFS
    public CANSparkMax inDexerMtrCtrlFRNT;

    private final int INDEXER_FRNT_MC_CAN_ID        = 1;
    private final int INDEXER_FRNT_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FRNT_SPEED = 0.4;
    public final double INDEXER_FRNT_SPEED_FAST = 1;

    //
    public final double INDEXER_SPEED_OFF = 0.0;
    
    //          SECOND MOTOR DEFS
    public CANSparkMax inDexerMtrCtrlBACK;

    private final int INDEXER_BACK_MC_CAN_ID        = 2;
    private final int INDEXER_BACK_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_BACK_SPEED = 0.2;
    
    //          Flipper Defs
    public CANSparkMax inDexerMtrCtrlFLIPPER;

    private final int INDEXER_FLIPPER_MC_CAN_ID        = 3;
    private final int INDEXER_FLIPPER_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FLIPPER_SPEED = 0.2;

    //          BEAMBREAK DEFS
  
    public DigitalInput beamBreak;
  
    private final int BEAM_BREAK_DIO_PORT_LFT = 4;//up and down?

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
    private double RedValue = -999.0;
    private double GreenValue = -999.0;
    private double BlueValue = -999.0;

    Color detectedColor;
    double IR;
    int proximity;

    public DigitalInput LimitSwitchRGT;
    public DigitalInput LimitSwitchLFT;

    private final int LIMIT_SWITCH_DIO_PORT_RGT = 5; 
    private final int LIMIT_SWITCH_DIO_PORT_LFT = 6;

    boolean limitSwitchPressedLFT;
    boolean limitSwitchPressedRGT;

    
    

    //          objectdetection and flags
    public boolean objectReady = false;
    
    private boolean objectDetected; 

    private boolean objectNotInPosition;

    private boolean coneTestActive;

    public boolean flipperActive;
    private boolean flipperDeactivating;

    private boolean colorTestActive;

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
    private double MAX_BELT_TIME = 4;
    private double CONE_TEST_DELAY_THRESHOLD = 4.8;
    private double FLIPPER_DELAY_THRESHOLD = 7;
   

    public CatzLog data;

    private RelativeEncoder m_encoder;
    private SparkMaxPIDController elevSpoolPid;
    private final double PID_ELEVATOR_SPOOL_KP = 0.00;
    private final double PID_ELEVATOR_SPOOL_KI = 0.00;
    private final double PID_ELEVATOR_SPOOL_KD = 0.00;



    //                      End of definitions

    public CatzIndexerNew() 
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
        inDexerMtrCtrlFLIPPER.setIdleMode(IdleMode.kCoast);
        inDexerMtrCtrlFLIPPER.setSmartCurrentLimit(INDEXER_FLIPPER_MC_CURRENT_LIMIT);
    
        
       

    
        indexerTimer = new Timer();

        flipperTimer = new Timer();


        beamBreak = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);

        LimitSwitchLFT = new DigitalInput(LIMIT_SWITCH_DIO_PORT_LFT);
        LimitSwitchRGT = new DigitalInput(LIMIT_SWITCH_DIO_PORT_RGT);

        
        m_encoder = inDexerMtrCtrlFLIPPER.getEncoder();
        m_encoder.setPositionConversionFactor(60);
        m_encoder.setPosition(0);
        

        elevSpoolPid = inDexerMtrCtrlFLIPPER.getPIDController();

        elevSpoolPid.setP(PID_ELEVATOR_SPOOL_KP);
        elevSpoolPid.setI(PID_ELEVATOR_SPOOL_KI);
        elevSpoolPid.setD(PID_ELEVATOR_SPOOL_KD);

        
        startIndexerThread();

      
    }

    public void startIndexerThread() 
    {
        Indexer = new Thread(() ->
        {
            while(true)
            {   
                
                collectColorValues();
                collectBeamBreakValues();
                collectColorSensorDistanceValues();
                collectLimitSwitchValues();

                if(objectNotInPosition = true)
                {
                runIndexerBelt();
                }

                if(limitSwitchPressedLFT && limitSwitchPressedRGT)
                {
                stopbelt();
                objectNotInPosition = false;
                objectReady = true;
                }

                if(CubeDetectedMethod())
                {
                    objectReady = true;
                }

                if(ConeDetectedMethod())
                {
                    coneTestActive = true;
                }
                 if(beamBreakContinuity = true)
                {
                    flipperActive = true;

                    if(flipperActive == true)
                    {
                        if(m_encoder.getPosition() > -100)
                        {
                            inDexerMtrCtrlFLIPPER.set(-0.1);
                        }
                        else
                        {
                            inDexerMtrCtrlFLIPPER.set(0.0);
                            flipperActive = false;
                            flipperDeactivating = true;
                        }
                    }
                    if(flipperDeactivating == true)
                    {
                        if(m_encoder.getPosition() < -20)
                        {
                            inDexerMtrCtrlFLIPPER.set(0.1);
                        }
                        else
                        {
                            inDexerMtrCtrlFLIPPER.set(0.0);
                            flipperDeactivating = false;
                            objectReady = true;
                        }
                    }
                }
                
                if(flipperActive == true)
                    {
                        if(m_encoder.getPosition() > -100)
                        {
                            inDexerMtrCtrlFLIPPER.set(-0.1);
                        }
                        else
                        {
                            inDexerMtrCtrlFLIPPER.set(0.0);
                            flipperActive = false;
                            flipperDeactivating = true;
                        }
                    }
                    if(flipperDeactivating == true)
                    {
                        if(m_encoder.getPosition() < -20)
                        {
                            inDexerMtrCtrlFLIPPER.set(0.1);
                        }
                        else
                        {
                            inDexerMtrCtrlFLIPPER.set(0.0);
                            flipperDeactivating = false;
                            objectReady = true;
                        }
                    }
            Timer.delay(0.02);
            }
            
         });
        Indexer.start();
    }
    

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

    void runIndexerBeltREV() {
        inDexerMtrCtrlFRNT.set(-INDEXER_FRNT_SPEED);
        inDexerMtrCtrlBACK.set(-INDEXER_BACK_SPEED);
    }

    void runIndexerBelt() {
        inDexerMtrCtrlFRNT.set(-INDEXER_FRNT_SPEED);
        inDexerMtrCtrlBACK.set(INDEXER_BACK_SPEED);
    }

    void runIndexerBeltFast(){
        inDexerMtrCtrlFRNT.set(INDEXER_FRNT_SPEED_FAST);
        inDexerMtrCtrlBACK.set(INDEXER_BACK_SPEED);
    }

    void stopbelt() {
        inDexerMtrCtrlFRNT.stopMotor();
    }




    public void SmartDashboardIndexer()
    {
        /* 
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putBoolean("Beam1", beamBreakLFTContinuity);
        SmartDashboard.putBoolean("Beam2", beamBreakRGTContinuity);
         */
        SmartDashboard.putString("object detected", colorSelected);
        SmartDashboard.putNumber("TraceID", finalStateINT);
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Encoder getpositionfactor", m_encoder.getPositionConversionFactor());

    }

    void DataCollectionINDEXER()
    {
        if (DataCollection.LOG_ID_INDEXER == DataCollection.chosenDataID.getSelected())
        {
            data = new CatzLog(Robot.currentTime.get(),detectedColor.red,detectedColor.blue,detectedColor.green,proximity,finalStateINT,0,0,0,0,0,0,0,0,0,1);  
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