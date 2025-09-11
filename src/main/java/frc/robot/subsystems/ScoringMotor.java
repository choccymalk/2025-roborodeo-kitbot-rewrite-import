package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringMotor extends SubsystemBase{
    //SparkMax scoringMotor = new SparkMax(13, MotorType.kBrushless);
    static Spark scoringMotor = new Spark(1);
    
    public ScoringMotor(){}
        
    public static void runMotorForwards(){
        scoringMotor.set(0.5);
    }

    public static void runMotorBackwards(){
        scoringMotor.set(-0.5);
    }

    public static void stop(){
        scoringMotor.stopMotor();
    }
}
