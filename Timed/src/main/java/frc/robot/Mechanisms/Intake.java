package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;

public class Intake extends Mechanism{
    private CANSparkMax motor1 = new CANSparkMax(super.getCanID1(), CANSparkMax.MotorType.kBrushless),
    motor2 = new CANSparkMax(super.getCanID2(), CANSparkMax.MotorType.kBrushless);
    public Intake(String mode, int axis, int canID1, int canID2, int forward, int backward, int power)
    {
        super(mode,axis,canID1,canID2,forward,backward,power);
    }
    public void run(){
        if(super.getMode().equalsIgnoreCase("axis"))
            motor1.set(super.getAxis());
        else if(super.getMode().equalsIgnoreCase("button")){
            
        }

    }
}