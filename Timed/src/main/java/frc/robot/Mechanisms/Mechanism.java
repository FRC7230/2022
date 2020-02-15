package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
//contains intake, conveyor, shooter, and climb
// sudokode
public class Mechanism{
    private boolean operating, polarity;
    private String mode;
    private int axis, canID1, canID2, forward, backward, power;
    private CANSparkMax motor1 = new CANSparkMax(canID2, CANSparkMax.MotorType.kBrushless),
    motor2 = new CANSparkMax(canID2, CANSparkMax.MotorType.kBrushless);
    private Joystick m_stick = new Joystick(0);
    //abstract
    
    //input to end mechanism (also emergeny stop)
    //mechanism state output
    public Mechanism(){
        this("axis",1,5,6,1,2,50);
        operating=false;
        polarity = false;
    }
    public Mechanism(String mode, int axis, int canID1, int canID2, int forward, int backward, int power)
    {
        this.mode = mode;
        this.axis = axis;
        this.canID1 =canID1;
        this.canID2 = canID2;
        this.forward = forward;
        this.backward = backward;
        this.power = power;
    }
    public boolean getState(){
        return operating;
    }

    public void flipPolarity(boolean direction){
        polarity = direction;
    }
    //input to starm mechanism
    public void start(Boolean is){
        operating = is;
    }
   
    public void addControlAxis(int a)
    {
        axis = a;
    }
    public void addForwardButton(int b)
    {
        forward =b;
    }
    public void addBackwardButton(int b)
    {
        backward = b;
    }
    public void setMode(String s){
        if(s.equalsIgnoreCase("button")||s.equalsIgnoreCase("axis"))
            mode = s;
    }
    public void setPower(int p){
        power = p;
    }
    public int getCanID1(){
        return canID1;
    }
    public int getCanID2(){
        return canID2;
    }
    public int getAxis(){
        return axis;
    }
    public int getForward(){
        return forward;
    }
    public int getBackward(){
        return backward;
    }
    public int getPower(){
        return power;
    }
    public String getMode(){
        return mode;
    }
    public void run(){
        if(mode.equalsIgnoreCase("axis")){
            motor1.set(m_stick.getRawAxis(axis));
            motor2.set(m_stick.getRawAxis(axis));
        }
        else if(mode.equalsIgnoreCase("button")){
            if(m_stick.getRawButton(forward)){
                motor1.set(power);
                motor2.set(power);
            }
            else if(m_stick.getRawButton(backward)){
                motor1.set(-power);
                motor2.set(-power);   
            }
        }

    }
    
}