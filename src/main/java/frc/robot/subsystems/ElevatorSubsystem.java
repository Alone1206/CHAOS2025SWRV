package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ElevatorSubsystem {
    private final TalonSRX
    
    elevatorMotor = new TalonSRX(25);
    private boolean is_stop = false;  // Shared variable
    private int sayac_degeri = 0;
    public void setStop(boolean stop) {  // Update method for external trigger
        this.is_stop = stop;
    }


    public void elevatorMoveUp(){
        elevatorMotor.set(ControlMode.PercentOutput, 0.6);
    }
    public void elevatorMoveDown(){
        elevatorMotor.set(ControlMode.PercentOutput, -0.6);
    }
    public void elevatorStop(){
        elevatorMotor.set(ControlMode.PercentOutput, 0);
    }
    /////////////////////////Bu KISMI YENİ YAZDIM METHODUNU ÇAĞIRMA AYARALAMA YAPICAM
    

    // public void autoelevatorrise() {  // No need to pass is_stop
    //     while (!is_stop && sayac_degeri <= 150) {  // is_stop now updates dynamically
    //         elevatorMotor.set(ControlMode.PercentOutput, -0.6);
    //         sayac_degeri += 1;
    //     }
    //     elevatorMotor.set(ControlMode.PercentOutput, 0); // Stop motor when exiting loop
    // }
}  
    
    
