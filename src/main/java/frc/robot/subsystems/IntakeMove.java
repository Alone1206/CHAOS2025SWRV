package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeMove {
    private final TalonSRX
    intakeMotor = new TalonSRX(26);

    public void intakeMotorMoveFront(){
        intakeMotor.set(ControlMode.PercentOutput, 0.8);
    }
    public void intakeMotorMoveRear(){
        intakeMotor.set(ControlMode.PercentOutput, -0.8);
    }
    public void intakeStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    
}
