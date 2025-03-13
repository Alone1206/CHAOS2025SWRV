package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;



public class ShooterSubsystem {
    private final SparkMax algae_motor = new SparkMax(22, MotorType.kBrushless);
    // private final SparkMax algae_motor2 = new SparkMax(23, MotorType.kBrushless);

   private final CommandJoystick m_operatorController =
      new CommandJoystick(OperatorConstants.kOperatorControllerPort);

        // Configure motor settings (example configurations)
        public void shootBall(double speed) {
            
            // System.out.println("value:");
            // System.out.println(speed);

            algae_motor.set(speed); // Use the passed speed parameter
            // algae_motor2.set(speed);
        }

    //         public Command shooter(){
    //     return new Command(){
    //         @Override
    //         public void initialize(){
    //             shootBall(m_operatorController.getRawAxis(1));
    //             System.out.println("Dropping the ball");
    //         }
    //         @Override
    //         public boolean isFinished(){
    //             return true;
    //         }
    //     };
    // }
}
    

   

    // Optionally add a stop method