package frc.robot.subsystems;

import frc.robot.motors.PIDFGains;
import frc.robot.constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;



public class Arm extends SubsystemBase{

    private DBugSparkMax _base;
    private Servo  _wrist;

    public Arm(){
        _base = DBugSparkMax.create(ArmConstants.baseCANID,  new PIDFGains(ArmConstants.kp),
                ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _wrist = new Servo(ArmConstants.wristPWMchannel);
    }

}
