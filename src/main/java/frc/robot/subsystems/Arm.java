package frc.robot.subsystems;


import frc.robot.motors.PIDFGains;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.ArmConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;



public class Arm extends SubsystemBase{

    private static final boolean UPDATE_DASHBOARD = true;

    private DBugSparkMax _base;
    private Servo _wrist;
    private double servoZeroPos;

    private BaseState baseState;

    public static enum BaseState {
        CIRCLE(ArmConstants.circleVelocity),
        SQUARE(ArmConstants.squareVelocity),
        OFF(ArmConstants.offVelocity);

        public final double velocity;

        private BaseState(double velocity){
            this.velocity = velocity;
        }
    }

    public Arm(){
        _base = DBugSparkMax.create(ArmConstants.baseCANID,  new PIDFGains(ArmConstants.kp, 0, 0, ArmConstants.kfBase),
                ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _wrist = new Servo(ArmConstants.wristPWMchannel);

        servoZeroPos = _wrist.getAngle();

    }

    public void resetPosotion(){
        this.servoZeroPos = _wrist.getAngle();
    }

    public void setWristAngle(double angle){
        _wrist.setAngle(angle + servoZeroPos);
    }

    public double getWristAngle(){
        return this._wrist.getAngle() - servoZeroPos;
    }

    public void setBaseState(BaseState state){
        this.baseState = state;
    }

    public double getBaseAngle(){
        return this._base.getPosition();
    }

    public void drawCircle(double R){

        double wristAngle = Math.acos((Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - Math.pow(R, 2))/ 2 * ArmConstants.armlength1 * ArmConstants.armlength2);
        this.setWristAngle(wristAngle);
        double startBaseAngle = getBaseAngle();
        this.baseState = BaseState.CIRCLE;
        this._base.set(baseState.velocity);

        while (startBaseAngle != getBaseAngle()) {
            this.setWristAngle(wristAngle);
        }

        this._base.set(BaseState.OFF.velocity);
    }

    public void drawSquare(){
        double radius = Math.sqrt((Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - 2 * ArmConstants.armlength1 * ArmConstants.armlength2) * Math.cos(getWristAngle()));
        double startBaseAngle = getBaseAngle()%360;
        this.baseState = BaseState.SQUARE;
        this._base.set(baseState.velocity);
        double ARM_LENGTH3;
        double targetY;
        double targetX;
        while (startBaseAngle != getBaseAngle()) {
            double angle1 = (getBaseAngle()-startBaseAngle) % 360;

            if(320 < angle1 || angle1 <= 45 ) {
                targetX = radius;

                double x1 = ArmConstants.armlength1 * Math.cos(angle1);
                double x2 = targetX-x1;
                double y1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(x2*x2));
                double y2 = ArmConstants.armlength1 * Math.sin(angle1);
                targetY = y2+y1;
            }
            else if(angle1 <= 135) {
                targetY = radius;

                double y1 = ArmConstants.armlength1 * Math.sin(angle1);
                double y2 = targetY-y1;
                double x1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(y2*y2));
                double x2 = ArmConstants.armlength1 * Math.cos(angle1);
                targetX =  x2-x1;
            }
            else if((angle1 <= 225)){

                targetX = -radius;

                double x1 = ArmConstants.armlength1 * Math.cos(angle1);
                double x2 = targetX-x1;
                double y1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(x2*x2));
                double y2 = ArmConstants.armlength1 * Math.sin(angle1);
                targetY = y2-y1;
            }
            else {
                targetY = -radius;

                double y1 = ArmConstants.armlength1 * Math.sin(angle1);
                double y2 = targetY-y1;
                double x1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(y2*y2));
                double x2 = ArmConstants.armlength1 * Math.cos(angle1);
                targetX =  x2+x1;
            }

            ARM_LENGTH3=Math.sqrt(targetX*targetX+targetY*targetY);

            double wristAngle = Math.acos((Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - Math.pow(ARM_LENGTH3, 2))/ 2 * ArmConstants.armlength1 * ArmConstants.armlength2);
            setWristAngle(wristAngle);
        }
        this._base.set(BaseState.OFF.velocity);

    }

    public void getCompetativeDrow(double firstR){
        double R = firstR;
        for (int i = 0; i<9; i++){
            
            double radius = firstR + i*1.5;
            SmartDashboard.putNumber("radius", radius);
            this.drawCircle(radius);
            this.drawSquare();
        }
    }
    private void updateSDB() {
        SmartDashboard.putNumber("Wrist angle", this.getWristAngle());
        SmartDashboard.putNumber("Base angle", this.getBaseAngle());
        SmartDashboard.putNumber("servoZeroPos", servoZeroPos);
    }

    @Override
    public void periodic() {
        if(UPDATE_DASHBOARD) {
            updateSDB();
        }
    }



}
