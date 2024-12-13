package frc.robot.subsystems;


import frc.robot.motors.PIDFGains;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;



public class Arm extends SubsystemBase{

    private static final boolean UPDATE_DASHBOARD = true;

    private DBugSparkMax _base;
    private Servo _wrist;

    private int forward = 1;
    private BaseState baseState;
    private static double inBetween = 0;
    private static double test = 0;

    public static enum BaseState {
        CIRCLE(ArmConstants.circleVolt),
        SQUARE(ArmConstants.squareVolt),
        OFF(ArmConstants.offVolt);

        public final double volt;

        private BaseState(double volt){
            this.volt = volt;
        }
    }

    public Arm(){
        _base = DBugSparkMax.create(ArmConstants.baseCANID,  new PIDFGains(ArmConstants.kp, 0, 0, ArmConstants.kfBase),
                ArmConstants.positionFactor, ArmConstants.velocityFactor, 0);
        _wrist = new Servo(ArmConstants.wristPWMchannel);
        _wrist.setAlwaysHighMode();


    }



    public void setWristAngle(double angle){
        _wrist.setAngle(angle);
    }



    public void testServo(){
        _wrist.set(0.5);
    }

    public void testNeo(){
        _base.set(-0.1);
    }

        public void offNeo(){
        _base.set(0);
    }

    public double getWristAngle(){
        return this._wrist.getAngle();
    }

    public void setBaseState(BaseState state){
        this.baseState = state;
    }

    public double getBaseAngle(){
        return Math.abs(this._base.getPosition());
    }

    public void drawCircle(double R){
        
        
        double wristAngle = Math.acos((Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - Math.pow(R, 2))/ (2 * ArmConstants.armlength1 * ArmConstants.armlength2));
        SmartDashboard.putNumber("preACOS",(Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - Math.pow(R, 2))/ (2 * ArmConstants.armlength1 * ArmConstants.armlength2));
        SmartDashboard.putNumber("wrist_angle", wristAngle);
        this.setWristAngle(Math.toDegrees(wristAngle));
        //double startBaseAngle = getBaseAngle();
        this.baseState = BaseState.CIRCLE;
        this._base.set(forward*ArmConstants.circleVolt);


        

    }

    public void stop(){
        this.baseState = BaseState.OFF;
        
        this._base.setReference(baseState.volt, ControlType.kVoltage);
    }

    public Command getCommandDrawSquare(double radius){
        double startBaseAngle = getBaseAngle()%360;
        this.baseState = BaseState.SQUARE;
        this._base.set(baseState.volt);
        double ARM_LENGTH3 = 0;
        double targetY = 0;
        double targetX = 0;
        SmartDashboard.putNumber("offset", 0);
        return Commands.run(()-> this.drawSquare(ARM_LENGTH3, targetY, targetX, radius, startBaseAngle),this);      
    }

    public void drawSquare(
        double ARM_LENGTH3,
        double targetY,
        double targetX,
        double radius,
        double startBaseAngle
    )
    {
        double angle1 =0;
        if (Robot.isSimulation()){
            angle1 = test % 360;
            test++;
        }
        else{
            angle1 = (getBaseAngle()-startBaseAngle) % 360;
        } 
        
        SmartDashboard.putNumber("Modolized Base Angle", angle1);

            if(315 < angle1+inBetween || angle1 + inBetween <= 45 ) {
                targetX = radius;

                double x1 = ArmConstants.armlength1 * Math.cos(Math.toRadians(angle1));
                SmartDashboard.putNumber("r2", ArmConstants.armlength2);
                double x2 = targetX-x1;
                SmartDashboard.putNumber("x2", x2);
                SmartDashboard.putBoolean("fail", ArmConstants.armlength2 < x2);
                double y1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(x2*x2));
                double y2 = ArmConstants.armlength1 * Math.sin(Math.toRadians(angle1));
                targetY = y2+y1;
            }
            else if(angle1 + inBetween <= 135) {
                targetY = radius;

                double y1 = ArmConstants.armlength1 * Math.sin(Math.toRadians(angle1));
                double y2 = targetY-y1;
                SmartDashboard.putBoolean("fail", ArmConstants.armlength2 <y2);
                double x1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(y2*y2));
                double x2 = ArmConstants.armlength1 * Math.cos(Math.toRadians(angle1));
                targetX =  x2-x1;
            }
            else if((angle1 + inBetween <= 225)){

                targetX = -radius;

                double x1 = ArmConstants.armlength1 * Math.cos(Math.toRadians(angle1));
                double x2 = targetX-x1;
                SmartDashboard.putBoolean("fail", ArmConstants.armlength2 <x2);
                double y1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(x2*x2));
                double y2 = ArmConstants.armlength1 * Math.sin(Math.toRadians(angle1));
                targetY = y2-y1;
            }
            else {
                targetY = -radius;

                double y1 = ArmConstants.armlength1 * Math.sin(Math.toRadians(angle1));
                double y2 = targetY-y1;
                SmartDashboard.putBoolean("fail", ArmConstants.armlength2 <y2);

                double x1 = Math.sqrt((ArmConstants.armlength2*ArmConstants.armlength2)-(y2*y2));
                double x2 = ArmConstants.armlength1 * Math.cos(Math.toRadians(angle1));
                targetX =  x2+x1;
            }
            if(Robot.isSimulation()) {
            SmartDashboard.putNumberArray("X", new Double[] {targetX+1000});
            SmartDashboard.putNumberArray("Y", new Double[] {targetY+1000});
            } else {
            SmartDashboard.putNumber("X", targetX);
            SmartDashboard.putNumber("Y", targetY);
            }

            SmartDashboard.putNumber("inBetween", inBetween+angle1 );
            ARM_LENGTH3=Math.sqrt(targetX*targetX+targetY*targetY);
            this.inBetween = Math.toDegrees(Math.acos((ARM_LENGTH3*ARM_LENGTH3+ArmConstants.armlength1*ArmConstants.armlength1-ArmConstants.armlength2*ArmConstants.armlength2)/(2*ARM_LENGTH3*ArmConstants.armlength1)));
            double wristAngle = Math.toDegrees(Math.acos((Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - Math.pow(ARM_LENGTH3, 2))/ (2 * ArmConstants.armlength1 * ArmConstants.armlength2)));
            SmartDashboard.putNumber("Arm Length 3", ARM_LENGTH3);
            SmartDashboard.putNumber("PreACOSSquare", (Math.pow(ArmConstants.armlength1, 2)+ Math.pow(ArmConstants.armlength2, 2) - Math.pow(ARM_LENGTH3, 2))/ (2 * ArmConstants.armlength1 * ArmConstants.armlength2));
            SmartDashboard.putNumber("Square Wrist Angle", wristAngle);
            this._base.set(forward*ArmConstants.squareVolt);
            setWristAngle(wristAngle+SmartDashboard.getNumber("offset", 0));
    }



    /*public void getCompetativeDrow(double firstR){
        double R = firstR;
        for (int i = 0; i<9; i++){
            
            double radius = firstR + i*1.5;
            SmartDashboard.putNumber("radius", radius);
            this.drawCircle(radius);
            this.drawSquare();
        }
    }*/
    private void updateSDB() {
        SmartDashboard.putNumber("Wrist angle", this.getWristAngle());
        SmartDashboard.putNumber("Base angle", this.getBaseAngle());
        SmartDashboard.putNumber("aaa", this._wrist.getAngle());
    }

    @Override
    public void periodic() {
        if(UPDATE_DASHBOARD) {
            updateSDB();
        }
    }

    public void changeDirection() {
        forward *= -1;
    }



}
