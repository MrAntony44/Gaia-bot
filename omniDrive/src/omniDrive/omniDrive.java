package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;

@TeleOp(name="driveJava", group="Pushbot")
public class DriveJava extends LinearOpMode {
    
    
    private VuforiaFG2019 vuforiaFG2019;

    VuforiaBase.TrackingResults vuforiaResult;
    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;
    private DcMotor shooter_thing = null;
    private DcMotor collection_thing = null;
    private DcMotor lift_thing = null;
    private DcMotor heavy = null;
    private VuforiaFG2019 vuforia2019;
    Servo servo;
    Servo collection_servo1;
    Servo collection_servo2;
    Servo lift_servo1;
    Servo lift_servo2;
    Servo angle_servo;
    
       // VuforiaBase.TrackingResults vuforiaResul
    
    BNO055IMU imu;
    @Override
    public void runOpMode()  {
        vuforiaFG2019 = new VuforiaFG2019();
 
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        collection_thing = hardwareMap.dcMotor.get( "collection_thing");
        
        servo = hardwareMap.get(Servo.class , "servo");
        collection_servo1 = hardwareMap.get(Servo.class , "collection_servo1");
        collection_servo2 = hardwareMap.get(Servo.class , "collection_servo2");
        lift_servo2 = hardwareMap.get(Servo.class , "lift_servo2");
        lift_servo1 = hardwareMap.get(Servo.class , "lift_servo1");
        angle_servo = hardwareMap.get(Servo.class , "angle_servo");


        lift_thing = hardwareMap.get(DcMotorEx.class ,"lift_thing");
        heavy = hardwareMap.get(DcMotorEx.class, "heavy");
        shooter_thing = hardwareMap.dcMotor.get( "shooter_thing");
        front_left_wheel.setDirection(DcMotor.Direction.REVERSE); 
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        collection_thing.setDirection(DcMotor.Direction.FORWARD);
        lift_servo1.setDirection(Servo.Direction.REVERSE);
        collection_servo1.setDirection(Servo.Direction.REVERSE);

       // vuforiaFG2019 = new Vuforia2019();
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vuforiaFG2019.initialize(hardwareMap.get(WebcamName.class, "Webcam 1"),
        false, true, 0, 0, 0, -90, 180, 0);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //lift_thing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        lift_thing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        heavy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_thing.setTargetPosition(0);
        heavy.setTargetPosition(0);

        //lift_thing.setVelocity(300);
        heavy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_thing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
      //  vuforiaFG2019.initialize(hardware.get(WebcamNameWebcamName.class, "Webcam 1"),false,true,0,0,0,-90,180,0);
        while(!opModeIsActive()){}
        
        while(opModeIsActive()){
            drive();
            resetAngle();
            vuforiaFG2019.activate();
            //driveSimple();
            
            telemetry.update();

            
            
        // Get the data for the Vuforia target
        vuforiaResult = vuforiaFG2019.track("World_Map");
        // Print the target's data to telemetry
        printVuforiaTelemetry();
        }
      // Deactivate Vuforia when the loop finishes
      vuforiaFG2019.deactivate();
        
    }
    private void printVuforiaTelemetry() {
    telemetry.addData("World Map Target", "");
    telemetry.addData("isVisible", vuforiaResult.isVisible);
    //if(vuforiaResult.isVisible && (Double.parseDouble(JavaUtil.formatNumber(vuforiaResult.x, 2)) >= 140)&&Double.parseDouble(JavaUtil.formatNumber(vuforiaResult.x, 2)) <= 150){
       // shoot();
       // while(vuforiaResult.isVisible){
           // shooter_thing.setPower(1);
       // }
   // }
    telemetry.addData("X (mm)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResult.x, 2)));
    telemetry.addData("Y (mm)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResult.y, 2)));
    telemetry.addData("Heading (degrees)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResult.zAngle, 2)));
    
  }
    public void driveSimple(){
        double power = .5;
        if(gamepad1.dpad_up){ //Forward
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(-power);
        }
        else if(gamepad1.dpad_left){ //Left
            front_left_wheel.setPower(power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(power);
            front_right_wheel.setPower(-power);
        }
        else if(gamepad1.dpad_down){ //Back
            front_left_wheel.setPower(power);
            back_left_wheel.setPower(power);
            back_right_wheel.setPower(power);
            front_right_wheel.setPower(power);
        }
        else if(gamepad1.dpad_right){ //Right
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(power);
        }
        else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
            front_left_wheel.setPower(-gamepad1.right_stick_x);
            back_left_wheel.setPower(-gamepad1.right_stick_x);
            back_right_wheel.setPower(gamepad1.right_stick_x);
            front_right_wheel.setPower(gamepad1.right_stick_x);
        }
        else{
            front_left_wheel.setPower(0);
            back_left_wheel.setPower(0);
            back_right_wheel.setPower(0);
            front_right_wheel.setPower(0);
        }
    }

    public void drive(){
         double Protate = gamepad1.right_stick_x/4;
         double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
         double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
         double theta = 0;
         double Px = 0;
         double Py = 0;
         boolean state = false;
         double i = 0;
         double lift = 0;

         double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
                if (gyroAngle <= 0) { 
            gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
            }
            gyroAngle = -1 * gyroAngle;
        
    if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly. 
        gyroAngle = -Math.PI/2;
        }
    }
}