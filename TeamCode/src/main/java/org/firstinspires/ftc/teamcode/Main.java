// @version 1.3.0
/*
 * @authors: Wojciech Boncela, Lukasz Gapiński, Mateusz Gapiński, Marceli Antosik, Jan Milczarek, Julia Sysdół, Witold Kardas
 *
 *
 *
 *
    Motors:

    0 - left
    1 - right

    * Velocity > 0 => drive backward
    * Velocity <   0 => drive forward


 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="OpMode2k19 1.3.0", group="Iterative Opmode")
public class Main extends LinearOpMode {

    enum Drive_direction{
        ROTATE_LEFT,
        ROTATE_RIGHT,
    }

    enum Lift_motion{
        UP,
        DOWN,
    }

    double maxSpeed = 1000.0;
    double turboSpeed = 2600.0;
    double rotationMaxSpeed = 600.0;
    double liftSpeed = 1200.0;

    DcMotorEx[] driveMotors = new DcMotorEx[2];
    DcMotorEx liftMotor;
    Servo wristServo;
    Servo jawServo;
    DistanceSensor distSensor;

    private void drive(double s, Drive_direction direction, double rotSpd){
        switch (direction){
            case ROTATE_RIGHT:
                driveMotors[0].setVelocity(s + rotSpd);
                driveMotors[1].setVelocity(s - rotSpd);
                break;
            case ROTATE_LEFT:
                driveMotors[0].setVelocity(s - rotSpd);
                driveMotors[1].setVelocity(s + rotSpd);
                break;
        }
    }

    private void drive(double s){
        driveMotors[0].setVelocity(s);
        driveMotors[1].setVelocity(s);
    }

    public void liftMotion(Lift_motion dir, double speed){
        switch(dir) {
            case UP:
                liftMotor.setVelocity(speed);
                break;

            case DOWN:
                liftMotor.setVelocity(-speed);
                break;
        }
    }

    public void setLiftPosition(int position, double power){
        liftMotor.setTargetPosition(position);
        liftMotor.setPower(power);
    }

    public void holdMotor(DcMotorEx motor){
        DcMotor.RunMode mode = motor.getMode();

        switch (mode){
            case RUN_USING_ENCODER:
                motor.setVelocity(0.0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case RUN_TO_POSITION:
                motor.setTargetPosition(motor.getCurrentPosition());
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
        }
    }

    private void initDcmotor(String motorName){
        //DC Motors mapping
        for(int i = 0 ; i < 2 ; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, motorName + Integer.toString(i));
        }
        //Set motors to rotate at set speed
        for (int i=0; i<2;i++){
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[1].setDirection(DcMotor.Direction.FORWARD);

    }
    private void initLiftMotor(String motorName){
        //initializing lift DC motor
        liftMotor = hardwareMap.get(DcMotorEx.class ,motorName);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void initWristServo(String servoName, double init_pos){
        //initializing arm servo
        wristServo = hardwareMap.get(Servo.class ,servoName+"0");
        wristServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        wristServo.setDirection(Servo.Direction.FORWARD);

        wristServo.setPosition(init_pos);
    }

    private void initJawServo(String servoName, double init_pos){
        jawServo = hardwareMap.get(Servo.class, servoName + "0");
        jawServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        jawServo.setDirection(Servo.Direction.FORWARD);
    }

    private void initDistanceSensor(String sensorName){
        distSensor = hardwareMap.get(DistanceSensor.class, sensorName);
        telemetry.addData("Distance Sensor: ", "Distance Sensor initialize successfully.");
        telemetry.update();
    }

    private void steerDriveMotors(){
        //driving the robot
        double speed;

        if(gamepad1.start){
            speed = gamepad1.left_stick_y * turboSpeed;
        }
        else {
            speed = gamepad1.left_stick_y * maxSpeed;
        }

        double rotSpeed = rotationMaxSpeed * gamepad1.right_stick_x;

        drive(speed, Drive_direction.ROTATE_LEFT,rotSpeed);
    }
    private void liftMotor(double speed){
        //controlling the lift
        if (gamepad1.y){
            liftMotion(Lift_motion.UP,speed);
        }
        else if(gamepad1.a){
            liftMotion(Lift_motion.DOWN, speed);
        }
        else{
            holdMotor(liftMotor);
        }

    }

    private void wristServo(){
        double pos = wristServo.getPosition();
        if(gamepad1.x){
            wristServo.setPosition(pos -= 0.01);
        }
        else if(gamepad1.b){
            wristServo.setPosition(pos += 0.01);
        }
    }
    private void showDistance()
    {
        telemetry.addData("Distance: ", distSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    //to be repaired
    private void turn_around()
    {
        drive(0, Drive_direction.ROTATE_RIGHT, 200);
        double min_dis = 1000.0;
        double dis;
        boolean finding_min = true;
        while(finding_min)
        {
            dis = distSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance: ", dis);
            telemetry.update();
            if(dis < min_dis)
                min_dis = dis;
            else if(dis > min_dis + 1.0)
            {
                drive(0, Drive_direction.ROTATE_RIGHT, 0);
                finding_min = false;
            }
            sleep(10);
        }

    }

    //main function:
    @Override
    public void runOpMode() throws InterruptedException {
        final long interval = 10;

        initDcmotor("motorTest");
        initLiftMotor("liftMotor");
        initWristServo("armServo", 1.0);
        initDistanceSensor("distanceTest");
        initJawServo("jawServo", 1.0);

        telemetry.addData("Status", "initialized");
        telemetry.update();

        Thread drive_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive())
                    steerDriveMotors();
            }
        });

        Thread lift_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive())
                    liftMotor(liftSpeed);
            }
        });

        Thread distance_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()){
                    showDistance();
                }
            }
        });

        Thread servo_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()){
                    sleep(interval);
                    wristServo();
                }
            }
        });

        waitForStart();

        /*
        Autonomous:
         */

       // turn_around();

        //Manual period:
        drive_thread.start();
        lift_thread.start();
        servo_thread.start();

        drive_thread.join();
        distance_thread.join();
        servo_thread.join();

        while(opModeIsActive()){
            idle();
        }
    }
}
