// @version 1.2
/*
 * @authors: Wojciech Boncela, Lukasz Gapiński, Mateusz Gapiński, Marceli Antosik, Jan Milczarek, Julia Sysdół, Witold Kardas
 *
 *
 *
 *
    Motors:

    0 - left
    1 - right

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test", group="Iterative Opmode")
public class Main extends LinearOpMode {

    enum Drive_direction{
        ROTATE_LEFT,
        ROTATE_RIGHT,
        STRAIGHT
    }

    enum Lift_motion{
        UP,
        DOWN,
    }

    double maxSpeed = 1000.0;
    double turboSpeed = 2600.0;
    double rotationMaxSpeed = 500.0;
    double liftSpeed = 1200.0;

    DcMotorEx[] driveMotors = new DcMotorEx[2];
    DcMotorEx liftMotor;
    Servo wristServo;
    DistanceSensor distSensor;

    private void drive(double s, Drive_direction direction, double rotSpd){
        switch (direction){
            case ROTATE_RIGHT:
                driveMotors[0].setVelocity(s - rotSpd);
                driveMotors[1].setVelocity(s + rotSpd);
                break;
            case ROTATE_LEFT:
                driveMotors[0].setVelocity(s + rotSpd);
                driveMotors[1].setVelocity(s - rotSpd);
                break;
        }
    }

    public void testSth(DcMotorEx motor, double vel){
        motor.setVelocity(vel);

    }

    private void drive(double x, double y)
    {
        driveMotors[0].setVelocity(x);
        driveMotors[1].setVelocity(y);
        driveMotors[2].setVelocity(x);
        driveMotors[3].setVelocity(y);
    }

    public void lift_motion(Lift_motion dir, double speed){
        switch(dir) {
            case UP:
                liftMotor.setVelocity(speed);
                break;

            case DOWN:
                liftMotor.setVelocity(-speed);
                break;
        }
    }

    public void set_lift_position(int position,double power){
        liftMotor.setTargetPosition(position);
        liftMotor.setPower(power);
    }

    public void hold_motor(DcMotorEx motor){
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

    private void init_dcmotor(String motorName){
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
    private void init_liftMotor(String motorName){
        //initializing lift DC motor
        liftMotor = hardwareMap.get(DcMotorEx.class ,motorName);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void init_wrist_servo(String servoName, double init_pos){
        //initializing arm servo
        wristServo = hardwareMap.get(Servo.class ,servoName+"0");
        wristServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        wristServo.setDirection(Servo.Direction.FORWARD);

        wristServo.setPosition(init_pos);
    }

    private void init_distance_sensor(String sensorName){
        distSensor = hardwareMap.get(DistanceSensor.class, sensorName);
        telemetry.addData("Distance Sensor: ", "Distance Sensor initialize successfully.");
        telemetry.update();
    }

    private void steer_Drive_Motors(){
        //driving the robot
        double speed;

        if(gamepad1.start){
            speed = gamepad1.left_stick_y * turboSpeed;
        }
        else {
            speed = gamepad1.left_stick_y * maxSpeed;
        }

        if(gamepad1.right_bumper) {
            drive(speed, Drive_direction.ROTATE_RIGHT,rotationMaxSpeed);
        }else if(gamepad1.left_bumper){
            drive(speed, Drive_direction.ROTATE_LEFT,rotationMaxSpeed);
        }else{
            drive(speed, Drive_direction.ROTATE_LEFT,0);
        }
    }
    private void lift_Motor(double speed){
        //controlling the lift
        if (gamepad1.y){
            lift_motion(Lift_motion.UP,speed);
        }
        else if(gamepad1.a){
            lift_motion(Lift_motion.DOWN, speed);
        }
        else{
            hold_motor(liftMotor);
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
    private void show_distance()
    {
        telemetry.addData("Distance: ", distSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    //main function:
    @Override
    public void runOpMode() throws InterruptedException {
        final long interval = 10;

        init_dcmotor("motorTest");
        init_liftMotor("liftMotor");
        init_wrist_servo("armServo", 1.0);
        init_distance_sensor("distanceTest");
        telemetry.addData("Status", "initialized");
        telemetry.update();
        waitForStart();

        Thread drive_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    steer_Drive_Motors();
            }
        });

        Thread lift_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    lift_Motor(liftSpeed);
            }
        });

        Thread distance_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true){
                    show_distance();
                }
            }
        });

        Thread servo_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true){
                    sleep(interval);
                    wristServo();
                }
            }
        });

        drive_thread.start();
        lift_thread.start();
        distance_thread.start();
        servo_thread.start();

        drive_thread.join();
        lift_thread.join();
        distance_thread.join();
        servo_thread.join();

        waitForStart();
        while(true){

        }
    }
}
