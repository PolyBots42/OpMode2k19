package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    enum Accurator_mode{
        EXTEND,
        RETRACT,
        LEFT_MOVE,
        RIGHT_MOVE
    }
    double maxSpeed = 1000.0;
    double rotationMaxSpeed = 500.0;
    double accMaxSpeed = 2600.0;

    DcMotorEx[] driveMotors = new DcMotorEx[4];
    DcMotorEx liftTest;
    DcMotorEx [] acc_motors = new DcMotorEx[2];
    Servo [] armServo = new Servo[2];
    DistanceSensor distSensor;
    private void drive(double x, double y, Drive_direction direction, double rotSpd)
    {
        switch(direction) {
            case ROTATE_LEFT:
                driveMotors[0].setVelocity(x - rotSpd);
                driveMotors[1].setVelocity(y + rotSpd);
                driveMotors[2].setVelocity(x + rotSpd);
                driveMotors[3].setVelocity(y - rotSpd);
                break;
            case ROTATE_RIGHT:
                driveMotors[0].setVelocity(x + rotSpd);
                driveMotors[1].setVelocity(y - rotSpd);
                driveMotors[2].setVelocity(x - rotSpd);
                driveMotors[3].setVelocity(y + rotSpd);
                break;
        }
    }

    private void drive(double x, double y)
    {
        driveMotors[0].setVelocity(x);
        driveMotors[1].setVelocity(y);
        driveMotors[2].setVelocity(x);
        driveMotors[3].setVelocity(y);
    }

    public void stop_lift(){
        liftTest.setVelocity(0.0);
    }

    public void lift_motion(Lift_motion dir, double speed){
        switch(dir) {
            case UP:
                liftTest.setVelocity(speed);
                break;

            case DOWN:
                liftTest.setVelocity(-speed);
                break;
        }
    }

    public void set_lift_position(int position){
        liftTest.setTargetPosition(position);
    }

    private void init_dcmotor(String motorName){
        //DC Motors mapping
        for(int i = 0 ; i < 4 ; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, motorName + Integer.toString(i));
        }
        //Set motors to rotate at set speed
        for (int i=0; i<4;i++){
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveMotors[0].setDirection(DcMotor.Direction.FORWARD);
        driveMotors[1].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotor.Direction.FORWARD);

    }
    private void init_lifTest(String motorName){
        //initializing lift DC motor
        liftTest = hardwareMap.get(DcMotorEx.class ,motorName);
        liftTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftTest.setDirection((DcMotor.Direction.FORWARD));
    }

    private void init_arm_servos(String servoName, double init_pos_left, double init_pos_right){
        //initializing arm servo
        armServo[0] = hardwareMap.get(Servo.class ,servoName+"0");
        armServo[0].scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        armServo[0].setDirection(Servo.Direction.FORWARD);

        armServo[1] = hardwareMap.get(Servo.class ,servoName+"1");
        armServo[1].scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        armServo[1].setDirection(Servo.Direction.REVERSE);

        armServo[0].setPosition(init_pos_left);
        armServo[1].setPosition(init_pos_right);

    }

    private void init_distance_sensor(String sensorName){
        distSensor = hardwareMap.get(DistanceSensor.class, sensorName);
        telemetry.addData("Distance Sensor: ", "Distance Sensor initialize successfully.");
        telemetry.update();
    }

    private void init_acc_motors(String motorName){
        acc_motors[0] = hardwareMap.get(DcMotorEx.class, motorName + "0");
        acc_motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        acc_motors[0].setDirection(DcMotorSimple.Direction.FORWARD);

        acc_motors[1] = hardwareMap.get(DcMotorEx.class, motorName + "1");
        acc_motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        acc_motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
    }
    private void steer_Drive_Motors(){
        //driving the robot
        double speedX = gamepad1.left_stick_x * maxSpeed;
        double speedY = gamepad1.left_stick_y * maxSpeed;

        if(gamepad1.right_bumper) {
            drive(speedX, speedY, Drive_direction.ROTATE_RIGHT,rotationMaxSpeed);
        }else if(gamepad1.left_bumper){
            drive(speedX, speedY, Drive_direction.ROTATE_LEFT,rotationMaxSpeed);
        }else{
            drive(speedX, speedY);
        }
    }

    private void lift_Motor(){
        //controlling the lift
        boolean position=false;


        if(gamepad1.y){
            position=true; //true-up, false-down
        }else if(gamepad1.a){
            position=false;
        }

        if (position==true){
            set_lift_position(100);
        } else{
            set_lift_position(0);
        }
    }



    private void acc_motors(){
        double [] velocities = new double[2];

        if(gamepad1.x){
            acc_motors[0].setVelocity(accMaxSpeed);
            acc_motors[1].setVelocity(accMaxSpeed);
        }
        else if(gamepad1.b){
            acc_motors[0].setVelocity(-accMaxSpeed);
            acc_motors[1].setVelocity(-accMaxSpeed);
        }
        else if(gamepad1.dpad_left){
            acc_motors[0].setVelocity(-accMaxSpeed);
            acc_motors[1].setVelocity(accMaxSpeed);
        }
        else if(gamepad1.dpad_right){
            acc_motors[0].setVelocity(accMaxSpeed);
            acc_motors[1].setVelocity(-accMaxSpeed);
        }
        else{
            acc_motors[0].setVelocity(0);
            acc_motors[1].setVelocity(0);
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
        init_dcmotor("motorTest");
        init_lifTest("liftTest");
        init_arm_servos("armServo", 1.0, 1.0);
        init_acc_motors("accMotor");
        init_distance_sensor("distanceTest");
        telemetry.addData("Status", "initialized");
        telemetry.update();

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
                    lift_Motor();
            }
        });

        Thread acc_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true){
                    acc_motors();
                }
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

        drive_thread.start();
        lift_thread.start();
        acc_thread.start();
        distance_thread.start();

        drive_thread.join();
        lift_thread.join();
        acc_thread.join();
        distance_thread.join();

        waitForStart();
        while(true){

        }
    }
}