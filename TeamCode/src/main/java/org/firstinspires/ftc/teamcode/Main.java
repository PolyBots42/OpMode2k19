// @version 1.1
/*
 * @authors: Wojciech Boncela, Lukasz Gapiński, Mateusz Gapiński, Marceli Antosik, Jan Milczarek, Julia Sysdół, Witold Kardas
 *
 *
 * */

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
    double liftSpeed = 800.0;
    int position = 0;
    boolean flag = false;
    boolean flag2 = false;

    DcMotorEx[] driveMotors = new DcMotorEx[4];
    DcMotorEx liftMotor;
    Servo [] armServo = new Servo[2];
    DistanceSensor distSensor;
    private void drive(double x, double y, Drive_direction direction, double rotSpd){
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

    public void testSth(DcMotorEx motor, double vel){
        motor.setVelocity(vel);

    }

    public boolean isClicked(Gamepad gmpd){
        if (gmpd.y){
            if(!flag){
                flag = true;
                return true;
            } else{
                return false;
            }
        }else{
            flag = false;
            return false;
        }
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
    private void init_liftMotor(String motorName){
        //initializing lift DC motor
        liftMotor = hardwareMap.get(DcMotorEx.class ,motorName);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    private void steer_Drive_Motors(){
        //driving the robot
        double speedX;
        double speedY;

        if(gamepad1.start){
            speedX = gamepad1.left_stick_x * turboSpeed;
            speedY = gamepad1.left_stick_y * turboSpeed;
        }
        else {
            speedX = gamepad1.left_stick_x * maxSpeed;
            speedY = gamepad1.left_stick_y * maxSpeed;
        }

        if(gamepad1.right_bumper) {
            drive(speedX, speedY, Drive_direction.ROTATE_RIGHT,rotationMaxSpeed);
        }else if(gamepad1.left_bumper){
            drive(speedX, speedY, Drive_direction.ROTATE_LEFT,rotationMaxSpeed);
        }else{
            drive(speedX, speedY);
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

    private void armServos(){
        if(gamepad1.b){
            armServo[0].setPosition(0.0);
            armServo[1].setPosition(0.0);
        }
        else if(gamepad1.x){
            armServo[0].setPosition(1.0);
            armServo[1].setPosition(1.0);
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
        init_liftMotor("liftMotor");
        init_arm_servos("armServo", 1.0, 1.0);
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
                    armServos();
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
