package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test", group="Iterative Opmode")
public class Main extends LinearOpMode {

    double maxSpeed = 1000.0;
    double rotationMaxSpeed = 500.0;

    DcMotorEx[] driveMotors = new DcMotorEx[4];
    DcMotorEx liftTest;
    Servo [] armServo = new Servo[2];

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
        liftTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTest.setDirection((DcMotor.Direction.FORWARD));
    }

    private void init_arm_servos(String servoName){
        //initializing arm servo
        armServo[0] = hardwareMap.get(Servo.class ,servoName+"0");
        armServo[0].scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        armServo[0].setDirection(Servo.Direction.FORWARD);

        armServo[1] = hardwareMap.get(Servo.class ,servoName+"1");
        armServo[1].scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        armServo[1].setDirection(Servo.Direction.REVERSE);
        // TODO
    }
    private void steer_Drive_Motors(){
        //driving the robot
        double speedX;
        double speedY;

        speedX = gamepad1.left_stick_x * maxSpeed;
        speedY = gamepad1.left_stick_y * maxSpeed;

        if(gamepad1.right_bumper) {
            driveMotors[0].setVelocity(speedX + rotationMaxSpeed);
            driveMotors[1].setVelocity(speedY - rotationMaxSpeed);
            driveMotors[2].setVelocity(speedX - rotationMaxSpeed);
            driveMotors[3].setVelocity(speedY + rotationMaxSpeed);
        }else if(gamepad1.left_bumper){
            driveMotors[0].setVelocity(speedX - rotationMaxSpeed);
            driveMotors[1].setVelocity(speedY + rotationMaxSpeed);
            driveMotors[2].setVelocity(speedX + rotationMaxSpeed);
            driveMotors[3].setVelocity(speedY - rotationMaxSpeed);
        }else{
            driveMotors[0].setVelocity(speedX);
            driveMotors[1].setVelocity(speedY);
            driveMotors[2].setVelocity(speedX);
            driveMotors[3].setVelocity(speedY);
        }
    }

    private void lift_Motor(){
        //controlling the lift
        double liftSpeed = 200.0;

        if(gamepad1.y){
            liftTest.setVelocity(liftSpeed);
        }else if(gamepad1.a){
            liftTest.setVelocity(-liftSpeed);
        }else{
            liftTest.setVelocity(0);
        }

    }

    private void arm_servos(){
        double servoPosition1=armServo[0].getPosition();
        double servoPosition2=armServo[0].getPosition();

        if(gamepad1.x){
            while(gamepad1.x) {
                servoPosition1+=0.05;
                servoPosition2-=0.05;
                armServo[0].setPosition(servoPosition2);
                armServo[0].setPosition(servoPosition2);
            }
        }else if(gamepad1.b){
            while(gamepad1.b) {
                servoPosition1-=0.05;
                servoPosition2+=0.05;
                armServo[0].setPosition(servoPosition2);
                armServo[0].setPosition(servoPosition2);
            }
        }else{
            armServo[0].setPosition(0.0);
        }
        //TODO
    }

    //main function:
    @Override
    public void runOpMode() throws InterruptedException {
        init_dcmotor("motorTest");
        init_lifTest("liftTest");
        init_arm_servos("armServo");
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

        Thread arm_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    arm_servos();
            }
        });


        drive_thread.start();
        lift_thread.start();
        arm_thread.start();

        drive_thread.join();
        lift_thread.join();
        arm_thread.join();

        waitForStart();
        while(true){

        }
    }
}