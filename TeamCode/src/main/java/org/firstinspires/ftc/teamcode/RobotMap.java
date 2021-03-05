package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Thread.sleep;

public class RobotMap {


    //IMPORTANTE: I commented out all mentions of wobbleArm to run some temp auto programs
    public RobotMap robot;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public Servo wobbleLatch = null;
    public DcMotor wobbleArm = null;
    public DcMotor launcher = null;
    public Servo ringPusher = null;
    public DcMotor ringIntake = null;
    public DcMotor flipper = null;
   ///* Copyright (c) 2017 FIRST. All rilic DcMotor leftFront = null;
   /* public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public Servo drop = null;
    //public DcMotor wobbleArm = null
    public DcMotor launcher = null;
    public Servo ringPusher = null;*/
    //public DcMotor ringIntake = null;
   ///* Copyright (c) 2017 FIRST. All rights reserved.
// *
/* PHONE CONFIG
expansion hub 4
    motors (NeveRest 40 Gearmotor)
        0 = right_front
        1 = right_back
        2 = left_front
        3 = left_back
     Servos
        5 = wobble_latch
    I2C bus 0
        imu 1
expansion hub 2
    motors (NeveRest 40 Gearmotor)
        0 = ring_intake
        1 = launcher
        (Rev robotics Core hex Motor)
        2 = flipper
        3 = wabble_arm
    Servo (Servo)
        4 = ring_Pusher
        5 = drop
    I2C bus 0
        imu 1
 */

 private Orientation angles;
    private Acceleration acceleration;
    public BNO055IMU imu;
    public float Gerror;
    public float deltaError;
    public float Derror;
    public float currentTime;
    public float currentError;
    public float preError;
    float deltaTime = 0;
    float preTime = 0;
    float PDout = 0;
    //1120 counts per rotation
    double COUNTS_PER_INCH = 29.86666666666666;
    double COUNTS_PER_INCH_LAUNCHER = 101.859163578;
    //Also potentially 133.69
    //double SCALE_FACTOR = 255;
    // 3:2 ratio for wheels?
    //1120 cpi for neverest 40
    //560 cpi for neverest 20
    // 0.218 in. for shaft
    // 0.68486719848 in. circumference for shaft
    // 12.5663706144 in. circumference for wheel
    // 0.68486719848 : 12.5663706144 ratio for shaft to wheel

    // 12.5 * 3/2 = 18.84
    // .68 in. : 18.8495559216 in.
    // 1120 counts per 18.84 in. (neverest40)
    // 1120 / 18.84 = 59.4178454211

    //12.5 * 3/2 = 18.75
    // .68in : 18.75 in.
    //560 / 18.75 in. = 29.86666666666666...

    // FOR LAUNCHER
    // 3.5 in. for flywheel
    // 10.9955742876 in. circumference
    // 10.9955742876 : 0.68486719848
    //
    // 1120 / 10.9955742876 = 101.859163578
    /* SHooting angles
    RIGHT = -175
    Center = -168
    Left = -162.5
     */


    //HardwareMap hwMap =  null;





    /* Constructor */
    public RobotMap() {

    }

    public void init(HardwareMap ahwmap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = ahwmap.get(DcMotor.class, "left_front");
        rightFront = ahwmap.get(DcMotor.class, "right_front");
        leftBack = ahwmap.get(DcMotor.class, "left_back");
        rightBack = ahwmap.get(DcMotor.class, "right_back");
        wobbleLatch =  ahwmap.get(Servo.class, "wobble_latch");
        wobbleArm = ahwmap.get(DcMotor.class, "wabble_arm");
        ringIntake = ahwmap.get(DcMotor.class, "ring_intake");
        imu = ahwmap.get(BNO055IMU.class, "imu");
        ringPusher = ahwmap.get(Servo.class, "ring_Pusher");
        launcher = ahwmap.get(DcMotor.class, "launcher");
        flipper = ahwmap.get(DcMotor.class, "flipper");




        //smallEyes = ahwmap.get(DistanceSensor.class, "smallEyes");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        wobbleArm.setDirection(DcMotor.Direction.REVERSE);
        ringPusher.setDirection(Servo.Direction.FORWARD);
        ringIntake.setDirection(DcMotor.Direction.REVERSE);
        wobbleLatch.setDirection(Servo.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        flipper.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ringIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

*/


    }
    public void resetEncoders(){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public synchronized void drive(double power, long time) throws InterruptedException {
        setMotor_br(power);
        setMotor_bl(power);
        setMotor_fl(power);
        setMotor_fr(power);
        sleep(time);
        setMotor_br(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_fr(0);
        sleep(500);
    }

    public synchronized void turn(double power, long time) throws InterruptedException {
        setMotor_bl(-power);
        setMotor_fl(-power);
        setMotor_br(power);
        setMotor_fr(power);
        sleep(time);
        setMotor_bl(0);
        setMotor_fl(0);        //ringPusher.setDirection(Servo.Direction.FORWARD);
        setMotor_br(0);
        setMotor_fr(0);
    }

    public synchronized void strafeRightForwardTime(double power, long time) throws InterruptedException {
        setMotor_fr(-power+.2);
        setMotor_bl(-power+.2);
        setMotor_fl(power+.2);
        setMotor_br(power+.2);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
    }
    public synchronized void strafeLeftForwardTime(double power, long time) throws InterruptedException {
        setMotor_fr(power + .2);
        setMotor_bl(power + .2);
        setMotor_fl(-power + .2);
        setMotor_br(-power + .2);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
    }
    public synchronized void strafeLeftTime(double power, long time) throws InterruptedException {
        setMotor_fr(power);
        setMotor_bl(power);
        setMotor_fl(-power);
        setMotor_br(-power);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
        sleep(250);

    }
    public synchronized void strafeRightTime(double power, long time) throws InterruptedException {
        setMotor_fr(-power);
        setMotor_bl(-power + .2);
        setMotor_fl(power);
        setMotor_br(power - .2);
        sleep(time);
        setMotor_fr(0);
        setMotor_bl(0);
        setMotor_fl(0);
        setMotor_br(0);
        sleep(250);
    }

    public synchronized void strafeLeft() throws InterruptedException {
        setMotor_fr(1);
        setMotor_bl(1);
        setMotor_fl(-1);
        setMotor_br(-1);


    }
    public synchronized void strafeRight() throws InterruptedException {
        setMotor_fr(-1);
        setMotor_bl(-1);
        setMotor_fl(1);
        setMotor_br(1);

    }

    public synchronized void stop() {
        setMotor_br(0);
        setMotor_fl(0);
        setMotor_bl(0);
        setMotor_fr(0);
    }
    public synchronized void leftArc(double power, long time) throws InterruptedException {
        setMotor_fl(power);
        setMotor_bl(power);
        sleep(time);
        setMotor_fl(0);
        setMotor_bl(0);
        sleep(500);
    }
    public synchronized void rightArc(double power, long time) throws InterruptedException {
        setMotor_fr(power);
        setMotor_br(power);
        sleep(time);
        setMotor_fl(0);
        setMotor_bl(0);
        sleep(500);


    }
    public synchronized void flyWheel(){
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPower(.644);
    }
    public synchronized void flyWheelSlow(){
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPower(594);
    }
    public synchronized void wobbleArm(double power, long time) throws InterruptedException {
        wobbleArm.setPower(power);
        sleep(time);
        wobbleArm.setPower(0);
        sleep(500);
    }


    public synchronized int getPos(){
        return rightFront.getCurrentPosition();

    }




    public synchronized void setMotor_fl(double power) {
        double convertedPower = (power);

        leftFront.setPower(convertedPower);
    }

    public synchronized void setMotor_bl(double power) {
        double convertedPower = (power);

        leftBack.setPower(convertedPower);
    }

    public synchronized void setMotor_fr(double power) {
        double convertedPower = (power);

        rightFront.setPower(convertedPower);
    }

    public synchronized void setMotor_br(double power) {
        double convertedPower = (power);

        rightBack.setPower(convertedPower);
    }
    /*public boolean Open(){
        boolean Open;
        if (robot.smallEyes.getDistance(DistanceUnit.CM) < .1) {
            Open = true;
        }
        else {
            Open = false;
        }
        return Open;
    }*/

/*    public float[] hsv(){
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (skyStone.red() * SCALE_FACTOR),
                (int) (skyStone.green() * SCALE_FACTOR),
                (int) (skyStone.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues;
    }*/
   /* public float linered(){
        return line.red();
    }
    public float lineblue         (){
        return line.blue();
    }
*/
    public void mechanumDrive(float forward, float strafe, float rotation){
        if(Math.abs(forward)< .2){
            forward = 0;
        }

        if(Math.abs(strafe)< .2){
            strafe = 0;
        }

        if(Math.abs(rotation)< .2){
            rotation = 0;
        }
        leftBack.setPower((4 *(forward + strafe - rotation))/5);
        leftFront.setPower((4 * (forward - strafe - rotation))/5);
        rightBack.setPower((4 * (forward - strafe + rotation))/5);
        rightFront.setPower((4 * (forward + strafe + rotation))/5);
    }
    public void Foward(float fowards){
        if(Math.abs(fowards) < .2){
            fowards = 0;
        }

        leftBack.setPower(fowards);
        rightBack.setPower(fowards);
        leftFront.setPower(fowards);
        rightFront.setPower(fowards);

    }

    public void Turn(float turns){
        if(Math.abs(turns) < .2) {
            turns = 0;
        }

            leftBack.setPower(turns);
            rightBack.setPower(-turns);
            leftFront.setPower(turns);
            rightFront.setPower(-turns);

    }

    public void imuINIT() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;//new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public float getHeading() {
        //PLZ dont touch *touch*
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double[] getAcceleration() {

        acceleration = imu.getAcceleration();
        double[] accel = new double[]{acceleration.xAccel, acceleration.yAccel, acceleration.zAccel};

        return accel;
    }
    public synchronized void flyWHeel(double power) throws InterruptedException{
        ringPusher.setPosition(1);
        sleep(800);
       launcher.setPower(power);
        sleep(1500);
        launcher.setPower(power);
        ringPusher.setPosition(-.1);
        sleep(600);
        launcher.setPower(0);
    }
    public synchronized void flyWHeel2() throws InterruptedException{
        ringPusher.setPosition(1);
        sleep(800);
        flySpeed(25);
        ringPusher.setPosition(-.1);
        sleep(300);
        launcher.setPower(0);
    }
    double currentSpeed;
    public void flySpeed(float targetSpeed )throws InterruptedException{

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        float Kp = (float) 0.00007;
        float Kd = (float) 0.0001;

        while (true) {
            deltaTime = System.nanoTime() - preTime;
            currentSpeed = ((launcher.getCurrentPosition() / 101.859163578))/deltaTime;
            sleep(1000);
            break;
           /* Gerror = (float) (targetSpeed - currentSpeed);
            PDout = (float) (.1 + (Kp * Gerror)) + (Kd * (Gerror / deltaTime));
            //PDout = Kp * Gerror;

            launcher.setPower(PDout);
            //preTime = currentTime;
            if (Math.abs(Gerror) <= 1) {
                launcher.setPower(0);
                break;
            }

            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        }
    }

    public void gyroturn(float degrees) throws InterruptedException {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        float Kp = (float) 0.00322;
        float Kd = (float) 0.001;

        while (true) {
            deltaTime = System.nanoTime() - preTime;
            Gerror = getHeading() - degrees;
            PDout = (Kp * Gerror) + (Kd * (Gerror / deltaTime));
            //PDout = Kp * Gerror;

                leftBack.setPower(PDout);
                leftFront.setPower(PDout);
                rightBack.setPower(-PDout);
                rightFront.setPower(-PDout);
                //preTime = currentTime;
             if (Math.abs(Gerror) <= 5) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                break;
            }

            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(500);
        }
    }

        public void gyroStrafe ( double power, double target, long time) throws InterruptedException
        {

            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



            float Kp = (float) 0.0005;
            float Kd = (float) 0.0001;

            double out = 0;

            ElapsedTime strafeTime = new ElapsedTime();

            while (true) {
                out = Kp * (getHeading() - target);
                leftBack.setPower(-power + out);
                leftFront.setPower((power) - out);
                rightBack.setPower((power + .3) - out);
                rightFront.setPower((-power - .3) + out);


                //preTime = currentTime;
                if (strafeTime.milliseconds() > time) {
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    leftFront.setPower(0);
                    break;
                }
            }
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(500);
        }
    public void gyroStrafeTeleOp ( double power, double target) {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        float Kp = (float) 0.0024;
        float Kd = (float) 0.0001;

        double out = 0;

        ElapsedTime strafeTime = new ElapsedTime();

        while (true) {
            out = Kp * (getHeading() - target);
            leftBack.setPower(-power + out);
            leftFront.setPower((power) - out);
            rightBack.setPower((power + .3) - out);
            rightFront.setPower((-power - .3) + out);
            break;
        }
    }
    public void gyroFlywheel ( double speed, boolean bottun) {




        ElapsedTime speedTime = new ElapsedTime();
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int seconds = 0;
        while(bottun == true){
        if (speedTime.milliseconds() > (1000 * seconds)){
            seconds += 1;
        }

      int newTargetdistance = (int)(speed * COUNTS_PER_INCH_LAUNCHER);
        float cSpeed = launcher.getCurrentPosition()/seconds;
        float desiredSpeed =  newTargetdistance / seconds;
        double errorSpeed = ((desiredSpeed - cSpeed)/ COUNTS_PER_INCH_LAUNCHER);
        double kp = .03;

        launcher.setTargetPosition(newTargetdistance / seconds );
        launcher.setPower(kp*errorSpeed);


      }
      launcher.setPower(0);


    }


            //1120 counts per rotation
    double flyPower;
    double currentPower = 0;

    public void gyroStrafeLeft(double power, double target, long time) throws InterruptedException {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        float Kp = (float) 0.007;
        float Kd = (float) 0.0001;

        double out = 0;

        ElapsedTime strafeTime = new ElapsedTime();

        while (true) {
            out = Kp * (getHeading()- target);
            leftBack.setPower(power - out);
            leftFront.setPower(-power + out);
            rightBack.setPower(-power + out);
            rightFront.setPower(power - out);


            //preTime = currentTime;
            if (strafeTime.milliseconds() > time) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                break;
            }
        }
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
    //1120 counts per rotation
   /* double flyPower;
    double currentPower = 0;*/

/*public void flySPeed(float targetSpeed, double currentSpeed){

    if (targetSpeed < currentSpeed && (targetSpeed - currentSpeed) < 10 && launcher.getPower() != 1){
        flyPower = currentPower + .1;
    } else if (targetSpeed < currentSpeed && launcher.getPower() != 0){
            flyPower = currentPower - .01;
    }else if (targetSpeed > currentSpeed && launcher.getPower() != 1){
            flyPower = currentPower + .01;
    }else {
            flyPower = currentPower;
        }
    currentPower = flyPower;
       launcher.setPower(flyPower);
    }*/
    /*    public double launcherSpeed;
    public double encoderError;
    public double launcherMath;
    public double s;
    public void launcherSpeed(double speed, float times,float fuck){

        float Kp = (float) 0.02;
        float Ki = (float) 1 ;
        float Kd = (float) 0.0001;
        deltaTime = times - preTime;
        launcherSpeed = ((fuck / COUNTS_PER_INCH_LAUNCHER)) / times;
        encoderError = launcherSpeed /speed;
        s = (encoderError / deltaTime);
        launcherMath = 1.8 - encoderError;
        //launcherMath = (Kp * encoderError) + (Ki/s) + (Kd * s);
        launcher.setPower(launcherMath);
    }*/
    public void gyroDrive(double power, double target, long time) throws InterruptedException {

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //float Kp = (float) 0.015;
        float Kp = (float) 0.03;
        float Kd = (float) 0.0001;

        double out = 0;

        ElapsedTime driveTime = new ElapsedTime();

        while (true) {
            out = Kp * (getHeading() - target);
            leftBack.setPower(power + out);
            leftFront.setPower(power + out);
            rightBack.setPower(power - out);
            rightFront.setPower(power - out);
            //preTime = currentTime;
            if (driveTime.milliseconds() > time) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);
                break;
            }
        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }


    public void encoderDrive(double speed, double inches) throws InterruptedException {


        int newLeftBackTarget;


        int newrightBackTarget;


        int newLeftFrontTarget;


        int newRightFrontTarget;


        boolean rightAhead = false;


        boolean leftAhead = false;




        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);


        newrightBackTarget = (int) (inches * COUNTS_PER_INCH);


        newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);


        newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);


        leftBack.setTargetPosition(newLeftBackTarget);


        rightBack.setTargetPosition(newrightBackTarget);


        leftFront.setTargetPosition(newLeftFrontTarget);


        rightFront.setTargetPosition(newRightFrontTarget);


        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftBack.setPower(speed *1.5);


        rightBack.setPower(speed*1.5);


        leftFront.setPower(speed*1.5);


        rightFront.setPower(speed*1.5);

        while (rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()){

            int x = 0;
            x ++;
        }

        leftBack.setPower(0);


        rightBack.setPower(0);


        leftFront.setPower(0);


        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

        }
    public void encoderGyroDrive(double speed, double target,


                             double inches) throws InterruptedException {


        int newLeftBackTarget;


        int newrightBackTarget;


        int newLeftFrontTarget;


        int newRightFrontTarget;


        boolean rightAhead = false;


        boolean leftAhead = false;

        double out = 0;

        float Kp = (float) 0.03;


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);


        newrightBackTarget = (int) (inches * COUNTS_PER_INCH);


        newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);


        newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);


        leftBack.setTargetPosition(newLeftBackTarget);


        rightBack.setTargetPosition(newrightBackTarget);


        leftFront.setTargetPosition(newLeftFrontTarget);


        rightFront.setTargetPosition(newRightFrontTarget);


        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftBack.setPower(speed);


        rightBack.setPower(speed);


        leftFront.setPower(speed);


        rightFront.setPower(speed);

        while (rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()){


            out = Kp * (getHeading()-target);
            leftBack.setPower(speed + out);
            leftFront.setPower(speed + out);
            rightBack.setPower(speed - out);
            rightFront.setPower(speed - out);

        }

        leftBack.setPower(0);


        rightBack.setPower(0);


        leftFront.setPower(0);


        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

    }
    public void encoderGyroStrafe(double speed, double target, double inches, boolean leftdirection) throws InterruptedException {


        int newLeftBackTarget;


        int newrightBackTarget;


        int newLeftFrontTarget;


        int newRightFrontTarget;


        boolean rightAhead = false;


        boolean leftAhead = false;

        double out = 0;

        float Kp = (float) 0.03;


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newLeftBackTarget = (int) (inches * COUNTS_PER_INCH);


        newrightBackTarget = (int) (inches * COUNTS_PER_INCH);


        newLeftFrontTarget = (int) (inches * COUNTS_PER_INCH);


        newRightFrontTarget = (int) (inches * COUNTS_PER_INCH);


        leftBack.setTargetPosition(newLeftBackTarget);


        rightBack.setTargetPosition(newrightBackTarget);


        leftFront.setTargetPosition(newLeftFrontTarget);


        rightFront.setTargetPosition(newRightFrontTarget);


        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

if(leftdirection == true){
        leftBack.setPower(speed);


        rightBack.setPower(-speed);


        leftFront.setPower(-speed);


        rightFront.setPower(speed);
}else{
    leftBack.setPower(-speed);


    rightBack.setPower(speed);


    leftFront.setPower(speed);


    rightFront.setPower(-speed);
}


        while (rightFront.isBusy() && leftFront.isBusy() && rightBack.isBusy() && leftBack.isBusy()){


            out = Kp * (getHeading()-target);
            leftBack.setPower(speed + out);
            leftFront.setPower(speed + out);
            rightBack.setPower(speed - out);
            rightFront.setPower(speed - out);

        }

        leftBack.setPower(0);


        rightBack.setPower(0);


        leftFront.setPower(0);


        rightFront.setPower(0);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

    }
    }

