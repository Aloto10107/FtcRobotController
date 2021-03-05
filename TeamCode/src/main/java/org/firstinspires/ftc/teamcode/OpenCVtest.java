package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;




/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

        import android.graphics.Bitmap;
        import android.graphics.Color;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.opencv.android.Utils;
        import org.opencv.core.Mat;
        import org.opencv.videoio.VideoCapture;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.openftc.easyopencv.OpenCvInternalCamera;
        import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="OpenCV", group="LinearOpMode")
public class OpenCVtest extends LinearOpMode {

    // Setup
    RobotMap robot = new RobotMap();


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        telemetry.addLine("reddhi speagatee");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        telemetry.addLine("part 1");
        telemetry.update();
        waitForStart();
    }} /*camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }
        });
        telemetry.addLine("part 1");
        telemetry.update();
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.startStreaming(320, 320);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        PipeLne pipeLine = new PipeLne();
        camera.setPipeline(pipeLine);

        Bitmap bitmap = Bitmap.createBitmap(pipeLine.Image.width(), pipeLine.Image.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(pipeLine.Image, bitmap);
        //Imgproc.cvtColor(imageArray, hsvImage, Imgproc.COLOR_BGR2HSV);
float lastResult = 0;

        int height;
        int width;
        int pixel;
        int oranges = 0;
        int bitmapWidth = pipeLine.Image.width();
        int bitmapHeight = pipeLine.Image.height();
        int colHeight = (int) ((double) bitmapHeight / 6.0);
        int height1StartCol = (int) ((double) bitmapHeight * (1.0 / 6.0) - ((double) colHeight / 2.0));
        //int colorCStartCol = (int) ((double) bitmapWidth * (3.0 / 6.0) - ((double) colWidth / 2.0));
        int height4StartCol = (int) ((double) bitmapHeight * (5.0 / 6.0) - ((double) colHeight / 2.0));

        for (width = 0; width < bitmapWidth; ++width) {
            for (height = 0; height < bitmapHeight; ++height){
                pixel = bitmap.getPixel(width, height);
                if (Color.red(pixel) > 245 && (Color.green(pixel) < 150 && Color.green(pixel) < 170) && (Color.blue(pixel) < 20)) {
                    oranges += 1;


                }
                if (oranges >= 100)
                {
                    lastResult = 3;
                }
                else if (oranges <=99 && oranges >= 40)
                {
                    lastResult = 1;
                }
                else
                {
                    lastResult = 0;
                }
            }
        }
telemetry.addData("Number", lastResult);





//       // I didn't know where to put this so ill leave it here
//
//        // ----------- Square Straight Ahead ----------- //
//        // Very rough estimate (not yet run)
//        waitForStart();
//        robot.encoderDrive(0.7,66);

        //

    }
    }
*/
