// #pragma once

// #include "utils/LimeLightHelper.hpp"

// #include <cameraserver/CameraServer.h>
// #include <opencv2/imgproc/imgproc.hpp>

// class Vision
// {
//     public:
//         Vision(Vision::camera_type cam);

//         // Pos
//         frc::Pose2d GetVisionPosition();

//         // Camera
//         void StartCams();

//         // You'll want 1 Pos and 1 Sight camera
//         // Pos does Pos estimation among other things
//         // Sight gives an output to NT
//         enum camera_type
//         {
//             // Pos
//             PHOTON,
//             LIMELIGHT,

//             // Sight
//             GENERIC_USB
//         };
//     private:
//         static void VisionOnAnotherThread()
//         {
//             cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture(); // gets camera

//             camera.SetResolution(640, 480); // sets resolution

//             // Get a CvSink. This will capture Mats from the Camera
//             cs::CvSink cvSink = frc::CameraServer::GetVideo();

//             // Setup a CvSource. This will send images back to the Dashboard
//             cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 640, 480);

//             cv::Mat mat;
//             while (true) 
//             {
//                 if (cvSink.GrabFrame(mat) == 0) 
//                 {
//                     outputStream.NotifyError(cvSink.GetError());
//                     continue;
//                 }
//                 // Put a rectangle on the image
//                 rectangle(mat, cv::Point(100, 100), cv::Point(400, 400), cv::Scalar(255, 255, 255), 5);
//                 // An idea for this is to have all of our basic telemetry data on the camera output, not suggested though

//                 // Give the output stream a new image to display
//                 outputStream.PutFrame(mat);
//             }
//         };

//         enum pos_type {P, L};
// }