#include <cstdio>
#include <iostream>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "camera_calibration/Settings.hpp"
#include "opencv2/objdetect/charuco_detector.hpp"

using namespace std::chrono_literals;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

class CalibrationNode : public rclcpp::Node
{
public:
    CalibrationNode()
    : Node("camera_calibration"), 
    release_object(false),
    RED(0,0,255), GREEN(0,255,0)
    {
        this->declare_parameter("inputSettingsFile", "1");
        this->declare_parameter("winSize", 5);

        inputSettingsFile = this->get_parameter("inputSettingsFile").as_string();
        winSize = this->get_parameter("winSize").as_int();
        FileStorage fs(inputSettingsFile, FileStorage::READ);
        if (!fs.isOpened())
        {
            cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
            return;
        }
        s.read(fs["Settings"]);
        fs.release();                                         // close Settings file
        //! [file_read]

        if (!s.goodInput)
        {
            cout << "Invalid input detected. Application stopping. " << endl;
            return;
        }

        grid_width = s.squareSize * (s.boardSize.width - 1);
        if (s.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
            grid_width = s.squareSize * (s.boardSize.width - 2);
        }

        //create CharucoBoard
        if (s.calibrationPattern == Settings::CHARUCOBOARD) {
            if (s.arucoDictFileName == "") {
                cv::aruco::PredefinedDictionaryType arucoDict;
                if (s.arucoDictName == "DICT_4X4_50") { arucoDict = cv::aruco::DICT_4X4_50; }
                else if (s.arucoDictName == "DICT_4X4_100") { arucoDict = cv::aruco::DICT_4X4_100; }
                else if (s.arucoDictName == "DICT_4X4_250") { arucoDict = cv::aruco::DICT_4X4_250; }
                else if (s.arucoDictName == "DICT_4X4_1000") { arucoDict = cv::aruco::DICT_4X4_1000; }
                else if (s.arucoDictName == "DICT_5X5_50") { arucoDict = cv::aruco::DICT_5X5_50; }
                else if (s.arucoDictName == "DICT_5X5_100") { arucoDict = cv::aruco::DICT_5X5_100; }
                else if (s.arucoDictName == "DICT_5X5_250") { arucoDict = cv::aruco::DICT_5X5_250; }
                else if (s.arucoDictName == "DICT_5X5_1000") { arucoDict = cv::aruco::DICT_5X5_1000; }
                else if (s.arucoDictName == "DICT_6X6_50") { arucoDict = cv::aruco::DICT_6X6_50; }
                else if (s.arucoDictName == "DICT_6X6_100") { arucoDict = cv::aruco::DICT_6X6_100; }
                else if (s.arucoDictName == "DICT_6X6_250") { arucoDict = cv::aruco::DICT_6X6_250; }
                else if (s.arucoDictName == "DICT_6X6_1000") { arucoDict = cv::aruco::DICT_6X6_1000; }
                else if (s.arucoDictName == "DICT_7X7_50") { arucoDict = cv::aruco::DICT_7X7_50; }
                else if (s.arucoDictName == "DICT_7X7_100") { arucoDict = cv::aruco::DICT_7X7_100; }
                else if (s.arucoDictName == "DICT_7X7_250") { arucoDict = cv::aruco::DICT_7X7_250; }
                else if (s.arucoDictName == "DICT_7X7_1000") { arucoDict = cv::aruco::DICT_7X7_1000; }
                else if (s.arucoDictName == "DICT_ARUCO_ORIGINAL") { arucoDict = cv::aruco::DICT_ARUCO_ORIGINAL; }
                else if (s.arucoDictName == "DICT_APRILTAG_16h5") { arucoDict = cv::aruco::DICT_APRILTAG_16h5; }
                else if (s.arucoDictName == "DICT_APRILTAG_25h9") { arucoDict = cv::aruco::DICT_APRILTAG_25h9; }
                else if (s.arucoDictName == "DICT_APRILTAG_36h10") { arucoDict = cv::aruco::DICT_APRILTAG_36h10; }
                else if (s.arucoDictName == "DICT_APRILTAG_36h11") { arucoDict = cv::aruco::DICT_APRILTAG_36h11; }
                else {
                    cout << "incorrect name of aruco dictionary \n";
                    return;
                }

                pdictionary.reset(new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(arucoDict)));
            }
            else {
                cv::FileStorage dict_file(s.arucoDictFileName, cv::FileStorage::Mode::READ);
                cv::FileNode fn(dict_file.root());
                pdictionary->readDictionary(fn);
            }
        }
        else {
            // default dictionary
            pdictionary.reset(new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(0)));
        }

        pch_board.reset(new cv::aruco::CharucoBoard({s.boardSize.width, s.boardSize.height}, s.squareSize, s.markerSize, *pdictionary));
        pch_detector.reset(new cv::aruco::CharucoDetector(*pch_board));

        mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;

        timer_ = this->create_wall_timer(50ms, 
            std::bind(&CalibrationNode::timer_callback, this));
    }

    void timer_callback();

private:
    bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object);

    void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/);

    bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr, vector<Point3f>& newObjPoints,
                            float grid_width, bool release_object);

    void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, const vector<Point3f>& newObjPoints );

    double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye);

    

    rclcpp::TimerBase::SharedPtr timer_;
    Settings s;
    string inputSettingsFile;
    int winSize;
    float grid_width;
    bool release_object;

    std::shared_ptr<cv::aruco::Dictionary> pdictionary;
    std::shared_ptr<cv::aruco::CharucoBoard> pch_board;
    std::shared_ptr<cv::aruco::CharucoDetector> pch_detector;


    std::vector<int> markerIds;

    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode;

    clock_t prevTimestamp = 0;
    const Scalar RED;
    const Scalar GREEN;
};

bool CalibrationNode::runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object) {
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr, newObjPoints, grid_width, release_object);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr, newObjPoints);
    return ok;
}

//! [compute_errors]
//! [board_corners]
void CalibrationNode::calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
            }
        }
        break;
    case Settings::CHARUCOBOARD:
        for (int i = 0; i < boardSize.height - 1; ++i) {
            for (int j = 0; j < boardSize.width - 1; ++j) {
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
            }
        }
        break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++) {
            for (int j = 0; j < boardSize.width; j++) {
                corners.push_back(Point3f((2 * j + i % 2)*squareSize, i*squareSize, 0));
            }
        }
        break;
    default:
        break;
    }
}

//! [board_corners]
bool CalibrationNode::runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr, vector<Point3f>& newObjPoints,
                            float grid_width, bool release_object)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( !s.useFisheye && s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    if (s.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
        objectPoints[0][s.boardSize.width - 2].x = objectPoints[0][0].x + grid_width;
    }
    else {
        objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    }
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        int iFixedPoint = -1;
        if (release_object)
            iFixedPoint = s.boardSize.width - 1;
        rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                                cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                                s.flag | CALIB_USE_LU);
    }

    if (release_object) {
        cout << "New board corners: " << endl;
        cout << newObjPoints[0] << endl;
        cout << newObjPoints[s.boardSize.width - 1] << endl;
        cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << endl;
        cout << newObjPoints.back() << endl;
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
void CalibrationNode::saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, const vector<Point3f>& newObjPoints )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;
    fs << "marker_size" << s.markerSize;

    if( !s.useFisheye && s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs.writeComment("distortion_coefficients: k1, k2, p1, p2, k3");
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    if( s.writeGrid && !newObjPoints.empty() )
    {
        fs << "grid_points" << newObjPoints;
    }
}

//! [compute_errors]
double CalibrationNode::computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

void CalibrationNode::timer_callback()
{
    Mat view;
    bool blinkOutput = false;

    view = s.nextImage();

    //-----  If no more image, or got enough, then stop calibration and show result -------------
    if( mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames )
    {
        if(runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
                                release_object))
            mode = CALIBRATED;
        else
            mode = DETECTION;
    }
    if(view.empty())          // If there are no more images stop the loop
    {
        // if calibration threshold was not reached yet, calibrate now
        if( mode != CALIBRATED && !imagePoints.empty() )
            runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
                                    release_object);
        timer_->cancel();
        return;
    }
    //! [get_input]

    imageSize = view.size();  // Format input image.
    if( s.flipVertical )    flip( view, view, 0 );

    //! [find_pattern]
    vector<Point2f> pointBuf;

    bool found;

    int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

    if(!s.useFisheye) {
        // fast check erroneously fails with high distortions like fisheye
        chessBoardFlags |= CALIB_CB_FAST_CHECK;
    }

    switch( s.calibrationPattern ) // Find feature points on the input format
    {
    case Settings::CHESSBOARD:
        found = findChessboardCorners( view, s.boardSize, pointBuf, chessBoardFlags);
        break;
    case Settings::CHARUCOBOARD:
        pch_detector->detectBoard( view, pointBuf, markerIds);
        found = pointBuf.size() == (size_t)((s.boardSize.height - 1)*(s.boardSize.width - 1));
        break;
    case Settings::CIRCLES_GRID:
        found = findCirclesGrid( view, s.boardSize, pointBuf );
        break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
        break;
    default:
        found = false;
        break;
    }
    //! [find_pattern]

    //! [pattern_found]
    if (found)                // If done with success,
    {
            // improve the found corners' coordinate accuracy for chessboard
            if( s.calibrationPattern == Settings::CHESSBOARD)
            {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix( viewGray, pointBuf, Size(winSize,winSize),
                    Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
            }

            if( mode == CAPTURING &&  // For camera only take new samples after delay time
                (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
            {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                blinkOutput = s.inputCapture.isOpened();
            }

            // Draw the corners.
            if(s.calibrationPattern == Settings::CHARUCOBOARD)
                drawChessboardCorners( view, cv::Size(s.boardSize.width-1, s.boardSize.height-1), Mat(pointBuf), found );
            else
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
    }
    //! [pattern_found]
    //----------------------------- Output Text ------------------------------------------------
    //! [output_text]
    //   string msg = (mode == CAPTURING) ? "100/100" :
    //                 mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
    //   int baseLine = 0;
    //   Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
    //   Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

    //   if( mode == CAPTURING )
    //   {
    //       if(s.showUndistorted)
    //           msg = cv::format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
    //       else
    //           msg = cv::format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
    //   }

    //   putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

    //   if( blinkOutput )
    //       bitwise_not(view, view);
    //   //! [output_text]
    //   //------------------------- Video capture  output  undistorted ------------------------------
    //   //! [output_undistorted]
    //   if( mode == CALIBRATED && s.showUndistorted )
    //   {
    //       Mat temp = view.clone();
    //       if (s.useFisheye)
    //       {
    //           Mat newCamMat;
    //           fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
    //                                                               Matx33d::eye(), newCamMat, 1);
    //           cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs, newCamMat);
    //       }
    //       else
    //         undistort(temp, view, cameraMatrix, distCoeffs);
    //   }
    //! [output_undistorted]
    //------------------------------ Show image and check for input commands -------------------
    //! [await_input]
    cv::imshow("Image View", view);
    char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

    if( key  == 27 ) {
        timer_->cancel();
        return;
    }

    //   if( key == 'u' && mode == CALIBRATED )
    //       s.showUndistorted = !s.showUndistorted;

    if( s.inputCapture.isOpened() && key == 'g' )
    {
        mode = CAPTURING;
        imagePoints.clear();
    }
    //! [await_input]
}
