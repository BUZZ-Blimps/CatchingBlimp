// ============================== INCLUDES ==============================
#include <libv4l2.h>
#include <iostream>
#include "ComputerVision.h"
#include <vector>
#include <string>

using namespace std;

string openCVType2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

// ============================== CLASS ==============================
void ComputerVision::init()
{
  // Init Camera  
  if(USE_VIDEO){
    // Open video
    cap.open("/home/corelab-laptop2/GitHub/CatchingBlimp/CrazyBurple.avi");
    if (!cap.isOpened()) {
      CV_Assert("CamL open failed");
    }
  }else{
    // Open camera
    cap.open(CAMERA_INDEX, CAP_V4L);
    if (!cap.isOpened()) {
      CV_Assert("CamL open failed");
    }
  }

  cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

  // Init stereo matcher
  left_matcher = StereoBM::create(16, 13); //Num disp, block size
	left_matcher->setPreFilterType(1);
	left_matcher->setPreFilterSize(PRE_FILTER_SIZE);
	left_matcher->setPreFilterCap(PRE_FILTER_CAP);
	left_matcher->setUniquenessRatio(UNIQUENESS_RATIO);

	wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
	right_matcher = ximgproc::createRightMatcher(left_matcher);

	wls_filter->setLambda(LAMBDA);
	wls_filter->setSigmaColor(SIGMA);

  setQ();
}

void ComputerVision::setQ()
{
  Mat Q_ = cv::Mat::zeros(4, 4, CV_64F);
  Q_.at<double>(0, 0) = 1;
  Q_.at<double>(1, 1) = 1;
  Q_.at<double>(2, 2) = 0;
  Q_.at<double>(3, 3) = 1;
  Q_.at<double>(0, 3) = -(CAMERA_WIDTH/2);
  Q_.at<double>(1, 3) = -(CAMERA_HEIGHT/2);
  Q_.at<double>(2, 3) = F;
  Q_.at<double>(3, 2) = -1.0/BASELINE;

  this->Q = Q_;
}

void ComputerVision::readCalibrationFiles(string srcDir)
{
  cout << "Reading Stereo Camera Parameters" << endl;
	
  string stereo_cal_path = srcDir + "/" + STEREO_CAL_FILENAME;

  FileStorage cv_file2 = FileStorage(stereo_cal_path, FileStorage::READ);
    if(!cv_file2.isOpened()){
        cout << "Read failed!" << endl;
        return;
    }
	
  cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
	cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
	cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
	cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
  //cv_file2["Q"] >> Q;


	cv_file2.release();
	cout << "Read Complete" << endl;
}

void ComputerVision::update(autoState mode, goalType goalColor)
{
  // Get Frames
  Mat left, right;
  getFrames(left, right);

  // Coord Variables
  float Ballx, Bally, Ballz = 0;
  float Goalx, Goaly, Goalz = 0;
  float ballArea = 0;
  float goalArea = 0;
  float goalAngle = 0;

  // Object Avoidance
  if (mode == searching || mode == goalSearch) {
    // Maybe Fix
    quad = 10;    
  }

  // Ball Detection
  mode = goalSearch;
  if (mode == searching || mode == approach || mode == catching) {
    getBall(Ballx, Bally, Ballz, ballArea, left, right);

    std::vector<float> balloon;
    balloon.push_back(Ballx);
    balloon.push_back(Bally);
    balloon.push_back(Ballz);
    balloon.push_back(ballArea);
    balloons.push_back(balloon);

  // Goal Detection
  } else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
    getGoal(Goalx, Goaly, Goalz, goalArea, goalAngle, left, right);

    std::vector<float> goal;
    goal.push_back(Goalx);
    goal.push_back(Goaly);
    goal.push_back(Goalz);
    goal.push_back(goalArea);
    //goal.push_back(goalAngle);
    goals.push_back(goal);

  }
}

void ComputerVision::getFrames(Mat &imgL, Mat &imgR)
{
  // 32 = space
  // 27 = esc

  // Get Frame
  if(USE_VIDEO){
    int delay = 10;
    if(capturing) delay = 1000.0 * 1/VIDEO_FPS;
    
    int key = waitKey(delay);
    if(key == 32){
      // Toggle video
      capturing = !capturing;
      if(capturing){
        cout << "Resumed video." << endl;
      }else{
        cout << "Paused video." << endl;
      }
    }else if(key == 27){
      //Kill program
    }
  }

  Mat frame2;
  if(capturing){
    if(cap.grab()){
      cap.retrieve(frame2);
    }else{
      cout << "VideoCapture not open. Reopening..." << endl;
      cap.release();
      init();
      cap >> frame2;
    }
    lastCaptured = frame2;
  }else{
    frame2 = lastCaptured;
  }

  // Split Frames
  Rect left_roi(0, 0, frame2.cols/2, frame2.rows);
  Rect right_roi (frame2.cols/2, 0, frame2.cols/2, frame2.rows);

  Mat crop_left(frame2, left_roi);
  Mat crop_right (frame2, right_roi);

  crop_left.copyTo(imgL);
  crop_right.copyTo(imgR);
}

bool ComputerVision::getBall(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR)
{
  // Initialize matrix for rectified stereo images
  Mat Left_nice, Right_nice;

  // Applying stereo image rectification on the left image
  remap(imgL,
        Left_nice,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(imgR,
        Right_nice,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying blur to reduce noise
  //GaussianBlur(Left_nice, Left_nice, Size(5, 5), 0);
  //GaussianBlur(Right_nice, Right_nice, Size(5, 5), 0);

  // Apply sharpen with laplacian filter
  //Mat imgSharp_L, imgSharp_R;
  //Laplacian(Left_nice, Left_nice, CV_16S, 3);
  //Laplacian(Right_nice, Right_nice, CV_16S, 3);

  //Mat imgSharp;
  //convertScaleAbs(imgL, imgL);
  //convertScaleAbs(imgR, imgR);

  //Sharpen
  Mat test_sharp;
  GaussianBlur(Left_nice, test_sharp, cv::Size(0, 0), 3);
  addWeighted(Left_nice, 2, test_sharp, -1.0, 0, test_sharp);

  namedWindow("test_sharp");
  imshow("test_sharp",test_sharp);
  //waitKey(1);


  //Apply correction
  Mat ball_CL, ball_CR;
  Mat balloonCorrected_L, balloonCorrected_R;
  Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
  Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
  balloonCorrect_L.setTo(B_CORRECTION);
  balloonCorrect_R.setTo(B_CORRECTION);

  add(Left_nice, balloonCorrect_L, ball_CL);
  add(Right_nice, balloonCorrect_R, ball_CR);

  //Apply HSV
  Mat left_HSV, right_HSV;
  cvtColor(ball_CL, left_HSV, cv::COLOR_BGR2HSV);
  cvtColor(ball_CR, right_HSV, cv::COLOR_BGR2HSV);

  //Isolate Ball
  Mat bMask_L, bMask_R;
  inRange(left_HSV, B_MIN, B_MAX, bMask_L);
  inRange(right_HSV, B_MIN, B_MAX, bMask_R);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  namedWindow("bMask_L");
  imshow("bMask_L", bMask_L_cleaned);

  namedWindow("bMask_R");
  imshow("bMask_R", bMask_R_cleaned);

  //Find Largest Contour (Largest Ball)
  vector<vector<Point> > contoursL;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_L;
  float largestArea_L = 0;
  int index_L = 0;

  for (unsigned int i = 0; i < contoursL.size(); i++) {
      double area = contourArea(contoursL[i]);
      if (area > largestArea_L) {
          largestArea_L = area;
          largestContour_L = contoursL[i];
          index_L = 0;
      }
  }

  vector<vector<Point> > contoursR;
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_R;
  float largestArea_R = 0;
  int index_R = 0;

  for (unsigned int i = 0; i < contoursR.size(); i++) {
      double area = contourArea(contoursR[i]);
      if (area > largestArea_R) {
          largestArea_R = area;
          largestContour_R = contoursR[i];
          index_R = i;
      }
  }

  // Exclude contours that are too small
  if (largestArea_L < 150) {
    largestContour_L.clear();
  }

  if (largestArea_R < 150) {
    largestContour_R.clear();
  } 

  // Debug draw contours
  namedWindow("contL");
  drawContours(imgL, contoursL, index_L, Scalar(255, 255, 255), -1);
  imshow("contL", imgL);

  namedWindow("contR");
  drawContours(imgR, contoursR, index_R, Scalar(255, 255, 255), -1);
  imshow("contR", imgR);

  // Center detection with blob centroid
  Moments m_L = moments(largestContour_L, true);
  Point p_L(m_L.m10/m_L.m00, m_L.m01/m_L.m00);

  //cout << largestArea_R << endl;
  //cout << largestArea_L << endl;

  Moments m_R = moments(largestContour_R, true);
  Point p_R(m_R.m10/m_R.m00, m_R.m01/m_R.m00);

  // Reveal area around chosen point in original image
  Mat maskL = Mat::zeros(imgL.size(), CV_8UC1);
  Mat maskR = Mat::zeros(imgL.size(), CV_8UC1);

  // Create Circle masks
  int radius = 50;

  circle(maskL, p_L, radius, Scalar(255, 255, 255), -1);
  circle(maskR, p_R, radius, Scalar(255, 255, 255), -1);

  threshold(maskL, maskL, 127, 255, THRESH_BINARY);
  threshold(maskR, maskR, 127, 255, THRESH_BINARY);

  // Apply mask
  Mat masked_imgL, masked_imgR;
  bitwise_and(Left_nice, Left_nice, masked_imgL, maskL);
  bitwise_and(Right_nice, Right_nice, masked_imgR, maskR);

  // Debug Circular Mask
  namedWindow("imgL");
  imshow("imgL",masked_imgL);
  //waitKey(1);

  namedWindow("imgR");
  imshow("imgR",masked_imgR);
  //waitKey(1);

  //Sharpen
  Mat sharp_L;
  GaussianBlur(masked_imgL, sharp_L, cv::Size(0, 0), 3);
  addWeighted(masked_imgL, 2, sharp_L, -1.0, 0, sharp_L);

  //Sharpen
  Mat sharp_R;
  GaussianBlur(masked_imgR, sharp_R, cv::Size(0, 0), 3);
  addWeighted(masked_imgR, 2, sharp_R, -1.0, 0, sharp_R);

  masked_imgL = sharp_L;
  masked_imgR = sharp_R;

  try {

  // Perform ORB feature extraction and matching
  int minHessian = 400;
  Ptr<ORB> orb = ORB::create(minHessian);

  vector<KeyPoint> keypointsL, keypointsR;
  Mat descriptorsL, descriptorsR;

  // Detect keypoints in the image
  orb->detect(masked_imgL, keypointsL);
  orb->detect(masked_imgR, keypointsR);

  //Define radius
  float radius = 15.0f;

  //Filter keypoints
  vector<KeyPoint> kp_filt_L;
  for (const auto& k : keypointsL)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_L.x, 2) + pow(k.pt.y - p_L.y, 2));

      if (dist < radius)
      {
        kp_filt_L.push_back(k);
      }
  }

  vector<KeyPoint> kp_filt_R;
  for (const auto& k : keypointsR)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_R.x, 2) + pow(k.pt.y - p_R.y, 2));

      if (dist < radius)
      {
        kp_filt_R.push_back(k);
      }
  }

  //DEBUG: Filtering
  //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
  //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

  //Compute matches
  orb->compute(masked_imgL, kp_filt_L, descriptorsL);
  orb->compute(masked_imgR, kp_filt_R, descriptorsR);

  // Match descriptors using Brute-Force matcher
  BFMatcher matcher(NORM_HAMMING);
  vector<vector<DMatch>> knn_matches;
  matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

  // Filter matches using ratio test
  const float ratio_thresh = 0.7f;
  vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }

  // Draw good matches
  Mat img_matches;
  drawMatches(masked_imgL, kp_filt_L, masked_imgR, kp_filt_R, knn_matches, img_matches);

  namedWindow("ORB Matches");
  imshow("ORB Matches", img_matches);
  //waitKey(1);

  // Calculate average distance of all matched points
  double avg_distance = 0.0;
  for (const auto& matches : knn_matches) {
    if (matches.size() < 2) continue;  // Skip if not enough matches
    const auto& kp_L = kp_filt_L[matches[0].queryIdx];
    const auto& kp_R1 = kp_filt_R[matches[0].trainIdx];
    const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
    double disparity = abs(kp_L.pt.x - kp_R1.pt.x);
    double ratio = matches[0].distance / matches[1].distance;
    if (ratio < ratio_thresh) {
      double distance = (F * BASELINE) / disparity;
      avg_distance += distance;
    }
  }



  // Add RANSAC maybe?


  avg_distance /= good_matches.size();

  //Assign XYZ of ball
  Z = avg_distance;
  X = (p_L.x + p_R.x)/2;
  Y = (p_L.y + p_R.y)/2;
  area = (largestArea_L + largestArea_R)/2;

  //DEBUG READ OUTPUT
  cout << "Distance: " << Z << endl;
  cout << "X: " << X << endl;
  cout << "Y: " << Y << endl;
  //cout << area << endl;
  //cout << isnan(Z) << endl;

  if (isnan(Z)) {
    return false;
  } else {
    return true;
  }

  }
  catch (const cv::Exception){
    cout << "Failed" << endl;
    return false;
  }
  
}

void ComputerVision::getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR){
  //cout << "goal" << endl;
  imshow("raw_L", imgL);
  imshow("raw_R", imgR);
  // Applying blur to reduce noise
  Mat imgBlurredL;
  GaussianBlur(imgL, imgBlurredL, Size(5, 5), 2);

  Mat imgBlurredR;
  GaussianBlur(imgR, imgBlurredR, Size(5, 5), 2);

  // Initialize matrix for rectified stereo images
  Mat Left_nice, Right_nice;

  // Applying stereo image rectification on the left image
  remap(imgBlurredL,
        Left_nice,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(imgBlurredR,
        Right_nice,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  //Apply correction
  Mat goal_CL, goal_CR;
  Mat balloonCorrected_L, balloonCorrected_R;
  Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
  Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
  balloonCorrect_L.setTo(ORANGE_G_CORRECTION);
  balloonCorrect_R.setTo(ORANGE_G_CORRECTION);

  add(Left_nice, balloonCorrect_L, goal_CL);
  add(Right_nice, balloonCorrect_R, goal_CR);

  //Apply HSV
  Mat left_HSV, right_HSV;
  cvtColor(goal_CL, left_HSV, cv::COLOR_BGR2HSV);
  cvtColor(goal_CR, right_HSV, cv::COLOR_BGR2HSV);

  //Isolate Goal
  Mat bMask_L, bMask_R;
  inRange(left_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_L);
  inRange(right_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_R);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  namedWindow("bMask_L");
  imshow("bMask_L", bMask_L_cleaned);
  //waitKey(1);

  namedWindow("bMask_R");
  imshow("bMask_R", bMask_R_cleaned);
  //waitKey(1);

  //Find Contours
  vector<vector<Point> > contoursL;
  vector<vector<Point> > contoursR;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // Apply morphological operations to fill in the gaps and complete the rectangle
  dilate(bMask_L_cleaned, bMask_L_cleaned, kernel);
  erode(bMask_L_cleaned, bMask_L_cleaned, kernel);
  dilate(bMask_R_cleaned, bMask_R_cleaned, kernel);
  erode(bMask_R_cleaned, bMask_R_cleaned, kernel);
  
  // Find the bounding rectangle of the largest contour
  Rect rectL;
  Rect rectR;

  // Try Catch for numerous failure points
  try {

    if (!contoursL.empty()) {
      size_t maxAreaIndexL = 0;
      for (size_t i = 1; i < contoursL.size(); i++) {
          if (contourArea(contoursL[i]) > contourArea(contoursL[maxAreaIndexL])) {
              maxAreaIndexL = i;
          }
      }

      if (pixelDensityL < 0.35){
        rectL = boundingRect(contoursL[maxAreaIndexL]);
      } else if (pixelDensityL > 0.35 && maxAreaIndexL != 0) {
        rectL = boundingRect(contoursL[maxAreaIndexL-1]);
      } 
      //std::cout << "index: " << maxAreaIndex << endl;
    }

    if (!contoursR.empty()) {
      size_t maxAreaIndexR = 0;
      for (size_t i = 1; i < contoursR.size(); i++) {
          if (contourArea(contoursR[i]) > contourArea(contoursR[maxAreaIndexR])) {
              maxAreaIndexR = i;
          }
      }

      if (pixelDensityR < 0.35){
        rectR = boundingRect(contoursR[maxAreaIndexR]);
      } else if (pixelDensityR > 0.35 && maxAreaIndexR != 0) {
        rectR = boundingRect(contoursR[maxAreaIndexR-1]);
      } 
      //std::cout << "index: " << maxAreaIndex << endl;
    }

    // Draw a new rectangle with the same aspect ratio and orientation
    if (!rectL.empty() && !rectR.empty()) {
        double aspectRatioL = (double)rectL.width / rectL.height;
        double aspectRatioR = (double)rectR.width / rectR.height;

        int newWidthL = (int)(aspectRatioL * rectL.height);
        int newWidthR = (int)(aspectRatioR * rectR.height);

        //center and 4 corners of the bounding box
        Point centerL = Point(rectL.x + rectL.width / 2, rectL.y + rectL.height / 2);
        Point ltCorner = Point(rectL.x, rectL.y + rectL.height);;
        Point rtCorner = Point(rectL.x + rectL.width, rectL.y + rectL.height);
        Point lbCorner = Point(rectL.x, rectL.y);
        Point rbCorner = Point(rectL.x + rectL.width, rectL.y);;

        Point centerR = Point(rectR.x + rectR.width / 2, rectR.y + rectR.height / 2);

        //show center and 4 corners
        circle(bMask_L_cleaned,centerL,1,Scalar(255,255,0),3,4,0);
        // circle(bMask_L_cleaned,ltCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,lbCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,rtCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,rbCorner,1,Scalar(255,255,0),20,4,0);
        circle(bMask_R_cleaned,centerR,1,Scalar(255,255,0),3,4,0);

        rectL = Rect(centerL.x - newWidthL / 2, rectL.y, newWidthL, rectL.height);
        rectR = Rect(centerR.x - newWidthR / 2, rectR.y, newWidthR, rectR.height);
        // std::cout << "Center(Left) (x,y): " << centerL.x << ", " << centerL.y << endl;
        // std::cout << "Center(Right) (x,y): " << centerR.x << ", " << centerR.y << endl;

        Mat cropedMaskL = bMask_L_cleaned.colRange(rectL.x,rectL.x + rectL.width).rowRange(rectL.y,rectL.y + rectL.height);
        Mat cropedMaskR = bMask_R_cleaned.colRange(rectR.x,rectR.x + rectR.width).rowRange(rectR.y,rectR.y + rectR.height);  

        int whitePixelsL = countNonZero(cropedMaskL);
        int whitePixelsR = countNonZero(cropedMaskR);
        //std::cout << "white pixels: " << whitePixels << endl;

        pixelDensityL = double(whitePixelsL)/double(rectL.width*rectL.height);
        pixelDensityR = double(whitePixelsR)/double(rectR.width*rectR.height);
        //std::cout << "pixel density: " << pixelDensity << endl;

        if (pixelDensityL > 0.1 && pixelDensityL < 0.35 && pixelDensityR > 0.1 && pixelDensityR < 0.35 ){
          rectangle(bMask_L_cleaned, rectL, Scalar(255), 2);

          //Coners
          vector<Point2f> corners;

          output = Mat::zeros(bMask_R_cleaned.size(), CV_32FC1);
          //cornerHarris for corner finding 
          cornerHarris(bMask_R_cleaned,output,5,3,0.04);
          //normalize to get the outputs 
          normalize(output,output_norm,0,255,NORM_MINMAX,CV_32FC1, Mat());
          convertScaleAbs(output_norm,output_norm_scaled);

          
          //draw the points on the image
          for (int j = 0; j<output_norm.rows;j++){
            for (int i = 0; i<output_norm.cols;i++){
              // the threshold can be changed to filter how many points are getting pushed to coners vector
              if ((int)output_norm.at<float>(j,i) > 150){
                //coordiates of the corners are (i,j), they can be stored in a vector 
                //circle(bMask_R_cleaned,Point(i,j),4,Scalar(0,0,255),2,8,0);
                corners.push_back(Point(i,j));
              }
            }
          }

          // K means clustering to remove close corners

          // Set the number of clusters
          int num_clusters = 4;

          // Convert the vector of corners to a matrix
          Mat corners_mat(corners.size(), 2, CV_32F);
          for (int i = 0; i < corners.size(); i++) {
              corners_mat.at<float>(i, 0) = corners[i].x;
              corners_mat.at<float>(i, 1) = corners[i].y;
          }

          // Perform k-means clustering
          Mat labels, centers;
          kmeans(corners_mat, num_clusters, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

          // Group the corners based on their cluster labels
          vector<vector<Point>> corner_groups(num_clusters);
          for (int i = 0; i < corners_mat.rows; i++) {
              int label = labels.at<int>(i);
              corner_groups[label].push_back(Point(corners_mat.at<float>(i, 0), corners_mat.at<float>(i, 1)));
          }

          // Compute the average of each group of corners
          vector<Point> averaged_corners;
          for (int i = 0; i < corner_groups.size(); i++) {
              if (corner_groups[i].size() > 0) {
                  int sum_x = 0, sum_y = 0;
                  for (int j = 0; j < corner_groups[i].size(); j++) {
                      sum_x += corner_groups[i][j].x;
                      sum_y += corner_groups[i][j].y;
                  }
                  int avg_x = round(sum_x / corner_groups[i].size());
                  int avg_y = round(sum_y / corner_groups[i].size());
                  averaged_corners.push_back(Point(avg_x, avg_y));
              }
          }

          std::cout << "corners" << averaged_corners << endl;

          // Draw circles at the averaged corners on the original image
          for (int i = 0; i < averaged_corners.size(); i++) {
              circle(bMask_R_cleaned, averaged_corners[i], 4, Scalar(0, 255, 0), 2);
          }

          // Set the radius of the circle around each corner point
          int radius = 30;

          // Create a grayscale image mask with the same size as the original image
          Mat mask(Left_nice.size(), CV_8UC1, Scalar(0));

          cout << averaged_corners.size() << endl;

          // Draw circles with the specified radius around each averaged corner point
          //for (int i = 0; i < averaged_corners.size(); i++) {
          //    circle(mask, averaged_corners[i], radius, Scalar(255), -1);
          //}
          
          Mat masked_imgL_;
          threshold(mask, mask, 127, 255, THRESH_BINARY);
          bitwise_and(imgL, imgL, masked_imgL_, mask);

          namedWindow("bruh");
          imshow("bruh", masked_imgL_);
          //waitKey(1);

          double widthHeightRatioL = (double)rectL.width / rectL.height;
          double widthHeightRatioR = (double)rectR.width / rectR.height;
          //std::cout << "Ratio: " << widthHeightRatio << endl;
          double areaL = rectL.width*rectL.height;
          double areaR = rectR.width*rectR.height;
          double areaRelDiff =  double((areaL-areaR)/(areaR)*100);

          if (areaRelDiff < 5) {
            //Calculate dist
            double disparity = abs(centerL.x - centerR.x);

            double distance = (F * BASELINE) / disparity;
            
            double distance2 = -0.000037*(areaL+areaR)/2 + 3.371;

            if (distance > 5.0){
              distance = 1000.0;
            } 
            
            cout << distance2 << endl;
            //std::cout << "Area (Left,Right): " << areaL << ", " << areaR << endl;

            // Set Outputs
            X = (centerL.x + centerR.x)/2;
            Y = (centerL.y + centerR.y)/2;
            Z = distance2;  // For now
            area = (areaL + areaR)/2;

            //corners 

            //C = P^-1*W
            //W = PC

            float ratio = (widthHeightRatioL+widthHeightRatioR)/2;
            //std::cout << "Ratio: " << ratio << endl;
            angle = acosf(ratio)/3.14159*180;
            if (ratio > 1){
              angle = 0;
            } 
            std::cout << "angle" << angle << endl;

          }

        } else {
          pixelDensityL = 0.2; //reinitiate
          pixelDensityR = 0.2;
        }      
    }

    // Display the resulot

    imshow("ApproximationsL", bMask_L_cleaned);
    //waitKey(1);
    
    imshow("ApproximationsR", bMask_R_cleaned);
    //waitKey(1);

    Mat masked_imgR_;
    bitwise_and(Left_nice, Left_nice, masked_imgR_, bMask_R_cleaned);

    namedWindow("Test");
    imshow("Test", masked_imgR_);
    //waitKey(1);
    
    /*
    //Approximate shapes
    std::vector<std::vector<cv::Point>> approximationsL(contoursL.size());
    for (int i = 0; i < contoursL.size(); i++) {
        approxPolyDP(contoursL[i], approximationsL[i], 50, false);
    }

    std::vector<std::vector<cv::Point>> approximationsR(contoursR.size());
    for (int i = 0; i < contoursR.size(); i++) {
        approxPolyDP(contoursR[i], approximationsR[i], 50, false);
    }

    // Visualize approximated contours
    Mat approx_image = bMask_L_cleaned.clone();
    for (int i = 0; i < approximationsL.size(); i++) {
        cv::drawContours(bMask_L_cleaned, approximationsL, i, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Approximations", bMask_L_cleaned);
    cv::waitKey(1);

    std::vector<cv::Point2f> corners;
      double qualityLevel = 0.01;
      double minDistance = 10;
      int blockSize = 3;
      bool useHarrisDetector = false;
      double k = 0.04;
      cv::goodFeaturesToTrack(bMask_L_cleaned, corners, 500, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

      for (size_t i = 0; i < corners.size(); i++)
      {
          cv::circle(bMask_L_cleaned, corners[i], 5, cv::Scalar(255), -1);
      }

    cv::imshow("Corners", bMask_L_cleaned);
    cv::waitKey(1);

    // Show image with detected incomplete rectangles
    imshow("Detected rectangles", bMask_L_cleaned);
    waitKey(1);

    */

  }
  catch (const cv::Exception){
    cout << "Failed" << endl;
    return;
  }
}

// Returns quadrant of nearest item
int ComputerVision::getAvoidance(Mat imgL, Mat imgR)
{

  // Get the dimensions of the images
  int height = imgL.rows;
  int width = imgL.cols;

  // Calculate the width and height of each section
  int section_width = width / 3;
  int section_height = height / 3;

  // Define vectors to store the rectangles and section images for each image
  std::vector<cv::Rect> sectionsL;
  std::vector<cv::Rect> sectionsR;
  std::vector<cv::Mat> section_imgsL;
  std::vector<cv::Mat> section_imgsR;

  // Loop through each section and extract it into a separate Mat object for each image
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          // Calculate the coordinates of the section
          int x = j * section_width;
          int y = i * section_height;

          // Define the rectangle for this section in the left image
          cv::Rect section_rect_L(x, y, section_width, section_height);

          // Store the rectangle in the vector for the left image
          sectionsL.push_back(section_rect_L);

          // Extract the section image from the left image and store it in the vector for the left image
          cv::Mat section_L = imgL(section_rect_L).clone();
          section_imgsL.push_back(section_L);

          // Define the rectangle for this section in the right image
          cv::Rect section_rect_R(x, y, section_width, section_height);

          // Store the rectangle in the vector for the right image
          sectionsR.push_back(section_rect_R);

          // Extract the section image from the right image and store it in the vector for the right image
          cv::Mat section_R = imgR(section_rect_R).clone();
          section_imgsR.push_back(section_R);
      }
  }

  // Show the first section image from the left image
  //cv::imshow("First section image from left image", section_imgsL[0]);
  //cv::waitKey(1);

  // Show the first section image from the right image
  //cv::imshow("First section image from right image", section_imgsR[0]);
  //cv::waitKey(1);
  
  // Define variables to keep track of the section with the smallest distance
  double min_dist = std::numeric_limits<double>::max();
  int min_index = -1;

  // Loop through each section and calculate the average distance between the corresponding pixels in each section
  for (int i = 0; i < 9; i++) {
      double dist = get_avg_dist_CA(section_imgsL[i], section_imgsR[i], to_string(i));

      // If the distance is smaller than the current minimum distance, update the variables
      if (dist < min_dist) {
          min_dist = dist;
          min_index = i;
      }
  }

  cout << "min dist: " << min_dist << endl;
  cout << "min index: " << min_index << endl;

  return 0; 
}

// Feature Mapping
float ComputerVision::get_avg_dist_FM(Mat imgL, Mat imgR, String index)
{
  try {

    // Perform ORB feature extraction and matching
    int minHessian = 800;
    Ptr<ORB> orb = ORB::create(minHessian);

    vector<KeyPoint> keypointsL, keypointsR;
    Mat descriptorsL, descriptorsR;

    // Detect keypoints in the image
    orb->detect(imgL, keypointsL);
    orb->detect(imgR, keypointsR);

    //DEBUG: Filtering
    //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
    //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

    //Compute matches
    orb->compute(imgL, keypointsL, descriptorsL);
    orb->compute(imgR, keypointsR, descriptorsR);

    // Match descriptors using Brute-Force matcher
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> knn_matches;
    matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

    //Filter matches using ratio test
    const float ratio_thresh = 0.7f;
    vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
        good_matches.push_back(knn_matches[i][0]);
      }
    }

    // Return 1000.0 if less than 8 good matches
    if (good_matches.size() < 8) {
      return 1000.0;
    }

    // Draw good matches
    Mat img_matches;
    drawMatches(imgL, keypointsL, imgR, keypointsR, good_matches, img_matches);

    imshow(index, img_matches);
    //waitKey(1);

    // Calculate average distance of all matched points
    double avg_distance = 0.0;
    for (const auto& matches : knn_matches) {
      if (matches.size() < 2) continue;  // Skip if not enough matches
      const auto& kp_L = keypointsL[matches[0].queryIdx];
      const auto& kp_R = keypointsR[matches[0].trainIdx];

      double disparity = abs(kp_L.pt.x - kp_R.pt.x);
      double ratio = matches[0].distance / matches[1].distance;

      double distance = (F * BASELINE) / disparity;
      avg_distance += distance;

    }

    // Add RANSAC maybe?

    avg_distance /= knn_matches.size();

    return avg_distance;

  }
  catch (const cv::Exception){
      cout << "Failed" << endl;
      return 1000.00;
  }
}

// Disparity Map
float ComputerVision::get_avg_dist_DM(Mat imgL, Mat imgR, String index)
{
  try {
    // Convert input images to grayscale
    Mat imgL_gray, imgR_gray;
    cvtColor(imgL, imgL_gray, COLOR_BGR2GRAY);
    cvtColor(imgR, imgR_gray, COLOR_BGR2GRAY);

    // Create a StereoBM object and set the parameters
    Ptr<StereoBM> stereo = StereoBM::create(16, 21);

    // Compute the disparity map
    Mat dispMap;
    stereo->compute(imgL_gray, imgR_gray, dispMap);

    Mat output;
    normalize(dispMap, output, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    Mat depth;
    reprojectImageTo3D(dispMap, depth, Q);

    // Display the disparity map
    imshow(index, output);
    //waitKey(1);

    // Compute the average positive disparity
    long count = 0;
    float sum = 0;
    for (int i = 0; i < depth.rows; i++) {
      for (int j = 0; j < depth.cols; j++) {
        float d = depth.at<float>(i, j);
        if (d > 0) {
          sum += d;
          count++;
        }
      }
    }

    float distance = sum / count;

    return distance; 

  }
  catch (const cv::Exception){
      cout << "Failed" << endl;
      return 1000.00;
  }
}

// Countour Area
float ComputerVision::get_avg_dist_CA(Mat imgL, Mat imgR, String index)
{
  // Convert the images to grayscale
  Mat grayL, grayR;
  cvtColor(imgL, grayL, COLOR_BGR2GRAY);
  cvtColor(imgR, grayR, COLOR_BGR2GRAY);

  // Apply a binary threshold to the images
  Mat threshL, threshR;
  threshold(grayL, threshL, 0, 255, THRESH_BINARY | THRESH_OTSU);
  threshold(grayR, threshR, 0, 255, THRESH_BINARY | THRESH_OTSU);

  // Find contours in the left frame
  vector<vector<Point>> contoursL;
  findContours(threshL, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // Find the largest contour in the left frame
  int maxContourIndexL = 0;
  double maxContourAreaL = 0;
  for (int i = 0; i < contoursL.size(); i++) {
      double contourAreaL = contourArea(contoursL[i]);
      if (contourAreaL > maxContourAreaL) {
          maxContourIndexL = i;
          maxContourAreaL = contourAreaL;
      }
  }

  // Find contours in the right frame
  vector<vector<Point>> contoursR;
  findContours(threshR, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // Find the largest contour in the right frame
  int maxContourIndexR = 0;
  double maxContourAreaR = 0;
  for (int i = 0; i < contoursR.size(); i++) {
      double contourAreaR = contourArea(contoursR[i]);
      if (contourAreaR > maxContourAreaR) {
          maxContourIndexR = i;
          maxContourAreaR = contourAreaR;
      }
  }

  // Draw the largest contours on the images
  Mat imgLContour = imgL.clone();
  Mat imgRContour = imgR.clone();
  drawContours(imgLContour, contoursL, maxContourIndexL, Scalar(0, 0, 255), 2);
  drawContours(imgRContour, contoursR, maxContourIndexR, Scalar(0, 0, 255), 2);

  // Show the images with the largest contours
  imshow(index, imgLContour);
  //waitKey(1);

  // Return the average distance between the frames
  return 0.0;
}

vector<vector<float>> ComputerVision::getTargetBalloon(){
    std::vector<std::vector<float> > target;
    //send back balloon data
    float area = 0;
    int index = -1;
    for (int i = 0; i < balloons.size(); i++) {
        if (area < balloons[i][3]) {
            area = balloons[i][3];
            index = i;
        }
    }

    if (index != -1) {
        target.push_back(balloons[index]);
    }
    return target;
}

vector<vector<float>> ComputerVision::getTargetGoal(){
    std::vector<std::vector<float> > target;
    //send back goal data
    float area = 0;
    int index = -1;
    for (int i = 0; i < goals.size(); i++) {
        if (area < goals[i][3]) {
            area = goals[i][3];
            index = i;
        }
    }

    if (index != -1) {
        target.push_back(goals[index]);
    }
    return target;
}
    
int ComputerVision::getQuad(){
    return quad;
}

bool ComputerVision::tuneBall_Lawson(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR)
{
  // Initialize matrix for rectified stereo images
  Mat Left_nice, Right_nice;

  // Applying stereo image rectification on the left image
  remap(imgL,
        Left_nice,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(imgR,
        Right_nice,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying blur to reduce noise
  //GaussianBlur(Left_nice, Left_nice, Size(5, 5), 0);
  //GaussianBlur(Right_nice, Right_nice, Size(5, 5), 0);

  // Apply sharpen with laplacian filter
  //Mat imgSharp_L, imgSharp_R;
  //Laplacian(imgL, imgL, CV_16S, 3);
  //Laplacian(imgR, imgR, CV_16S, 3);

  //Mat imgSharp;
  //convertScaleAbs(imgL, imgL);
  //convertScaleAbs(imgR, imgR);

  namedWindow("Sliders");

  createTrackbar("correction1", "Sliders", &correction1, 255);
  createTrackbar("correction2", "Sliders", &correction2, 255);
  createTrackbar("correction3", "Sliders", &correction3, 255);
  createTrackbar("min1", "Sliders", &min1, 255);
  createTrackbar("min2", "Sliders", &min2, 255);
  createTrackbar("min3", "Sliders", &min3, 255);
  createTrackbar("max1", "Sliders", &max1, 255);
  createTrackbar("max2", "Sliders", &max2, 255);
  createTrackbar("max3", "Sliders", &max3, 255);
  //waitKey(1);

  Scalar b_cor = Scalar(correction1, correction2, correction3);
  Scalar min_ = Scalar(min1, min2, min3);
  Scalar max_ = Scalar(max1, max2, max3);

  //Apply correction
  Mat ball_CL, ball_CR;
  Mat balloonCorrected_L, balloonCorrected_R;
  Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
  Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
  balloonCorrect_L.setTo(b_cor);
  balloonCorrect_R.setTo(b_cor);

  add(Left_nice, balloonCorrect_L, ball_CL);
  add(Right_nice, balloonCorrect_R, ball_CR);

  namedWindow("Raw_L");
  imshow("Raw_L", Left_nice);

  namedWindow("Corrected_L");
  imshow("Corrected_L", ball_CL);

  //Apply HSV
  Mat left_HSV, right_HSV;
  cvtColor(ball_CL, left_HSV, cv::COLOR_BGR2HSV);
  cvtColor(ball_CR, right_HSV, cv::COLOR_BGR2HSV);

  //Isolate Ball
  Mat bMask_L, bMask_R;
  inRange(left_HSV, min_, max_, bMask_L);
  inRange(right_HSV, min_, max_, bMask_R);

  vector<Mat> HSV;
  split(left_HSV, HSV); // Split image into HSV channels

  namedWindow("S");
  imshow("S", HSV[1]);

  namedWindow("V");
  imshow("V", HSV[2]);

  (HSV[1]).setTo(255); // Set S channel to max
  (HSV[2]).setTo(255); // Set V channel to max
  Mat newHSV, newHSVRGB;
  merge(HSV, newHSV);
  cvtColor(newHSV, newHSVRGB, COLOR_HSV2BGR);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  namedWindow("bMask_L");
  imshow("bMask_L", bMask_L_cleaned);

  namedWindow("bMask_R");
  imshow("bMask_R", bMask_R_cleaned);

  namedWindow("bMask_L_H");
  imshow("bMask_L_H", newHSVRGB);


  //Find Largest Contour (Largest Ball)
  vector<vector<Point> > contoursL;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_L;
  float largestArea_L = 0;
  int index_L = 0;

  for (unsigned int i = 0; i < contoursL.size(); i++) {
      double area = contourArea(contoursL[i]);
      if (area > largestArea_L) {
          largestArea_L = area;
          largestContour_L = contoursL[i];
          index_L = 0;
      }
  }

  vector<vector<Point> > contoursR;
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_R;
  float largestArea_R = 0;
  int index_R = 0;

  for (unsigned int i = 0; i < contoursR.size(); i++) {
      double area = contourArea(contoursR[i]);
      if (area > largestArea_R) {
          largestArea_R = area;
          largestContour_R = contoursR[i];
          index_R = i;
      }
  }

  // Exclude contours that are too small
  if (largestArea_L < 350) {
    largestContour_L.clear();
  }

  if (largestArea_R < 350) {
    largestContour_R.clear();
  } 

  // Debug draw contours
  namedWindow("contL");
  drawContours(imgL, contoursL, index_L, Scalar(255, 255, 255), -1);
  imshow("contL", imgL);

  namedWindow("contR");
  drawContours(imgR, contoursR, index_R, Scalar(255, 255, 255), -1);
  imshow("contR", imgR);

  // Center detection with blob centroid
  Moments m_L = moments(largestContour_L, true);
  Point p_L(m_L.m10/m_L.m00, m_L.m01/m_L.m00);

  //cout << largestArea_R << endl;
  //cout << largestArea_L << endl;

  Moments m_R = moments(largestContour_R, true);
  Point p_R(m_R.m10/m_R.m00, m_R.m01/m_R.m00);

  // Reveal area around chosen point in original image
  Mat maskL = Mat::zeros(imgL.size(), CV_8UC1);
  Mat maskR = Mat::zeros(imgL.size(), CV_8UC1);

  // Create Circle masks
  int radius = 50;

  circle(maskL, p_L, radius, Scalar(255, 255, 255), -1);
  circle(maskR, p_R, radius, Scalar(255, 255, 255), -1);

  threshold(maskL, maskL, 127, 255, THRESH_BINARY);
  threshold(maskR, maskR, 127, 255, THRESH_BINARY);

  // Apply mask
  Mat masked_imgL, masked_imgR;
  bitwise_and(Left_nice, Left_nice, masked_imgL, maskL);
  bitwise_and(Right_nice, Right_nice, masked_imgR, maskR);

  // Debug Circular Mask
  namedWindow("imgL");
  imshow("imgL",masked_imgL);
  //waitKey(1);

  namedWindow("imgR");
  imshow("imgR",masked_imgR);
  //waitKey(1);

  try {

  // Perform ORB feature extraction and matching
  int minHessian = 400;
  Ptr<ORB> orb = ORB::create(minHessian);

  vector<KeyPoint> keypointsL, keypointsR;
  Mat descriptorsL, descriptorsR;

  // Detect keypoints in the image
  orb->detect(masked_imgL, keypointsL);
  orb->detect(masked_imgR, keypointsR);

  //Define radius
  float radius = 15.0f;

  //Filter keypoints
  vector<KeyPoint> kp_filt_L;
  for (const auto& k : keypointsL)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_L.x, 2) + pow(k.pt.y - p_L.y, 2));

      if (dist < radius)
      {
        kp_filt_L.push_back(k);
      }
  }

  vector<KeyPoint> kp_filt_R;
  for (const auto& k : keypointsR)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_R.x, 2) + pow(k.pt.y - p_R.y, 2));

      if (dist < radius)
      {
        kp_filt_R.push_back(k);
      }
  }

  //DEBUG: Filtering
  //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
  //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

  //Compute matches
  orb->compute(masked_imgL, kp_filt_L, descriptorsL);
  orb->compute(masked_imgR, kp_filt_R, descriptorsR);

  // Match descriptors using Brute-Force matcher
  BFMatcher matcher(NORM_HAMMING);
  vector<vector<DMatch>> knn_matches;
  matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

  // Filter matches using ratio test
  const float ratio_thresh = 0.7f;
  vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }

  // Draw good matches
  Mat img_matches;
  drawMatches(masked_imgL, kp_filt_L, masked_imgR, kp_filt_R, knn_matches, img_matches);

  namedWindow("ORB Matches");
  imshow("ORB Matches", img_matches);
  //waitKey(1);

  // Calculate average distance of all matched points
  double avg_distance = 0.0;
  for (const auto& matches : knn_matches) {
    if (matches.size() < 2) continue;  // Skip if not enough matches
    const auto& kp_L = kp_filt_L[matches[0].queryIdx];
    const auto& kp_R1 = kp_filt_R[matches[0].trainIdx];
    const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
    double disparity = abs(kp_L.pt.x - kp_R1.pt.x);
    double ratio = matches[0].distance / matches[1].distance;
    if (ratio < ratio_thresh) {
      double distance = (F * BASELINE) / disparity;
      avg_distance += distance;
    }
  }



  // Add RANSAC maybe?


  avg_distance /= good_matches.size();

  //Assign XYZ of ball
  Z = avg_distance;
  X = (p_L.x + p_R.x)/2;
  Y = (p_L.y + p_R.y)/2;
  area = (largestArea_L + largestArea_R)/2;

  //DEBUG READ OUTPUT
  cout << "Distance: " << Z << endl;
  cout << "X: " << X << endl;
  cout << "Y: " << Y << endl;
  //cout << area << endl;
  //cout << isnan(Z) << endl;

  if (isnan(Z)) {
    return false;
  } else {
    return true;
  }

  }
  catch (const cv::Exception){
    cout << "Failed" << endl;
    return false;
  }
  
}

// Single channel
Mat absSplitMat(Mat image, float split, float max){
  //imshow("_image", image);

  Mat mask_lower, mask_upper;
  inRange(image, 0, split, mask_lower);
  bitwise_not(mask_lower, mask_upper);

  //imshow("_mask_lower", mask_lower);
  //imshow("_mask_upper", mask_upper);

  Mat lowerSub, upperSub;
  subtract(split, image, lowerSub, mask_lower);
  subtract(image, split, upperSub, mask_upper);
  //imshow("_lowerSub", lowerSub);
  //imshow("_upperSub", upperSub);

  Mat output(image.size(), image.type());
  //cout << "F1" << endl;
  output.setTo(Scalar(0));
  //cout << "F2" << endl;

  // handle lower domain
  Mat lowerSub2;
  add(lowerSub, max-split, lowerSub2);
  //imshow("lowerSub2", lowerSub2);

  // handle upper domain
  Mat upperSub2;
  subtract(upperSub, split, upperSub2);
  //imshow("upperSub2", upperSub2);

  //cout << "F3" << endl;
  lowerSub2.copyTo(output, mask_lower);
  upperSub2.copyTo(output, mask_upper);
  //cout << "F4" << endl;

  return output;
}

Mat clickedMat;
static void onMouse(int event, int x, int y, int, void*){
  if(event != EVENT_LBUTTONDOWN){
    return;
  }

  Point point(x, y);
  Mat clickedMatHSV;
  cvtColor(clickedMat, clickedMatHSV, COLOR_BGR2HSV);
  Vec3b pixel = clickedMatHSV.at<Vec3b>(point);

  cout << "Clicked pixel: (" + to_string(pixel[0]) + ", " + to_string(pixel[1]) + ", " + to_string(pixel[2]) + ")" << endl;
}

bool firstFrame = true;
bool ComputerVision::tuneBall(float &X, float &Y, float &Z, float &area, Mat img_L, Mat img_R)
{
  if(firstFrame){
    firstFrame = false;
  }else{
    //x diff = 320, y diff = 277
    moveWindow("Raw_L",       640, 248);
    moveWindow("Target HSV",  960, 248);
    moveWindow("diffMask",    960, 525);
    moveWindow("diff",        960, 802);
    moveWindow("H",           1280, 248);
    moveWindow("S",           1280, 525);
    moveWindow("V",           1280, 802);
    moveWindow("diffH",       1600, 248);
    moveWindow("diffS",       1600, 525);
    moveWindow("diffV",       1600, 802);

  }
  setMouseCallback("Raw_L", onMouse);

  // Initialize matrix for rectified stereo images
  Mat rect_L, rect_R;

  // Applying stereo image rectification on the left image
  remap(img_L,
        rect_L,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(img_R,
        rect_R,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  clickedMat = rect_L;
  // Applying blur to reduce noise
  //GaussianBlur(Left_nice, Left_nice, Size(5, 5), 0);
  //GaussianBlur(Right_nice, Right_nice, Size(5, 5), 0);

  // Apply sharpen with laplacian filter
  //Mat imgSharp_L, imgSharp_R;
  //Laplacian(imgL, imgL, CV_16S, 3);
  //Laplacian(imgR, imgR, CV_16S, 3);

  //Mat imgSharp;
  //convertScaleAbs(imgL, imgL);
  //convertScaleAbs(imgR, imgR);

  namedWindow("Sliders");

  createTrackbar("targetH", "Sliders", &targetH, 180);
  createTrackbar("targetS", "Sliders", &targetS, 255);
  createTrackbar("targetV", "Sliders", &targetV, 255);
  createTrackbar("minH", "Sliders", &minH, 255);
  createTrackbar("minS", "Sliders", &minS, 255);
  createTrackbar("minV", "Sliders", &minV, 255);
  createTrackbar("maxH", "Sliders", &maxH, 255);
  createTrackbar("maxS", "Sliders", &maxS, 255);
  createTrackbar("maxV", "Sliders", &maxV, 255);
  createTrackbar("maxDiff", "Sliders", &maxDiff, 255);
  //waitKey(1);

  Scalar minHSV = Scalar(minH, minS, minV);
  Scalar maxHSV = Scalar(maxH, maxS, maxV);

  imshow("Raw_L", rect_L);
  imshow("Raw_R", rect_R);

  Size leftSize = rect_L.size();

  //cout << "type: " << rect_L.type() << endl;
  //cout << "opencv type: " << openCVType2str(rect_L.type()) << endl;

  // Show target HSV
  Mat HSV_Target(leftSize, rect_L.type());
  Scalar targetHSV(targetH, targetS, targetV);
  HSV_Target.setTo(targetHSV);
  cvtColor(HSV_Target, HSV_Target, COLOR_HSV2BGR);
  imshow("Target HSV", HSV_Target);

  // Split H S V
  Mat HSV_L;
  cvtColor(rect_L, HSV_L, COLOR_BGR2HSV);
  vector<Mat> HSV;
  split(HSV_L, HSV); // Split image into HSV channels
  Mat tempS, tempV;
  HSV[1].copyTo(tempS);
  HSV[2].copyTo(tempV);
  tempS.setTo(255);
  tempV.setTo(255);
  vector<Mat> tempHSV;
  tempHSV.push_back(HSV[0]);
  tempHSV.push_back(tempS);
  tempHSV.push_back(tempV);
  Mat newHSV, newHSVRGB;
  merge(tempHSV, newHSV);
  cvtColor(newHSV, newHSVRGB, cv::COLOR_HSV2BGR);

  imshow("H", newHSVRGB);
  imshow("S", HSV[1]);
  imshow("V", HSV[2]);

  // Create diffs
  Mat diffH, diffS, diffV;
  diffH = absSplitMat(HSV[0], targetH, 180);
  diffS = absSplitMat(HSV[1], targetS, 255);
  diffV = absSplitMat(HSV[2], targetV, 255);

  imshow("diffH", diffH);
  imshow("diffS", diffS);
  imshow("diffV", diffV);

  Mat diffTemp, diff;
  diff = 1.0/3 * (diffH + diffS + diffV);
  //multiply(diffH, diffS, diffTemp);
  //multiply(diffV, diffTemp, diff);
  imshow("diff", diff);

  Mat diffMask;
  inRange(diff, 0, maxDiff, diffMask);
  imshow("diffMask", diffMask);
  /*

  //Apply HSV
  Mat HSV_L, HSV_R;
  cvtColor(rect_L, HSV_L, cv::COLOR_BGR2HSV);
  cvtColor(rect_R, HSV_R, cv::COLOR_BGR2HSV);
  //Isolate Ball
  Mat bMask_L, bMask_R;
  inRange(left_HSV, min_, max_, bMask_L);
  inRange(right_HSV, min_, max_, bMask_R);

  vector<Mat> HSV;
  split(left_HSV, HSV); // Split image into HSV channels

  namedWindow("S");
  imshow("S", HSV[1]);

  namedWindow("V");
  imshow("V", HSV[2]);

  (HSV[1]).setTo(255); // Set S channel to max
  (HSV[2]).setTo(255); // Set V channel to max
  Mat newHSV, newHSVRGB;
  merge(HSV, newHSV);
  cvtColor(newHSV, newHSVRGB, COLOR_HSV2BGR);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  namedWindow("bMask_L");
  imshow("bMask_L", bMask_L_cleaned);

  namedWindow("bMask_R");
  imshow("bMask_R", bMask_R_cleaned);

  namedWindow("bMask_L_H");
  imshow("bMask_L_H", newHSVRGB);


  //Find Largest Contour (Largest Ball)
  vector<vector<Point> > contoursL;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_L;
  float largestArea_L = 0;
  int index_L = 0;

  for (unsigned int i = 0; i < contoursL.size(); i++) {
      double area = contourArea(contoursL[i]);
      if (area > largestArea_L) {
          largestArea_L = area;
          largestContour_L = contoursL[i];
          index_L = 0;
      }
  }

  vector<vector<Point> > contoursR;
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_R;
  float largestArea_R = 0;
  int index_R = 0;

  for (unsigned int i = 0; i < contoursR.size(); i++) {
      double area = contourArea(contoursR[i]);
      if (area > largestArea_R) {
          largestArea_R = area;
          largestContour_R = contoursR[i];
          index_R = i;
      }
  }

  // Exclude contours that are too small
  if (largestArea_L < 350) {
    largestContour_L.clear();
  }

  if (largestArea_R < 350) {
    largestContour_R.clear();
  } 

  // Debug draw contours
  namedWindow("contL");
  drawContours(imgL, contoursL, index_L, Scalar(255, 255, 255), -1);
  imshow("contL", imgL);

  namedWindow("contR");
  drawContours(imgR, contoursR, index_R, Scalar(255, 255, 255), -1);
  imshow("contR", imgR);

  // Center detection with blob centroid
  Moments m_L = moments(largestContour_L, true);
  Point p_L(m_L.m10/m_L.m00, m_L.m01/m_L.m00);

  //cout << largestArea_R << endl;
  //cout << largestArea_L << endl;

  Moments m_R = moments(largestContour_R, true);
  Point p_R(m_R.m10/m_R.m00, m_R.m01/m_R.m00);

  // Reveal area around chosen point in original image
  Mat maskL = Mat::zeros(imgL.size(), CV_8UC1);
  Mat maskR = Mat::zeros(imgL.size(), CV_8UC1);

  // Create Circle masks
  int radius = 50;

  circle(maskL, p_L, radius, Scalar(255, 255, 255), -1);
  circle(maskR, p_R, radius, Scalar(255, 255, 255), -1);

  threshold(maskL, maskL, 127, 255, THRESH_BINARY);
  threshold(maskR, maskR, 127, 255, THRESH_BINARY);

  // Apply mask
  Mat masked_imgL, masked_imgR;
  bitwise_and(Left_nice, Left_nice, masked_imgL, maskL);
  bitwise_and(Right_nice, Right_nice, masked_imgR, maskR);

  // Debug Circular Mask
  namedWindow("imgL");
  imshow("imgL",masked_imgL);
  waitKey(1);

  namedWindow("imgR");
  imshow("imgR",masked_imgR);
  waitKey(1);

  try {

  // Perform ORB feature extraction and matching
  int minHessian = 400;
  Ptr<ORB> orb = ORB::create(minHessian);

  vector<KeyPoint> keypointsL, keypointsR;
  Mat descriptorsL, descriptorsR;

  // Detect keypoints in the image
  orb->detect(masked_imgL, keypointsL);
  orb->detect(masked_imgR, keypointsR);

  //Define radius
  float radius = 15.0f;

  //Filter keypoints
  vector<KeyPoint> kp_filt_L;
  for (const auto& k : keypointsL)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_L.x, 2) + pow(k.pt.y - p_L.y, 2));

      if (dist < radius)
      {
        kp_filt_L.push_back(k);
      }
  }

  vector<KeyPoint> kp_filt_R;
  for (const auto& k : keypointsR)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_R.x, 2) + pow(k.pt.y - p_R.y, 2));

      if (dist < radius)
      {
        kp_filt_R.push_back(k);
      }
  }

  //DEBUG: Filtering
  //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
  //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

  //Compute matches
  orb->compute(masked_imgL, kp_filt_L, descriptorsL);
  orb->compute(masked_imgR, kp_filt_R, descriptorsR);

  // Match descriptors using Brute-Force matcher
  BFMatcher matcher(NORM_HAMMING);
  vector<vector<DMatch>> knn_matches;
  matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

  // Filter matches using ratio test
  const float ratio_thresh = 0.7f;
  vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }

  // Draw good matches
  Mat img_matches;
  drawMatches(masked_imgL, kp_filt_L, masked_imgR, kp_filt_R, knn_matches, img_matches);

  namedWindow("ORB Matches");
  imshow("ORB Matches", img_matches);
  waitKey(1);

  // Calculate average distance of all matched points
  double avg_distance = 0.0;
  for (const auto& matches : knn_matches) {
    if (matches.size() < 2) continue;  // Skip if not enough matches
    const auto& kp_L = kp_filt_L[matches[0].queryIdx];
    const auto& kp_R1 = kp_filt_R[matches[0].trainIdx];
    const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
    double disparity = abs(kp_L.pt.x - kp_R1.pt.x);
    double ratio = matches[0].distance / matches[1].distance;
    if (ratio < ratio_thresh) {
      double distance = (F * BASELINE) / disparity;
      avg_distance += distance;
    }
  }



  // Add RANSAC maybe?


  avg_distance /= good_matches.size();

  //Assign XYZ of ball
  Z = avg_distance;
  X = (p_L.x + p_R.x)/2;
  Y = (p_L.y + p_R.y)/2;
  area = (largestArea_L + largestArea_R)/2;

  //DEBUG READ OUTPUT
  cout << "Distance: " << Z << endl;
  cout << "X: " << X << endl;
  cout << "Y: " << Y << endl;
  //cout << area << endl;
  //cout << isnan(Z) << endl;

  if (isnan(Z)) {
    return false;
  } else {
    return true;
  }

  }
  catch (const cv::Exception){
    cout << "Failed" << endl;
    return false;
  }
  */
 return false;
}

void ComputerVision::tuneGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR){
  //cout << "goal" << endl;
  imshow("raw_L", imgL);
  imshow("raw_R", imgR);
  // Applying blur to reduce noise
  Mat imgBlurredL;
  GaussianBlur(imgL, imgBlurredL, Size(5, 5), 2);

  Mat imgBlurredR;
  GaussianBlur(imgR, imgBlurredR, Size(5, 5), 2);

  // Initialize matrix for rectified stereo images
  Mat Left_nice, Right_nice;

  // Applying stereo image rectification on the left image
  remap(imgBlurredL,
        Left_nice,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(imgBlurredR,
        Right_nice,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);
  
  namedWindow("Sliders");

  createTrackbar("targetH", "Sliders", &targetH, 180);
  createTrackbar("targetS", "Sliders", &targetS, 255);
  createTrackbar("targetV", "Sliders", &targetV, 255);
  createTrackbar("minH", "Sliders", &minH, 255);
  createTrackbar("minS", "Sliders", &minS, 255);
  createTrackbar("minV", "Sliders", &minV, 255);
  createTrackbar("maxH", "Sliders", &maxH, 255);
  createTrackbar("maxS", "Sliders", &maxS, 255);
  createTrackbar("maxV", "Sliders", &maxV, 255);
  //waitKey(1);

  Scalar corr_ = Scalar(targetH, targetS, targetV);
  Scalar minHSV = Scalar(minH, minS, minV);
  Scalar maxHSV = Scalar(maxH, maxS, maxV);

  //Apply correction
  Mat goal_CL, goal_CR;
  Mat balloonCorrected_L, balloonCorrected_R;
  Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
  Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
  balloonCorrect_L.setTo(corr_);
  balloonCorrect_R.setTo(corr_);

  add(Left_nice, balloonCorrect_L, goal_CL);
  add(Right_nice, balloonCorrect_R, goal_CR);

  //Apply HSV
  Mat left_HSV, right_HSV;
  cvtColor(goal_CL, left_HSV, cv::COLOR_BGR2HSV);
  cvtColor(goal_CR, right_HSV, cv::COLOR_BGR2HSV);

  //Isolate Goal
  Mat bMask_L, bMask_R;
  inRange(left_HSV, minHSV, maxHSV, bMask_L);
  inRange(right_HSV, minHSV, maxHSV, bMask_R);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  namedWindow("bMask_L");
  imshow("bMask_L", bMask_L_cleaned);
  //waitKey(1);

  namedWindow("bMask_R");
  imshow("bMask_R", bMask_R_cleaned);
  //waitKey(1);

  //Find Contours
  vector<vector<Point> > contoursL;
  vector<vector<Point> > contoursR;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // Apply morphological operations to fill in the gaps and complete the rectangle
  dilate(bMask_L_cleaned, bMask_L_cleaned, kernel);
  erode(bMask_L_cleaned, bMask_L_cleaned, kernel);
  dilate(bMask_R_cleaned, bMask_R_cleaned, kernel);
  erode(bMask_R_cleaned, bMask_R_cleaned, kernel);
  
  // Find the bounding rectangle of the largest contour
  Rect rectL;
  Rect rectR;

  // Try Catch for numerous failure points
  try {

    if (!contoursL.empty()) {
      size_t maxAreaIndexL = 0;
      for (size_t i = 1; i < contoursL.size(); i++) {
          if (contourArea(contoursL[i]) > contourArea(contoursL[maxAreaIndexL])) {
              maxAreaIndexL = i;
          }
      }

      if (pixelDensityL < 0.35){
        rectL = boundingRect(contoursL[maxAreaIndexL]);
      } else if (pixelDensityL > 0.35 && maxAreaIndexL != 0) {
        rectL = boundingRect(contoursL[maxAreaIndexL-1]);
      } 
      //std::cout << "index: " << maxAreaIndex << endl;
    }

    if (!contoursR.empty()) {
      size_t maxAreaIndexR = 0;
      for (size_t i = 1; i < contoursR.size(); i++) {
          if (contourArea(contoursR[i]) > contourArea(contoursR[maxAreaIndexR])) {
              maxAreaIndexR = i;
          }
      }

      if (pixelDensityR < 0.35){
        rectR = boundingRect(contoursR[maxAreaIndexR]);
      } else if (pixelDensityR > 0.35 && maxAreaIndexR != 0) {
        rectR = boundingRect(contoursR[maxAreaIndexR-1]);
      } 
      //std::cout << "index: " << maxAreaIndex << endl;
    }

    // Draw a new rectangle with the same aspect ratio and orientation
    if (!rectL.empty() && !rectR.empty()) {
        double aspectRatioL = (double)rectL.width / rectL.height;
        double aspectRatioR = (double)rectR.width / rectR.height;

        int newWidthL = (int)(aspectRatioL * rectL.height);
        int newWidthR = (int)(aspectRatioR * rectR.height);

        //center and 4 corners of the bounding box
        Point centerL = Point(rectL.x + rectL.width / 2, rectL.y + rectL.height / 2);
        Point ltCorner = Point(rectL.x, rectL.y + rectL.height);;
        Point rtCorner = Point(rectL.x + rectL.width, rectL.y + rectL.height);
        Point lbCorner = Point(rectL.x, rectL.y);
        Point rbCorner = Point(rectL.x + rectL.width, rectL.y);;

        Point centerR = Point(rectR.x + rectR.width / 2, rectR.y + rectR.height / 2);

        //show center and 4 corners
        circle(bMask_L_cleaned,centerL,1,Scalar(255,255,0),3,4,0);
        // circle(bMask_L_cleaned,ltCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,lbCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,rtCorner,1,Scalar(255,255,0),20,4,0);
        // circle(bMask_L_cleaned,rbCorner,1,Scalar(255,255,0),20,4,0);
        circle(bMask_R_cleaned,centerR,1,Scalar(255,255,0),3,4,0);

        rectL = Rect(centerL.x - newWidthL / 2, rectL.y, newWidthL, rectL.height);
        rectR = Rect(centerR.x - newWidthR / 2, rectR.y, newWidthR, rectR.height);
        // std::cout << "Center(Left) (x,y): " << centerL.x << ", " << centerL.y << endl;
        // std::cout << "Center(Right) (x,y): " << centerR.x << ", " << centerR.y << endl;

        Mat cropedMaskL = bMask_L_cleaned.colRange(rectL.x,rectL.x + rectL.width).rowRange(rectL.y,rectL.y + rectL.height);
        Mat cropedMaskR = bMask_R_cleaned.colRange(rectR.x,rectR.x + rectR.width).rowRange(rectR.y,rectR.y + rectR.height);  

        int whitePixelsL = countNonZero(cropedMaskL);
        int whitePixelsR = countNonZero(cropedMaskR);
        //std::cout << "white pixels: " << whitePixels << endl;

        pixelDensityL = double(whitePixelsL)/double(rectL.width*rectL.height);
        pixelDensityR = double(whitePixelsR)/double(rectR.width*rectR.height);
        //std::cout << "pixel density: " << pixelDensity << endl;

        if (pixelDensityL > 0.1 && pixelDensityL < 0.35 && pixelDensityR > 0.1 && pixelDensityR < 0.35 ){
          rectangle(bMask_L_cleaned, rectL, Scalar(255), 2);

          //Coners
          vector<Point2f> corners;

          output = Mat::zeros(bMask_R_cleaned.size(), CV_32FC1);
          //cornerHarris for corner finding 
          cornerHarris(bMask_R_cleaned,output,5,3,0.04);
          //normalize to get the outputs 
          normalize(output,output_norm,0,255,NORM_MINMAX,CV_32FC1, Mat());
          convertScaleAbs(output_norm,output_norm_scaled);

          
          //draw the points on the image
          for (int j = 0; j<output_norm.rows;j++){
            for (int i = 0; i<output_norm.cols;i++){
              // the threshold can be changed to filter how many points are getting pushed to coners vector
              if ((int)output_norm.at<float>(j,i) > 150){
                //coordiates of the corners are (i,j), they can be stored in a vector 
                //circle(bMask_R_cleaned,Point(i,j),4,Scalar(0,0,255),2,8,0);
                corners.push_back(Point(i,j));
              }
            }
          }

          // K means clustering to remove close corners

          // Set the number of clusters
          int num_clusters = 4;

          // Convert the vector of corners to a matrix
          Mat corners_mat(corners.size(), 2, CV_32F);
          for (int i = 0; i < corners.size(); i++) {
              corners_mat.at<float>(i, 0) = corners[i].x;
              corners_mat.at<float>(i, 1) = corners[i].y;
          }

          // Perform k-means clustering
          Mat labels, centers;
          kmeans(corners_mat, num_clusters, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

          // Group the corners based on their cluster labels
          vector<vector<Point>> corner_groups(num_clusters);
          for (int i = 0; i < corners_mat.rows; i++) {
              int label = labels.at<int>(i);
              corner_groups[label].push_back(Point(corners_mat.at<float>(i, 0), corners_mat.at<float>(i, 1)));
          }

          // Compute the average of each group of corners
          vector<Point> averaged_corners;
          for (int i = 0; i < corner_groups.size(); i++) {
              if (corner_groups[i].size() > 0) {
                  int sum_x = 0, sum_y = 0;
                  for (int j = 0; j < corner_groups[i].size(); j++) {
                      sum_x += corner_groups[i][j].x;
                      sum_y += corner_groups[i][j].y;
                  }
                  int avg_x = round(sum_x / corner_groups[i].size());
                  int avg_y = round(sum_y / corner_groups[i].size());
                  averaged_corners.push_back(Point(avg_x, avg_y));
              }
          }

          std::cout << "corners" << averaged_corners << endl;

          // Draw circles at the averaged corners on the original image
          for (int i = 0; i < averaged_corners.size(); i++) {
              circle(bMask_R_cleaned, averaged_corners[i], 4, Scalar(0, 255, 0), 2);
          }

          // Set the radius of the circle around each corner point
          int radius = 30;

          // Create a grayscale image mask with the same size as the original image
          Mat mask(Left_nice.size(), CV_8UC1, Scalar(0));

          cout << averaged_corners.size() << endl;

          // Draw circles with the specified radius around each averaged corner point
          //for (int i = 0; i < averaged_corners.size(); i++) {
          //    circle(mask, averaged_corners[i], radius, Scalar(255), -1);
          //}
          
          Mat masked_imgL_;
          threshold(mask, mask, 127, 255, THRESH_BINARY);
          bitwise_and(imgL, imgL, masked_imgL_, mask);

          namedWindow("bruh");
          imshow("bruh", masked_imgL_);
          //waitKey(1);

          double widthHeightRatioL = (double)rectL.width / rectL.height;
          double widthHeightRatioR = (double)rectR.width / rectR.height;
          //std::cout << "Ratio: " << widthHeightRatio << endl;
          double areaL = rectL.width*rectL.height;
          double areaR = rectR.width*rectR.height;
          double areaRelDiff =  double((areaL-areaR)/(areaR)*100);

          if (areaRelDiff < 5) {
            //Calculate dist
            double disparity = abs(centerL.x - centerR.x);

            double distance = (F * BASELINE) / disparity;
            
            double distance2 = -0.000037*(areaL+areaR)/2 + 3.371;

            if (distance > 5.0){
              distance = 1000.0;
            } 
            
            cout << distance2 << endl;
            //std::cout << "Area (Left,Right): " << areaL << ", " << areaR << endl;

            // Set Outputs
            X = (centerL.x + centerR.x)/2;
            Y = (centerL.y + centerR.y)/2;
            Z = distance2;  // For now
            area = (areaL + areaR)/2;

            //corners 

            //C = P^-1*W
            //W = PC

            float ratio = (widthHeightRatioL+widthHeightRatioR)/2;
            //std::cout << "Ratio: " << ratio << endl;
            angle = acosf(ratio)/3.14159*180;
            if (ratio > 1){
              angle = 0;
            } 
            std::cout << "angle" << angle << endl;

          }

        } else {
          pixelDensityL = 0.2; //reinitiate
          pixelDensityR = 0.2;
        }      
    }

    // Display the resulot

    imshow("ApproximationsL", bMask_L_cleaned);
    //waitKey(1);
    
    imshow("ApproximationsR", bMask_R_cleaned);
    //waitKey(1);

    Mat masked_imgR_;
    bitwise_and(Left_nice, Left_nice, masked_imgR_, bMask_R_cleaned);

    namedWindow("Test");
    imshow("Test", masked_imgR_);
    //waitKey(1);
    
    /*
    //Approximate shapes
    std::vector<std::vector<cv::Point>> approximationsL(contoursL.size());
    for (int i = 0; i < contoursL.size(); i++) {
        approxPolyDP(contoursL[i], approximationsL[i], 50, false);
    }

    std::vector<std::vector<cv::Point>> approximationsR(contoursR.size());
    for (int i = 0; i < contoursR.size(); i++) {
        approxPolyDP(contoursR[i], approximationsR[i], 50, false);
    }

    // Visualize approximated contours
    Mat approx_image = bMask_L_cleaned.clone();
    for (int i = 0; i < approximationsL.size(); i++) {
        cv::drawContours(bMask_L_cleaned, approximationsL, i, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Approximations", bMask_L_cleaned);
    cv::waitKey(1);

    std::vector<cv::Point2f> corners;
      double qualityLevel = 0.01;
      double minDistance = 10;
      int blockSize = 3;
      bool useHarrisDetector = false;
      double k = 0.04;
      cv::goodFeaturesToTrack(bMask_L_cleaned, corners, 500, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

      for (size_t i = 0; i < corners.size(); i++)
      {
          cv::circle(bMask_L_cleaned, corners[i], 5, cv::Scalar(255), -1);
      }

    cv::imshow("Corners", bMask_L_cleaned);
    cv::waitKey(1);

    // Show image with detected incomplete rectangles
    imshow("Detected rectangles", bMask_L_cleaned);
    waitKey(1);

    */

  }
  catch (const cv::Exception){
    cout << "Failed" << endl;
    return;
  }
}