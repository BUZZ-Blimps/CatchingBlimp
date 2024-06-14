#include "StereoCombined.hpp"

namespace enc = sensor_msgs::image_encodings;

StereoCombined::StereoCombined() : Node("stereo_combined_node") {
    using namespace std::placeholders;

	//SPLIT
    cinfoLeft_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "elp_left");
    cinfoRight_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "elp_right");

    this->declare_parameter<std::string>("calibration_file", "camera1");
    std::string calibrationFile = this->get_parameter("calibration_file").as_string();
    std::string cinfoLeftFilePath = "package://stereo_combined/calibration/" + calibrationFile + "_elp_left.yaml";
    std::string cinfoRightFilePath = "package://stereo_combined/calibration/" + calibrationFile + "_elp_right.yaml";

    // Load calibration files
    cinfoLeft_->loadCameraInfo(cinfoLeftFilePath);
    cinfoRight_->loadCameraInfo(cinfoRightFilePath);

    // cinfoLeft_->loadCameraInfo("package://opencv_telemetry/calibration/elp_left.yaml");
    // cinfoRight_->loadCameraInfo("package://opencv_telemetry/calibration/elp_right.yaml");

    // Set camera namespace
    this->declare_parameter<std::string>("camera_ns", "BurnCreamBlimp");
    cameraNs_ = this->get_parameter("camera_ns").as_string();

    // Create publishers for left and right images
    leftImagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/" + cameraNs_ + "/left/image_raw", 1);
    rightImagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/" + cameraNs_ + "/right/image_raw", 1);

     // Create publishers for left and right camera info
    leftCameraInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/" + cameraNs_ + "/left/camera_info", 1);
    rightCameraInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/" + cameraNs_ + "/right/camera_info", 1);

    // Subscribe to synchronized image topic
    syncImageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/" + cameraNs_ + "/sync/image_raw", 1, std::bind(&StereoCombined::syncImageCallback, this, std::placeholders::_1));

    //DEBAYER
	pub_mono_left_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/left/image_mono");
	pub_color_left_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/left/image_color");

	pub_mono_right_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/right/image_mono");
	pub_color_right_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/right/image_color");

	debayer_ = this->declare_parameter("debayer", 3);

    //RECTIFIER
    interpolation = this->declare_parameter("interpolation", 1);
    pub_rect_mono_left_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/left/image_rect");
    pub_rect_color_left_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/left/image_rect_color");
    pub_rect_mono_right_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/right/image_rect");
    pub_rect_color_right_ = image_transport::create_publisher(this, "/" + cameraNs_ + "/right/image_rect_color");

    //DISPARITY
    // Declare/read parameters
    this->declare_parameter("use_system_default_qos", false);

    // Register a callback for when parameters are set
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&StereoCombined::parameterSetCb, this, _1));

    // Describe int parameters
    std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> int_params;
    add_param_to_map(
        int_params,
        "stereo_algorithm",
        "Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1)",
        0, 0, 1, 1);  // default, from, to, step
    add_param_to_map(
        int_params,
        "prefilter_size",
        "Normalization window size in pixels (must be odd)",
        11, 5, 255, 2);
    add_param_to_map(
        int_params,
        "prefilter_cap",
        "Bound on normalized pixel values",
        63, 1, 63, 1);
    add_param_to_map(
        int_params,
        "correlation_window_size",
        "SAD correlation window width in pixels (must be odd)",
        21, 5, 255, 2);
    add_param_to_map(
        int_params,
        "min_disparity",
        "Disparity to begin search at in pixels",
        0, -2048, 2048, 1);
    add_param_to_map(
        int_params,
        "disparity_range",
        "Number of disparities to search in pixels (must be a multiple of 16)",
        128, 32, 4096, 16);
    add_param_to_map(
        int_params,
        "texture_threshold",
        "Filter out if SAD window response does not exceed texture threshold",
        2175, 0, 10000, 1);
    add_param_to_map(
        int_params,
        "speckle_size",
        "Reject regions smaller than this size in pixels",
        138, 0, 1000, 1);
    add_param_to_map(
        int_params,
        "speckle_range",
        "Maximum allowed difference between detected disparities",
        31, 0, 31, 1);
    add_param_to_map(
        int_params,
        "disp12_max_diff",
        "Maximum allowed difference in the left-right disparity check in pixels"
        " (Semi-Global Block Matching only)",
        5, 0, 128, 1);

    // Describe double parameters
    std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> double_params;
    add_param_to_map(
        double_params,
        "uniqueness_ratio",
        "Filter out if best match does not sufficiently exceed the next-best match",
        5.0, 0.0, 100.0, 0.0);
    add_param_to_map(
        double_params,
        "P1",
        "The first parameter ccontrolling the disparity smoothness (Semi-Global Block Matching only)",
        200.0, 0.0, 4000.0, 0.0);
    add_param_to_map(
        double_params,
        "P2",
        "The second parameter ccontrolling the disparity smoothness (Semi-Global Block Matching only)",
        400.0, 0.0, 4000.0, 0.0);

    // Describe bool parameters
    std::map<std::string, std::pair<bool, rcl_interfaces::msg::ParameterDescriptor>> bool_params;
    rcl_interfaces::msg::ParameterDescriptor full_dp_descriptor;
    full_dp_descriptor.description = "Run the full variant of the algorithm (Semi-Global Block Matching only)";
    bool_params["full_dp"] = std::make_pair(false, full_dp_descriptor);

    // Declaring parameters triggers the previously registered callback
    this->declare_parameters("", int_params);
    this->declare_parameters("", double_params);
    this->declare_parameters("", bool_params);

    pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("/" + cameraNs_ + "/disparity", 10);
    pub_disparity_img_ = create_publisher<sensor_msgs::msg::Image>("/" + cameraNs_ + "/disparity_img", 10);

    //POINT CLOUD
    // Declare/read parameters
    // bool approx = this->declare_parameter("approximate_sync", false);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    // TODO(ivanpauno): Confirm if using point cloud padding in `sensor_msgs::msg::PointCloud2`
    // can improve performance in some cases or not.
    descriptor.description =
      "This parameter avoids using alignment padding in the generated point cloud."
      "This reduces bandwidth requirements, as the point cloud size is halved."
      "Using point clouds without alignment padding might degrade performance for some algorithms.";
    this->declare_parameter("avoid_point_cloud_padding", false, descriptor);
    this->declare_parameter("use_color", true);

    pub_targets_ = create_publisher<std_msgs::msg::Float64MultiArray>("/" + cameraNs_ + "/targets", 10);
    pub_points2_ = create_publisher<sensor_msgs::msg::PointCloud2>("/" + cameraNs_ + "/points2", 10);
    pub_pixels_ = create_publisher<std_msgs::msg::Int64MultiArray>("/" + cameraNs_ + "/pixels", 10);
    pub_avoidance_ = create_publisher<std_msgs::msg::Float64MultiArray>("/" + cameraNs_ + "/avoidance", 10);

    bbox_msg = std::make_shared<yolo_msgs::msg::BoundingBox>();
    // boundingBoxSub_ = this->create_subscription<yolo_msgs::msg::BoundingBox>(
    //     "/" + cameraNs_ + "/bounding_box", 10, std::bind(&StereoCombined::boundingBoxCallback, this, std::placeholders::_1));
}

void StereoCombined::boundingBoxCallback(const yolo_msgs::msg::BoundingBox::SharedPtr bounding_box_msg) {
    //Update bounding box message
    bbox_msg = std::make_shared<yolo_msgs::msg::BoundingBox>(*bounding_box_msg);
    // bbox_msg = bounding_box_msg;
}

void StereoCombined::syncImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    // Given Image, convert to OpenCV Mat
    cv_bridge::CvImagePtr cvPtrLeft;
    cv_bridge::CvImagePtr cvPtrRight;
    std::string leftFrame = cameraNs_ + "_left_optical_frame";
    std::string rightFrame = cameraNs_ + "_right_optical_frame";
    try {
        cvPtrLeft = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvPtrRight = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    int combinedRows = cvPtrLeft->image.rows;
    int combinedCols = cvPtrLeft->image.cols;
    int imageCols = combinedCols / 2;
    int imageRows = combinedRows;

    // Split the combined image into left and right images
    cv::Rect leftROI(0, 0, imageCols, imageRows);
    cv::Rect rightROI(imageCols, 0, imageCols, imageRows);
    cv::Mat leftCrop = cvPtrLeft->image(leftROI);
    cv::Mat rightCrop = cvPtrRight->image(rightROI);
    cvPtrLeft->image = leftCrop;
    cvPtrRight->image = rightCrop;
    cvPtrLeft->header.frame_id = leftFrame;
    cvPtrRight->header.frame_id = rightFrame;
    
    //get the header for each   
    int32_t seconds_left = cvPtrLeft->header.stamp.sec;
    int32_t nanoseconds_left = cvPtrLeft->header.stamp.nanosec;

    int32_t seconds_right = cvPtrRight->header.stamp.sec;
    int32_t nanoseconds_right = cvPtrRight->header.stamp.nanosec;

    // Get camera infos
    sensor_msgs::msg::CameraInfo camera_info_left_ = cinfoLeft_->getCameraInfo();
    sensor_msgs::msg::CameraInfo camera_info_right_ = cinfoRight_->getCameraInfo();
    
    //Set camera info headers
    camera_info_left_.header.frame_id = leftFrame;
    camera_info_left_.header.stamp.sec = seconds_left;
    camera_info_left_.header.stamp.nanosec = nanoseconds_left;

    camera_info_right_.header.frame_id = rightFrame;
    camera_info_right_.header.stamp.sec = seconds_right;
    camera_info_right_.header.stamp.nanosec = nanoseconds_right;

    sensor_msgs::msg::Image image_raw_left = *(cvPtrLeft->toImageMsg());
    sensor_msgs::msg::Image image_raw_right = *(cvPtrRight->toImageMsg());

    sensor_msgs::msg::Image::SharedPtr image_raw_left_ptr = std::make_shared<sensor_msgs::msg::Image>(image_raw_left);
    sensor_msgs::msg::Image::SharedPtr image_raw_right_ptr = std::make_shared<sensor_msgs::msg::Image>(image_raw_right);

    // Publish the left and right raw images
    leftImagePub_->publish(image_raw_left);
    rightImagePub_->publish(image_raw_right);

    // Publish the left and right camera info
    leftCameraInfoPub_->publish(camera_info_left_);
    rightCameraInfoPub_->publish(camera_info_right_);

    RCLCPP_INFO(this->get_logger(), "published");

    /*************** Debay Left ***************/
    sensor_msgs::msg::Image::SharedPtr color_left_msg = std::make_shared<sensor_msgs::msg::Image>();
    sensor_msgs::msg::Image::SharedPtr mono_left_msg = std::make_shared<sensor_msgs::msg::Image>();

    int bit_depth_left = enc::bitDepth(image_raw_left.encoding);

    // TODO(someone): Fix as soon as bitDepth fixes it
    if (image_raw_left.encoding == enc::YUV422) {
        bit_depth_left = 8;
    }

    RCLCPP_INFO(this->get_logger(), "1");

    // First publish to mono if needed
    if (enc::isMono(image_raw_left.encoding)) {
        if (pub_mono_left_.getNumSubscribers()) pub_mono_left_.publish(image_raw_left_ptr);
    } else {
        if ((bit_depth_left != 8) && (bit_depth_left != 16)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Raw image data from topic '%s' has unsupported depth: %d",
                "/left/image_raw", bit_depth_left);
        } else {
            // Use cv_bridge to convert to Mono. If a type is not supported,
            // it will error out there
            try {
                RCLCPP_INFO(this->get_logger(), "3");

                if (bit_depth_left == 8) {

                    cv_bridge::CvImagePtr cv_img_left = cv_bridge::toCvCopy(image_raw_left_ptr, enc::MONO8);
                    
                    // RCLCPP_INFO(this->get_logger(), "rows=%d, cols=%d", cv_img_left->image.rows, cv_img_left->image.cols);
                    
                    mono_left_msg = cv_img_left->toImageMsg();
                } else {
                    RCLCPP_INFO(this->get_logger(), "its not 8");
                    mono_left_msg = cv_bridge::toCvCopy(image_raw_left_ptr, enc::MONO16)->toImageMsg();
                }

                RCLCPP_INFO(this->get_logger(), "its not the publish call");

                if (pub_mono_left_.getNumSubscribers()) pub_mono_left_.publish(mono_left_msg);
            } catch (cv_bridge::Exception & e) {
                RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
            }
        }
    }

    return;

    // Next, publish to color
    // if (!pub_color_left_.getNumSubscribers()) {
    //     return;
    // }

    if (enc::isMono(image_raw_left.encoding)) {
        // For monochrome, no processing needed!
        color_left_msg = std::make_shared<sensor_msgs::msg::Image>(image_raw_left);
        if (pub_color_left_.getNumSubscribers()) pub_color_left_.publish(color_left_msg);

        // Warn if the user asked for color
        RCLCPP_WARN(
            this->get_logger(),
            "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
            pub_color_left_.getTopic().c_str(), "/left/image_raw");
    } else if (enc::isColor(image_raw_left.encoding)) {
        color_left_msg = std::make_shared<sensor_msgs::msg::Image>(image_raw_left);
        if (pub_color_left_.getNumSubscribers()) pub_color_left_.publish(color_left_msg);
    } else if (enc::isBayer(image_raw_left.encoding)) {
        int type_left = bit_depth_left == 8 ? CV_8U : CV_16U;
        const cv::Mat bayer_left(
            image_raw_left.height, image_raw_left.width, CV_MAKETYPE(type_left, 1),
            const_cast<uint8_t *>(&image_raw_left.data[0]), image_raw_left.step);

        color_left_msg->header = image_raw_left.header;
        color_left_msg->height = image_raw_left.height;
        color_left_msg->width = image_raw_left.width;
        color_left_msg->encoding = bit_depth_left == 8 ? enc::BGR8 : enc::BGR16;
        color_left_msg->step = color_left_msg->width * 3 * (bit_depth_left / 8);
        color_left_msg->data.resize(color_left_msg->height * color_left_msg->step);

        cv::Mat color_left(color_left_msg->height, color_left_msg->width, CV_MAKETYPE(type_left, 3), &color_left_msg->data[0], color_left_msg->step);

        int algorithm_left;
        // std::loc_guard<std::recursive_mutex> loc(config_mutex_)
        algorithm_left = debayer_;

        if (algorithm_left == debayer_edgeaware_ || algorithm_left == debayer_edgeaware_weighted_) {
            // These algorithms are not in OpenCV yet
            if (image_raw_left.encoding != enc::BAYER_GRBG8) {
                RCLCPP_WARN(
                    this->get_logger(), "Edge aware algorithms currently only support GRBG8 Bayer. "
                    "Falling back to bilinear interpolation.");
                algorithm_left = debayer_bilinear_;
            } else {
                if (algorithm_left == debayer_edgeaware_) {
                    debayerEdgeAware(bayer_left, color_left);
                } else {
                    debayerEdgeAwareWeighted(bayer_left, color_left);
                }
            }
        }

        if (algorithm_left == debayer_bilinear_ || algorithm_left == debayer_vng_) {
            int code_left = -1;

            if (image_raw_left.encoding == enc::BAYER_RGGB8 || image_raw_left.encoding == enc::BAYER_RGGB16) {
                code_left = cv::COLOR_BayerBG2BGR;
            } else if (image_raw_left.encoding == enc::BAYER_BGGR8 || image_raw_left.encoding == enc::BAYER_BGGR16) {
                code_left = cv::COLOR_BayerRG2BGR;
            } else if (image_raw_left.encoding == enc::BAYER_GBRG8 || image_raw_left.encoding == enc::BAYER_GBRG16) {
                code_left = cv::COLOR_BayerGR2BGR;
            } else if (image_raw_left.encoding == enc::BAYER_GRBG8 || image_raw_left.encoding == enc::BAYER_GRBG16) {
                code_left = cv::COLOR_BayerGB2BGR;
            }

            if (algorithm_left == debayer_vng_) {
                code_left += cv::COLOR_BayerBG2BGR_VNG - cv::COLOR_BayerBG2BGR;
            }

            cv::cvtColor(bayer_left, color_left, code_left);
        }

        if (pub_color_left_.getNumSubscribers()) pub_color_left_.publish(color_left_msg);
    } else if (image_raw_left.encoding == enc::YUV422 || image_raw_left.encoding == enc::YUV422_YUY2) {
        // Use cv_bridge to convert to BGR8
        sensor_msgs::msg::Image::SharedPtr color_left_msg;

        try {
            color_left_msg = cv_bridge::toCvCopy(image_raw_left, enc::BGR8)->toImageMsg();
            if (pub_color_left_.getNumSubscribers()) pub_color_left_.publish(color_left_msg);
        } catch (cv_bridge::Exception & e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
        }
    } else if (image_raw_left.encoding == enc::TYPE_8UC3) {
        // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
        RCLCPP_WARN(
            this->get_logger(),
            "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
            "source should set the encoding to 'bgr8' or 'rgb8'.",
            "/left/image_raw");
    } else {
        RCLCPP_WARN(
            this->get_logger(), "Raw image topic '%s' has unsupported encoding '%s'",
            "/left/image_raw", image_raw_left.encoding.c_str());
    }

    /*************** Debay Right ***************/
    sensor_msgs::msg::Image::SharedPtr color_right_msg = std::make_shared<sensor_msgs::msg::Image>();
    sensor_msgs::msg::Image::SharedPtr mono_right_msg = std::make_shared<sensor_msgs::msg::Image>();

    int bit_depth_right = enc::bitDepth(image_raw_right.encoding);

    // TODO(someone): Fix as soon as bitDepth fixes it
    if (image_raw_right.encoding == enc::YUV422) {
        bit_depth_right = 8;
    }

    // First publish to mono if needed
    if (enc::isMono(image_raw_right.encoding)) {
        mono_right_msg = std::make_shared<sensor_msgs::msg::Image>(image_raw_right);
        if (pub_mono_right_.getNumSubscribers()) pub_mono_right_.publish(mono_right_msg);
    } else {
        if ((bit_depth_right != 8) && (bit_depth_right != 16)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Raw image data from topic '%s' has unsupported depth: %d",
                "/right/image_raw", bit_depth_right);
        } else {
            // Use cv_bridge to convert to Mono. If a type is not supported,
            // it will error out there
            try {
                if (bit_depth_right == 8) {
                    mono_right_msg = cv_bridge::toCvCopy(image_raw_right, enc::MONO8)->toImageMsg();
                } else {
                    mono_right_msg = cv_bridge::toCvCopy(image_raw_right, enc::MONO16)->toImageMsg();
                }

                if (pub_mono_right_.getNumSubscribers()) pub_mono_right_.publish(mono_right_msg);
            } catch (cv_bridge::Exception & e) {
                RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
            }
        }
    }

    // Next, publish to color
    // if (!pub_color_right_.getNumSubscribers()) {
    //     return;
    // }

    if (enc::isMono(image_raw_right.encoding)) {
        // For monochrome, no processing needed!
        color_right_msg = std::make_shared<sensor_msgs::msg::Image>(image_raw_right);
        if (pub_color_right_.getNumSubscribers()) pub_color_right_.publish(color_right_msg);

        // Warn if the user asked for color
        RCLCPP_WARN(
            this->get_logger(),
            "Color topic '%s' requested, but raw image data from topic '%s' is grayscale",
            pub_color_right_.getTopic().c_str(), "/right/image_raw");
    } else if (enc::isColor(image_raw_right.encoding)) {
        color_right_msg = std::make_shared<sensor_msgs::msg::Image>(image_raw_right);
        if (pub_color_right_.getNumSubscribers()) pub_color_right_.publish(color_right_msg);
    } else if (enc::isBayer(image_raw_right.encoding)) {
        int type_right = bit_depth_right == 8 ? CV_8U : CV_16U;
        const cv::Mat bayer_right(
            image_raw_right.height, image_raw_right.width, CV_MAKETYPE(type_right, 1),
            const_cast<uint8_t *>(&image_raw_right.data[0]), image_raw_right.step);

        color_right_msg->header = image_raw_right.header;
        color_right_msg->height = image_raw_right.height;
        color_right_msg->width = image_raw_right.width;
        color_right_msg->encoding = bit_depth_right == 8 ? enc::BGR8 : enc::BGR16;
        color_right_msg->step = color_right_msg->width * 3 * (bit_depth_right / 8);
        color_right_msg->data.resize(color_right_msg->height * color_right_msg->step);

        cv::Mat color_right(color_right_msg->height, color_right_msg->width, CV_MAKETYPE(type_right, 3), &color_right_msg->data[0], color_right_msg->step);

        int algorithm_right;
        algorithm_right = debayer_;

        if (algorithm_right == debayer_edgeaware_ || algorithm_right == debayer_edgeaware_weighted_) {
            // These algorithms are not in OpenCV yet
            if (image_raw_right.encoding != enc::BAYER_GRBG8) {
                RCLCPP_WARN(
                    this->get_logger(), "Edge aware algorithms currently only support GRBG8 Bayer. "
                    "Falling back to bilinear interpolation.");
                algorithm_right = debayer_bilinear_;
            } else {
                if (algorithm_right == debayer_edgeaware_) {
                    debayerEdgeAware(bayer_right, color_right);
                } else {
                    debayerEdgeAwareWeighted(bayer_right, color_right);
                }
            }
        }

        if (algorithm_right == debayer_bilinear_ || algorithm_right == debayer_vng_) {
            int code_right = -1;

            if (image_raw_right.encoding == enc::BAYER_RGGB8 || image_raw_right.encoding == enc::BAYER_RGGB16) {
                code_right = cv::COLOR_BayerBG2BGR;
            } else if (image_raw_right.encoding == enc::BAYER_BGGR8 || image_raw_right.encoding == enc::BAYER_BGGR16) {
                code_right = cv::COLOR_BayerRG2BGR;
            } else if (image_raw_right.encoding == enc::BAYER_GBRG8 || image_raw_right.encoding == enc::BAYER_GBRG16) {
                code_right = cv::COLOR_BayerGR2BGR;
            } else if (image_raw_right.encoding == enc::BAYER_GRBG8 || image_raw_right.encoding == enc::BAYER_GRBG16) {
                code_right = cv::COLOR_BayerGB2BGR;
            }

            if (algorithm_right == debayer_vng_) {
                code_right += cv::COLOR_BayerBG2BGR_VNG - cv::COLOR_BayerBG2BGR;
            }

            cv::cvtColor(bayer_right, color_right, code_right);
        }

        if (pub_color_right_.getNumSubscribers()) pub_color_right_.publish(color_right_msg);
    } else if (image_raw_right.encoding == enc::YUV422 || image_raw_right.encoding == enc::YUV422_YUY2) {
        // Use cv_bridge to convert to BGR8
        try {
            color_right_msg = cv_bridge::toCvCopy(image_raw_right, enc::BGR8)->toImageMsg();
            if (pub_color_right_.getNumSubscribers()) pub_color_right_.publish(color_right_msg);
        } catch (cv_bridge::Exception & e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge conversion error: '%s'", e.what());
        }
    } else if (image_raw_right.encoding == enc::TYPE_8UC3) {
        // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
        RCLCPP_WARN(
            this->get_logger(),
            "Raw image topic '%s' has ambiguous encoding '8UC3'. The "
            "source should set the encoding to 'bgr8' or 'rgb8'.",
            "/right/image_raw");
    } else {
        RCLCPP_WARN(
            this->get_logger(), "Raw image topic '%s' has unsupported encoding '%s'",
            "/right/image_raw", image_raw_right.encoding.c_str());
    }

    /*************** Rectify Left Mono ***************/
    sensor_msgs::msg::Image::SharedPtr rect_msg_mono_left = std::make_shared<sensor_msgs::msg::Image>();

    // Verify camera is actually calibrated
    if (camera_info_left_.k[0] == 0.0) {
        RCLCPP_ERROR(
            this->get_logger(), "Rectified topic '%s' requested but camera publishing '%s' "
            "is uncalibrated", pub_rect_mono_left_.getTopic().c_str(), "/left/camera_info");
        return;
    }

    // If zero distortion, just pass the message along
    bool zero_distortion_left = true;

    for (size_t i = 0; i < camera_info_left_.d.size(); ++i) {
        if (camera_info_left_.d[i] != 0.0) {
            zero_distortion_left = false;
            break;
        }
    }

    // This will be true if D is empty/zero sized
    if (zero_distortion_left) {
        rect_msg_mono_left = mono_left_msg;
        if (pub_rect_mono_left_.getNumSubscribers()) pub_rect_mono_left_.publish(rect_msg_mono_left);
    }

    cv_bridge::CvImagePtr cv_image_mono_left = cv_bridge::toCvCopy(mono_left_msg);
    cv::Mat resized_image_mono_left;
    cv::resize(cv_image_mono_left->image, resized_image_mono_left, cv::Size(cv_image_mono_left->image.cols, cv_image_mono_left->image.rows)); //original size right now

    cv::Mat rect_left; //empty matrix

    // Rectify and publish
    cv::Mat map_1_left, map_2_left;
    cv::Size size_left = cv::Size(camera_info_left_.width, camera_info_left_.height);

    cv::Matx33d K_left(camera_info_left_.k[0], camera_info_left_.k[1], camera_info_left_.k[2],
        camera_info_left_.k[3], camera_info_left_.k[4], camera_info_left_.k[5],
        camera_info_left_.k[6], camera_info_left_.k[7], camera_info_left_.k[8]);

    cv::Matx33d R_left(camera_info_left_.r[0], camera_info_left_.r[1], camera_info_left_.r[2],
        camera_info_left_.r[3], camera_info_left_.r[4], camera_info_left_.r[5],
        camera_info_left_.r[6], camera_info_left_.r[7], camera_info_left_.r[8]);

    cv::Matx34d P_left(camera_info_left_.p[0], camera_info_left_.p[1], camera_info_left_.p[2], camera_info_left_.p[3],
        camera_info_left_.p[4], camera_info_left_.p[5], camera_info_left_.p[6], camera_info_left_.p[7],
        camera_info_left_.p[8], camera_info_left_.p[9], camera_info_left_.p[10], camera_info_left_.p[11]);

    cv::Mat D_left(1, 5, CV_32F);
    D_left.at<float>(0, 0) = camera_info_left_.d[0];
    D_left.at<float>(0, 1) = camera_info_left_.d[1];
    D_left.at<float>(0, 2) = camera_info_left_.d[2];
    D_left.at<float>(0, 3) = camera_info_left_.d[3];
    D_left.at<float>(0, 4) = camera_info_left_.d[4];

    cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, size_left, CV_16SC2, map_1_left, map_2_left);
    cv::remap(resized_image_mono_left, rect_left, map_1_left, map_2_left, interpolation);

    // Allocate new rectified image message
    rect_msg_mono_left = cv_bridge::CvImage(mono_left_msg->header, mono_left_msg->encoding, rect_left).toImageMsg();
    if (pub_rect_mono_left_.getNumSubscribers()) pub_rect_mono_left_.publish(rect_msg_mono_left);

    /*************** Rectify Left Color ***************/
    sensor_msgs::msg::Image::SharedPtr rect_msg_color_left = std::make_shared<sensor_msgs::msg::Image>();

    // This will be true if D is empty/zero sized
    if (zero_distortion_left) {
        rect_msg_color_left = color_left_msg;
        if (pub_rect_color_left_.getNumSubscribers()) pub_rect_color_left_.publish(rect_msg_color_left);
    }

    cv_bridge::CvImagePtr cv_image_color_left = cv_bridge::toCvCopy(color_left_msg);
    cv::Mat resized_image_color_left;
    cv::resize(cv_image_color_left->image, resized_image_color_left, cv::Size(cv_image_color_left->image.cols, cv_image_color_left->image.rows)); //original size right now

    cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, size_left, CV_16SC2, map_1_left, map_2_left);
    cv::remap(resized_image_color_left, rect_left, map_1_left, map_2_left, interpolation);

    // Allocate new rectified image message
    rect_msg_color_left = cv_bridge::CvImage(color_left_msg->header, color_left_msg->encoding, rect_left).toImageMsg();
    if (pub_rect_color_left_.getNumSubscribers()) pub_rect_color_left_.publish(rect_msg_color_left);

    /*************** Rectify Right Mono ***************/
    sensor_msgs::msg::Image::SharedPtr rect_msg_mono_right = std::make_shared<sensor_msgs::msg::Image>();

    // Verify camera is actually calibrated
    if (camera_info_right_.k[0] == 0.0) {
        RCLCPP_ERROR(
            this->get_logger(), "Rectified topic '%s' requested but camera publishing '%s' "
            "is uncalibrated", pub_rect_mono_right_.getTopic().c_str(), "/right/camera_info");
        return;
    }

    // If zero distortion, just pass the message along
    bool zero_distortion_right = true;

    for (size_t i = 0; i < camera_info_right_.d.size(); ++i) {
        if (camera_info_right_.d[i] != 0.0) {
            zero_distortion_right = false;
            break;
        }
    }

    // This will be true if D is empty/zero sized
    if (zero_distortion_right) {
        rect_msg_mono_right = mono_right_msg;
        if (pub_rect_mono_right_.getNumSubscribers()) pub_rect_mono_right_.publish(rect_msg_mono_right);
    }

    cv_bridge::CvImagePtr cv_image_mono_right = cv_bridge::toCvCopy(mono_right_msg);
    cv::Mat resized_image_mono_right;
    cv::resize(cv_image_mono_right->image, resized_image_mono_right, cv::Size(cv_image_mono_right->image.cols, cv_image_mono_right->image.rows)); //original size right now

    cv::Mat rect_right; //empty matrix

    // Rectify and publish
    cv::Mat map_1_right, map_2_right;
    cv::Size size_right = cv::Size(camera_info_right_.width, camera_info_right_.height);

    cv::Matx33d K_right(camera_info_right_.k[0], camera_info_right_.k[1], camera_info_right_.k[2],
        camera_info_right_.k[3], camera_info_right_.k[4], camera_info_right_.k[5],
        camera_info_right_.k[6], camera_info_right_.k[7], camera_info_right_.k[8]);

    cv::Matx33d R_right(camera_info_right_.r[0], camera_info_right_.r[1], camera_info_right_.r[2],
        camera_info_right_.r[3], camera_info_right_.r[4], camera_info_right_.r[5],
        camera_info_right_.r[6], camera_info_right_.r[7], camera_info_right_.r[8]);

    cv::Matx34d P_right(camera_info_right_.p[0], camera_info_right_.p[1], camera_info_right_.p[2], camera_info_right_.p[3],
        camera_info_right_.p[4], camera_info_right_.p[5], camera_info_right_.p[6], camera_info_right_.p[7],
        camera_info_right_.p[8], camera_info_right_.p[9], camera_info_right_.p[10], camera_info_right_.p[11]);

    cv::Mat D_right(1, 5, CV_32F);
    D_right.at<float>(0, 0) = camera_info_right_.d[0];
    D_right.at<float>(0, 1) = camera_info_right_.d[1];
    D_right.at<float>(0, 2) = camera_info_right_.d[2];
    D_right.at<float>(0, 3) = camera_info_right_.d[3];
    D_right.at<float>(0, 4) = camera_info_right_.d[4];

    cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, size_right, CV_16SC2, map_1_right, map_2_right);
    cv::remap(resized_image_mono_right, rect_right, map_1_right, map_2_right, interpolation);

    // Allocate new rectified image message
    rect_msg_mono_right = cv_bridge::CvImage(mono_right_msg->header, mono_right_msg->encoding, rect_right).toImageMsg();
    if (pub_rect_mono_right_.getNumSubscribers()) pub_rect_mono_right_.publish(rect_msg_mono_right);

    /*************** Rectify Right Color ***************/
    sensor_msgs::msg::Image::SharedPtr rect_msg_color_right = std::make_shared<sensor_msgs::msg::Image>();

    // This will be true if D is empty/zero sized
    if (zero_distortion_right) {
        rect_msg_color_right = color_right_msg;
        if (pub_rect_color_right_.getNumSubscribers()) pub_rect_color_right_.publish(rect_msg_color_right);
    }

    cv_bridge::CvImagePtr cv_image_color_right = cv_bridge::toCvCopy(color_right_msg);
    cv::Mat resized_image_color_right;
    cv::resize(cv_image_color_right->image, resized_image_color_right, cv::Size(cv_image_color_right->image.cols, cv_image_color_right->image.rows)); //original size right now

    cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, size_right, CV_16SC2, map_1_right, map_2_right);
    cv::remap(resized_image_color_right, rect_right, map_1_right, map_2_right, interpolation);

    // Allocate new rectified image message
    rect_msg_color_right = cv_bridge::CvImage(color_right_msg->header, color_right_msg->encoding, rect_right).toImageMsg();
    if (pub_rect_color_right_.getNumSubscribers()) pub_rect_color_right_.publish(rect_msg_color_right);

    /*************** Disparity (Mono only) ***************/
    // Update the camera model
    stereo_model_.fromCameraInfo(camera_info_left_, camera_info_right_);

    // Allocate new disparity image message
    auto disparity_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
    disparity_msg->header = camera_info_left_.header;
    disparity_msg->image.header = camera_info_left_.header;

    // Compute window of (potentially) valid disparities
    int border = block_matcher_.getCorrelationWindowSize() / 2;
    int left = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
    int wtf;
    if (block_matcher_.getMinDisparity() >= 0) {
        wtf = border + block_matcher_.getMinDisparity();
    } else {
        wtf = std::max(border, -block_matcher_.getMinDisparity());
    }
    // TODO(jacobperron): the message width has not been set yet! What should we do here?
    int right = disparity_msg->image.width - 1 - wtf;
    int top = border;
    int bottom = disparity_msg->image.height - 1 - border;
    disparity_msg->valid_window.x_offset = left;
    disparity_msg->valid_window.y_offset = top;
    disparity_msg->valid_window.width = right - left;
    disparity_msg->valid_window.height = bottom - top;

    // Create cv::Mat views onto all buffers
    const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(mono_left_msg, sensor_msgs::image_encodings::MONO8)->image;
    const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(mono_right_msg, sensor_msgs::image_encodings::MONO8)->image;

    // Perform block matching to find the disparities
    block_matcher_.processDisparity(l_image, r_image, stereo_model_, *disparity_msg);

    pub_disparity_->publish(*disparity_msg);
    // pub_disparity_img_->publish(disparity_msg->image);

    /*************** Point Cloud ***************/
    // Update the camera model
    // model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Calculate point cloud
    const sensor_msgs::msg::Image & dimage = disparity_msg->image;
    // The cv::Mat_ constructor doesn't accept a const data data pointer
    // so we remove the constness before reinterpreting into float.
    // This is "safe" since our cv::Mat is const.
    float * data = reinterpret_cast<float *>(const_cast<uint8_t *>(&dimage.data[0]));

    const cv::Mat_<float> dmat(dimage.height, dimage.width, data, dimage.step);
    stereo_model_.projectDisparityImageTo3d(dmat, points_mat_, true);
    cv::Mat_<cv::Vec3f> mat = points_mat_;

    // Fill in new PointCloud2 message (2D image-like layout)
    auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    points_msg->header = disparity_msg->header;
    points_msg->height = mat.rows;
    points_msg->width = mat.cols;
    points_msg->is_bigendian = false;
    points_msg->is_dense = false;  // there may be invalid points

    sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);

    // Avoidance message initialization 
    auto avoidance_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

    // Target message initialization 
    auto targets_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

    // Pixel message initialization 
    auto pixels_msg = std::make_shared<std_msgs::msg::Int64MultiArray>();

    if (!this->get_parameter("avoid_point_cloud_padding").as_bool()) {
        if (this->get_parameter("use_color").as_bool()) {
        // Data will be packed as (DC=don't care, each item is a float):
        //   x, y, z, DC, rgb, DC, DC, DC
        // Resulting step size: 32 bytes
            pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        } else {
        // Data will be packed as:
        //   x, y, z, DC
        // Resulting step size: 16 bytes
            pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
        }
    } else {
        if (this->get_parameter("use_color").as_bool()) {
        // Data will be packed as:
        //   x, y, z, rgb
        // Resulting step size: 16 bytes
            pcd_modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        } else {
            // Data will be packed as:
            //   x, y, z
            // Resulting step size: 12 bytes
            pcd_modifier.setPointCloud2Fields(
                3,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        }
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");

//position vectors for each object (separated to x,y,z vecs and x,y,z)
    std::vector<float> balloon_x_vec;
    std::vector<float> balloon_y_vec;
    std::vector<float> balloon_z_vec;
    std::vector<float> o_goal_x_vec;
    std::vector<float> o_goal_y_vec;
    std::vector<float> o_goal_z_vec;
    std::vector<float> y_goal_x_vec;
    std::vector<float> y_goal_y_vec;
    std::vector<float> y_goal_z_vec;

//xyz vector of 3 objects, vectors are set to be the size of 3
//initialize as if there is no objects
    std::vector<float> balloon_xyz;
    balloon_xyz.resize(3);
    balloon_xyz[0] = 1000.0;
    balloon_xyz[1] = 1000.0;
    balloon_xyz[2] = 1000.0;
    std::vector<float> o_goal_xyz;
    o_goal_xyz.resize(3);
    o_goal_xyz[0] = 1000.0;
    o_goal_xyz[1] = 1000.0;
    o_goal_xyz[2] = 1000.0;
    std::vector<float> y_goal_xyz;
    y_goal_xyz.resize(3);
    y_goal_xyz[0] = 1000.0;
    y_goal_xyz[1] = 1000.0;
    y_goal_xyz[2] = 1000.0;


//balloon pixel in integers
//initialize as if there is no objects

    std::vector<int64_t> balloon_pixel;
    balloon_pixel.resize(3);
    balloon_pixel[0] = 1000;
    balloon_pixel[1] = 1000;
    balloon_pixel[2] = 0;
    std::vector<int64_t> o_goal_pixel;
    o_goal_pixel.resize(3);
    o_goal_pixel[0] = 1000;
    o_goal_pixel[1] = 1000;
    o_goal_pixel[2] = 0;
    std::vector<int64_t> y_goal_pixel;
    y_goal_pixel.resize(3);
    y_goal_pixel[0] = 1000;
    y_goal_pixel[1] = 1000;
    y_goal_pixel[2] = 0;

// Define the boundaries for the 9 quadrants
    int numRows = mat.rows;
    int numCols = mat.cols;
    int numRowsPerQuadrant = numRows / 3;
    int numColsPerQuadrant = numCols / 3;

// Variables to store the mat(v,u)[2] values for each quadrant
    float quadrantSums[3][3];
    int quadrantCounts[3][3] = {0};

// Set every value to 1000.0f
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            quadrantSums[i][j] = 1000.0f;
        }
    }

    float bad_point = std::numeric_limits<float>::quiet_NaN();
    for (int v = 0; v < mat.rows; ++v) {
        for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z) {
            if (isValidPoint(mat(v, u))) {

                // x,y,z
                *iter_x = mat(v, u)[0];
                *iter_y = mat(v, u)[1];
                *iter_z = mat(v, u)[2];

                // Determine the quadrant
                int quadrantRow = v / numRowsPerQuadrant;
                int quadrantCol = u / numColsPerQuadrant;

                // Add mat(v,u)[2] to the sum of the corresponding quadrant, also record the number of points that are good
                quadrantSums[quadrantRow][quadrantCol] += mat(v, u)[2];
                quadrantCounts[quadrantRow][quadrantCol]++;

                // check if points belong in any of the 3 objects 
                if (isInBoundingBoxBalloon(bbox_msg,u,v)) {
                    // x,y,z
                    balloon_x_vec.push_back(mat(v, u)[0]);
                    balloon_y_vec.push_back(mat(v, u)[1]);
                    balloon_z_vec.push_back(mat(v, u)[2]);
                }

                if (isInBoundingBoxOgoal(bbox_msg,u,v)) {
                    // x,y,z
                    o_goal_x_vec.push_back(mat(v, u)[0]);
                    o_goal_y_vec.push_back(mat(v, u)[1]);
                    o_goal_z_vec.push_back(mat(v, u)[2]);
                }

                if (isInBoundingBoxYgoal(bbox_msg,u,v)) {
                    // x,y,z
                    y_goal_x_vec.push_back(mat(v, u)[0]);
                    y_goal_y_vec.push_back(mat(v, u)[1]);
                    y_goal_z_vec.push_back(mat(v, u)[2]);  
                }

                //else the point is bad
            } else {
                *iter_x = *iter_y = *iter_z = bad_point;
            }
        }
    }

    // Calculate the average for each quadrant
    float quadrantAverages[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (quadrantCounts[i][j] > 0) {
                quadrantAverages[i][j] = quadrantSums[i][j] / quadrantCounts[i][j];
            } else {
                quadrantAverages[i][j] = 1000.0f;
            }
        }
    }

    //check balloon xyz centroid
    //Filter outliers using standard deviation
    if (balloon_x_vec.size() != 0) {
        const float numStdDevs = 1.96; // Adjust as needed

        auto calculateFilteredMean = [&](const std::vector<float>& values) -> float {
            float mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            float sumSqDiff = std::accumulate(values.begin(), values.end(), 0.0,
                [mean](float acc, float value) {
                    float diff = value - mean;
                    return acc + diff * diff;
                });
            float stdDev = std::sqrt(sumSqDiff / values.size());

            std::vector<float> filteredValues;
            std::copy_if(values.begin(), values.end(), std::back_inserter(filteredValues),
                [mean, stdDev, numStdDevs](float value) {
                    return std::abs(value - mean) <= numStdDevs * stdDev;
                });

            return filteredValues.empty() ?
            1000.0 :
            std::accumulate(filteredValues.begin(), filteredValues.end(), 0.0) / filteredValues.size();
        };

        balloon_xyz[0] = calculateFilteredMean(balloon_x_vec);
        balloon_xyz[1] = calculateFilteredMean(balloon_y_vec);
        balloon_xyz[2] = calculateFilteredMean(balloon_z_vec);
    } else {
        balloon_xyz[0] = 1000.0;
        balloon_xyz[1] = 1000.0;
        balloon_xyz[2] = 1000.0;
    }

    if (o_goal_x_vec.size() != 0) {
        const float numStdDevs = 1.96; // Adjust as needed

        auto calculateFilteredMean = [&](const std::vector<float>& values) -> float {
            float mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            float sumSqDiff = std::accumulate(values.begin(), values.end(), 0.0,
                [mean](float acc, float value) {
                    float diff = value - mean;
                    return acc + diff * diff;
                });
            float stdDev = std::sqrt(sumSqDiff / values.size());

            std::vector<float> filteredValues;
            std::copy_if(values.begin(), values.end(), std::back_inserter(filteredValues),
                [mean, stdDev, numStdDevs](float value) {
                    return std::abs(value - mean) <= numStdDevs * stdDev;
                });

            return filteredValues.empty() ?
            1000.0 :
            std::accumulate(filteredValues.begin(), filteredValues.end(), 0.0) / filteredValues.size();
        };

        o_goal_xyz[0] = calculateFilteredMean(o_goal_x_vec);
        o_goal_xyz[1] = calculateFilteredMean(o_goal_y_vec);
        o_goal_xyz[2] = calculateFilteredMean(o_goal_z_vec);
    } else {
        o_goal_xyz[0] = 1000.0;
        o_goal_xyz[1] = 1000.0;
        o_goal_xyz[2] = 1000.0;
    }

    if (y_goal_x_vec.size() != 0) {
        const float numStdDevs = 1.96; // Adjust as needed

        auto calculateFilteredMean = [&](const std::vector<float>& values) -> float {
            float mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            float sumSqDiff = std::accumulate(values.begin(), values.end(), 0.0,
                [mean](float acc, float value) {
                    float diff = value - mean;
                    return acc + diff * diff;
                });
            float stdDev = std::sqrt(sumSqDiff / values.size());

            std::vector<float> filteredValues;
            std::copy_if(values.begin(), values.end(), std::back_inserter(filteredValues),
                [mean, stdDev, numStdDevs](float value) {
                    return std::abs(value - mean) <= numStdDevs * stdDev;
                });

            return filteredValues.empty() ?
            1000.0 :
            std::accumulate(filteredValues.begin(), filteredValues.end(), 0.0) / filteredValues.size();
        };

        y_goal_xyz[0] = calculateFilteredMean(y_goal_x_vec);
        y_goal_xyz[1] = calculateFilteredMean(y_goal_y_vec);
        y_goal_xyz[2] = calculateFilteredMean(y_goal_z_vec);
    } else {
        y_goal_xyz[0] = 1000.0;
        y_goal_xyz[1] = 1000.0;
        y_goal_xyz[2] = 1000.0;
    }

    // static_cast<double>(bbox_msg->x_center_balloon - 1280/2),
    // static_cast<double>(bbox_msg->y_center_balloon - 960/2),
    // static_cast<double>(bbox_msg->x_center_o_goal - 1280/2),
    // static_cast<double>(bbox_msg->y_center_o_goal - 960/2),
    // static_cast<double>(bbox_msg->x_center_y_goal - 1280/2),
    // static_cast<double>(bbox_msg->y_center_y_goal - 960/2),
    //check pixels
    if (bbox_msg->x_center_balloon != -1) {
        balloon_pixel[0] = bbox_msg->x_center_balloon - (1280/2);
        balloon_pixel[1] = bbox_msg->y_center_balloon - (960/2);
        balloon_pixel[2] = (bbox_msg->height_balloon)*(bbox_msg->width_balloon); //pixel area
    } else {
        balloon_pixel[0] = 1000;
        balloon_pixel[1] = 1000;
        balloon_pixel[2] = 0;
    }

    if (bbox_msg->x_center_o_goal != -1){
        o_goal_pixel[0] = bbox_msg->x_center_o_goal - (1280/2);
        o_goal_pixel[1] = bbox_msg->y_center_o_goal - (960/2);
        o_goal_pixel[2] = (bbox_msg->height_o_goal)*(bbox_msg->width_o_goal); //pixel area
    }else{
        o_goal_pixel[0] = 1000;
        o_goal_pixel[1] = 1000;
        o_goal_pixel[2] = 0;
    }

    if (bbox_msg->x_center_y_goal != -1){
        y_goal_pixel[0] = bbox_msg->x_center_y_goal - (1280/2);
        y_goal_pixel[1] = bbox_msg->y_center_y_goal - (960/2);
        y_goal_pixel[2] = (bbox_msg->height_y_goal)*(bbox_msg->width_y_goal); //pixel area
    }else{
        y_goal_pixel[0] = 1000;
        y_goal_pixel[1] = 1000;
        y_goal_pixel[2] = 0;
    }

    //basic averaging working code

    /*
    if (balloon_x_vec.size() != 0){
    float balloon_x_sum = std::accumulate(balloon_x_vec.begin(), balloon_x_vec.end(), 0);
    float balloon_x = balloon_x_sum/balloon_x_vec.size();
    float balloon_y_sum = std::accumulate(balloon_y_vec.begin(), balloon_y_vec.end(), 0);
    float balloon_y = balloon_y_sum/balloon_y_vec.size();
    float balloon_z_sum = std::accumulate(balloon_z_vec.begin(), balloon_z_vec.end(), 0);
    float balloon_z = balloon_z_sum/balloon_z_vec.size();

    balloon_xyz[0] = balloon_x;
    balloon_xyz[1] = balloon_y;
    balloon_xyz[2] = balloon_z;

    }else{
    balloon_xyz[0] = 1000.0;
    balloon_xyz[1] = 1000.0;
    balloon_xyz[2] = 1000.0;
    }

    //check orange goal xyz centroid
    if (o_goal_x_vec.size() != 0){
    float o_goal_x_sum = std::accumulate(o_goal_x_vec.begin(), o_goal_x_vec.end(), 0);
    float o_goal_x = o_goal_x_sum/o_goal_x_vec.size();
    float o_goal_y_sum = std::accumulate(o_goal_y_vec.begin(), o_goal_y_vec.end(), 0);
    float o_goal_y = o_goal_y_sum/o_goal_y_vec.size();
    float o_goal_z_sum = std::accumulate(o_goal_z_vec.begin(), o_goal_z_vec.end(), 0);
    float o_goal_z = o_goal_z_sum/o_goal_z_vec.size();

    o_goal_xyz[0] = o_goal_x;
    o_goal_xyz[1] = o_goal_y;
    o_goal_xyz[2] = o_goal_z;

    }else{
    o_goal_xyz[0] = 1000.0;
    o_goal_xyz[1] = 1000.0;
    o_goal_xyz[2] = 1000.0;
    }

    //check yellow goal xyz centroid
    if (y_goal_x_vec.size() != 0){
    //TODO:filter points through Gaussian fiter etc
    float y_goal_x_sum = std::accumulate(y_goal_x_vec.begin(), y_goal_x_vec.end(), 0);
    float y_goal_x = y_goal_x_sum/y_goal_x_vec.size();
    float y_goal_y_sum = std::accumulate(y_goal_y_vec.begin(), y_goal_y_vec.end(), 0);
    float y_goal_y = y_goal_y_sum/y_goal_y_vec.size();
    float y_goal_z_sum = std::accumulate(y_goal_z_vec.begin(), y_goal_z_vec.end(), 0);
    float y_goal_z = y_goal_z_sum/y_goal_z_vec.size();

    y_goal_xyz[0] = y_goal_x;
    y_goal_xyz[1] = y_goal_y;
    y_goal_xyz[2] = y_goal_z;

    }else{
    y_goal_xyz[0] = 1000.0;
    y_goal_xyz[1] = 1000.0;
    y_goal_xyz[2] = 1000.0;
    }
    */

    // place all avoidance data in one float64multiarray
    //cast to double
    avoidance_msg->data = {
        static_cast<double>(quadrantAverages[0][0]),
        static_cast<double>(quadrantAverages[0][1]),
        static_cast<double>(quadrantAverages[0][2]),
        static_cast<double>(quadrantAverages[1][0]),
        static_cast<double>(quadrantAverages[1][1]),
        static_cast<double>(quadrantAverages[1][2]),
        static_cast<double>(quadrantAverages[2][0]),
        static_cast<double>(quadrantAverages[2][1]),
        static_cast<double>(quadrantAverages[2][2])
    };


    // place all target data in one float64multiarray
    //cast to double
    targets_msg->data = {
        static_cast<double>(balloon_xyz[0]),
        static_cast<double>(balloon_xyz[1]),
        static_cast<double>(balloon_xyz[2]),
        static_cast<double>(o_goal_xyz[0]),
        static_cast<double>(o_goal_xyz[1]),
        static_cast<double>(o_goal_xyz[2]),
        static_cast<double>(y_goal_xyz[0]),
        static_cast<double>(y_goal_xyz[1]),
        static_cast<double>(y_goal_xyz[2])
    };

    //send pixel value (0,0 as the center of the camera frame)
    //cast to double
    pixels_msg->data = {
        balloon_pixel[0],
        balloon_pixel[1],
        balloon_pixel[2],
        o_goal_pixel[0],
        o_goal_pixel[1],
        o_goal_pixel[2],
        y_goal_pixel[0],
        y_goal_pixel[1],
        y_goal_pixel[2]
    };

    if (this->get_parameter("use_color").as_bool()) {
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

        // Fill in color
        namespace enc = sensor_msgs::image_encodings;
        const std::string & encoding = rect_msg_color_left->encoding;
        if (encoding == enc::MONO8) {
            const cv::Mat_<uint8_t> color(
                rect_msg_color_left->height, rect_msg_color_left->width,
                const_cast<uint8_t *>(&rect_msg_color_left->data[0]),
                rect_msg_color_left->step);
            for (int v = 0; v < mat.rows; ++v) {
                for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b) {
                    uint8_t g = color(v, u);
                    *iter_r = *iter_g = *iter_b = g;
                }
            }
        } else if (encoding == enc::RGB8) {
            const cv::Mat_<cv::Vec3b> color(
                rect_msg_color_left->height, rect_msg_color_left->width,
                (cv::Vec3b *)(&rect_msg_color_left->data[0]),
                rect_msg_color_left->step);
            for (int v = 0; v < mat.rows; ++v) {
                for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b) {
                    const cv::Vec3b & rgb = color(v, u);
                    *iter_r = rgb[0];
                    *iter_g = rgb[1];
                    *iter_b = rgb[2];
                }
            }
        } else if (encoding == enc::BGR8) {
            const cv::Mat_<cv::Vec3b> color(
                rect_msg_color_left->height, rect_msg_color_left->width,
                (cv::Vec3b *)(&rect_msg_color_left->data[0]),
                rect_msg_color_left->step);
            for (int v = 0; v < mat.rows; ++v) {
                for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b) {
                    const cv::Vec3b & bgr = color(v, u);
                    *iter_r = bgr[2];
                    *iter_g = bgr[1];
                    *iter_b = bgr[0];
                }
            }
        } else {
            // Throttle duration in milliseconds
            RCUTILS_LOG_WARN_THROTTLE(
                RCUTILS_STEADY_TIME, 30000,
                "Could not fill color channel of the point cloud, "
                "unsupported encoding '%s'", encoding.c_str());
        }
    }

    pub_avoidance_->publish(*avoidance_msg);
    pub_targets_->publish(*targets_msg);
    pub_points2_->publish(*points_msg);
    pub_pixels_->publish(*pixels_msg);
}

rcl_interfaces::msg::SetParametersResult StereoCombined::parameterSetCb(const std::vector<rclcpp::Parameter> & parameters) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
        const std::string param_name = param.get_name();
        if ("stereo_algorithm" == param_name) {
            const int stereo_algorithm_value = param.as_int();
            if (BLOCK_MATCHING == stereo_algorithm_value) {
                block_matcher_.setStereoType(StereoProcessor::BM);
            } else if (SEMI_GLOBAL_BLOCK_MATCHING == stereo_algorithm_value) {
                block_matcher_.setStereoType(StereoProcessor::SGBM);
            } else {
                result.successful = false;
                std::ostringstream oss;
                oss << "Unknown stereo algorithm type '" << stereo_algorithm_value << "'";
                result.reason = oss.str();
            }
        } else if ("prefilter_size" == param_name) {
            block_matcher_.setPreFilterSize(param.as_int());
        } else if ("prefilter_cap" == param_name) {
            block_matcher_.setPreFilterCap(param.as_int());
        } else if ("correlation_window_size" == param_name) {
            block_matcher_.setCorrelationWindowSize(param.as_int());
        } else if ("min_disparity" == param_name) {
            block_matcher_.setMinDisparity(param.as_int());
        } else if ("disparity_range" == param_name) {
            block_matcher_.setDisparityRange(param.as_int());
        } else if ("uniqueness_ratio" == param_name) {
            block_matcher_.setUniquenessRatio(param.as_double());
        } else if ("texture_threshold" == param_name) {
            block_matcher_.setTextureThreshold(param.as_int());
        } else if ("speckle_size" == param_name) {
            block_matcher_.setSpeckleSize(param.as_int());
        } else if ("speckle_range" == param_name) {
            block_matcher_.setSpeckleRange(param.as_int());
        } else if ("full_dp" == param_name) {
            block_matcher_.setSgbmMode(param.as_bool());
        } else if ("P1" == param_name) {
            block_matcher_.setP1(param.as_double());
        } else if ("P2" == param_name) {
            block_matcher_.setP2(param.as_double());
        } else if ("disp12_max_diff" == param_name) {
            block_matcher_.setDisp12MaxDiff(param.as_int());
        }
    }

    return result;
}



