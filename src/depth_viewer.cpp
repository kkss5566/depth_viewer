#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthViewer
{
public:
    DepthViewer() : nh_("~")
    {
        // 初始化參數
        depth_scale_ = 0.001; // 毫米轉米
        max_depth_ = 3.0;     // 最大深度（米）
        x_width_ = 500;       // 俯視圖寬度
        z_height_ = 300;      // 俯視圖高度
        sample_step_ = 4;     // 採樣步長

        // 相機內參（根據實際相機調整）
        fx_ = 644.9059;
        fy_ = 644.9059;
        cx_ = 640.2595 - 150;
        cy_ = 352.2614;

        // 創建訂閱者
        sub_ = nh_.subscribe("/camera/depth/image_rect_raw", 1, 
                            &DepthViewer::imageCallback, this);
        
        // 創建發布者
    	top_view_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/top_view", 1);
        
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            // 轉換ROS圖像到OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            // 創建深度圖（轉換為米）
            cv::Mat depth_meters;
            cv_ptr->image.convertTo(depth_meters, CV_32F, depth_scale_);
            current_depth_image_ = depth_meters.clone();

            // 創建深度可視化圖像
            cv::Mat depth_colored = createDepthVisualization(depth_meters);
            
            // 創建俯視圖
            cv::Mat top_view = createTopView(depth_meters);

            // 顯示圖像
            cv::imshow("Depth Image", depth_colored);
            cv::imshow("Top View", top_view);
            
             // 將俯視圖轉換為ROS消息並發布
             sensor_msgs::ImagePtr top_view_msg = 
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", top_view).toImageMsg();
             top_view_msg->header.stamp = ros::Time::now();
             top_view_msg->header.frame_id = "camera_frame";  // 設置合適的frame_id
             top_view_pub_.publish(top_view_msg);

            // 設置鼠標回調
            cv::setMouseCallback("Depth Image", 
                               mouseCallbackWrapper, 
                               this);

            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    static void mouseCallbackWrapper(int event, int x, int y, int flags, void* userdata)
    {
        DepthViewer* viewer = static_cast<DepthViewer*>(userdata);
        viewer->mouseCallback(event, x, y);
    }

    cv::Mat createDepthVisualization(const cv::Mat& depth_meters)
    {
        cv::Mat normalized;
        cv::normalize(depth_meters, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::Mat colored;
        cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
        return colored;
    }

    cv::Mat createTopView(const cv::Mat& depth_meters)
    {
        cv::Mat top_view = cv::Mat::zeros(z_height_, x_width_, CV_8UC3);

        for (int y = 0; y < depth_meters.rows; y += sample_step_) {
            for (int x = 0; x < depth_meters.cols; x += sample_step_) {
                float depth = depth_meters.at<float>(y, x);
                
                if (depth > 0 && depth < max_depth_) {
                    // 計算3D點
                    cv::Point3f point3d = deprojectPixelToPoint(x, y, depth);
                    
                    // 轉換到俯視圖座標
                    int X = static_cast<int>(point3d.x * 100 + x_width_ / 2);
                    int Z = z_height_ - static_cast<int>(point3d.z * 100); // 反轉Z軸

                    if (X >= 0 && X < x_width_ && Z >= 0 && Z < z_height_) {
                        // 根據深度設置顏色
                        cv::Vec3b color = getDepthColor(depth);
                        cv::circle(top_view, cv::Point(X, Z), 1, color, -1);
                    }
                }
            }
        }

        // 添加網格線
        int grid_interval = 50;
        for (int i = 0; i < x_width_; i += grid_interval) {
            cv::line(top_view, cv::Point(i, 0), cv::Point(i, z_height_-1), 
                    cv::Scalar(50,50,50), 1);
        }
        for (int i = 0; i < z_height_; i += grid_interval) {
            cv::line(top_view, cv::Point(0, i), cv::Point(x_width_-1, i), 
                    cv::Scalar(50,50,50), 1);
        }

        // 添加中心線
        cv::line(top_view, 
                cv::Point(x_width_/2, 0), 
                cv::Point(x_width_/2, z_height_-1), 
                cv::Scalar(0,255,0), 2);

        // 添加距離標記
        for (int i = 0; i < static_cast<int>(max_depth_); ++i) {
            int y_pos = i * 100;
            if (y_pos >= 0 && y_pos < z_height_) {
                cv::putText(top_view, 
                           std::to_string(i) + "m",
                           cv::Point(x_width_-40, z_height_ - y_pos),
                           cv::FONT_HERSHEY_SIMPLEX, 
                           0.5, 
                           cv::Scalar(255,255,255), 
                           1);
            }
        }

        return top_view;
    }

    cv::Point3f deprojectPixelToPoint(float x, float y, float depth)
    {
        float x_world = (x - cx_) / fx_ * depth;
        float y_world = (y - cy_) / fy_ * depth;
        return cv::Point3f(x_world, y_world, depth);
    }

    cv::Vec3b getDepthColor(float depth)
    {
        float normalized_depth = 1.0f - std::min(depth / max_depth_, 1.0f);
        float hue = 120 + 120 * normalized_depth;
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
        cv::Mat rgb;
        cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
        return rgb.at<cv::Vec3b>(0, 0);
    }

    void mouseCallback(int event, int x, int y)
    {
        if (event == cv::EVENT_MOUSEMOVE && !current_depth_image_.empty()) {
            float depth = current_depth_image_.at<float>(y, x);
            if (depth > 0 && depth < max_depth_) {
                cv::Point3f point3d = deprojectPixelToPoint(x, y, depth);
                
                cv::Mat depth_display = createDepthVisualization(current_depth_image_);
                
                // 顯示距離信息
                char depth_str[100];
                sprintf(depth_str, "Distance: %.3f m", depth);
                cv::putText(depth_display, depth_str,
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                           cv::Scalar(255,255,255), 2);
                
                // 顯示3D座標
                char coord_str[100];
                sprintf(coord_str, "X:%.2f Y:%.2f Z:%.2f",
                        point3d.x, point3d.y, point3d.z);
                cv::putText(depth_display, coord_str,
                           cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1,
                           cv::Scalar(255,255,255), 2);
                
                cv::circle(depth_display, cv::Point(x, y), 4,
                          cv::Scalar(0,255,0), -1);
                
                cv::imshow("Depth Image", depth_display);
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher top_view_pub_;  // 添加發布者
    float depth_scale_;
    float max_depth_;
    int x_width_;
    int z_height_;
    int sample_step_;
    float fx_, fy_, cx_, cy_;
    cv::Mat current_depth_image_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_viewer");
    DepthViewer viewer;
    ros::spin();
    return 0;
}
