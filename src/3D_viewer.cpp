#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class DepthViewer
{
public:
    DepthViewer() : nh_("~"), viewer_("3D Point Cloud")
    {
        // 初始化參數
        depth_scale_ = 0.001;      // 將深度值從毫米轉換為米
        max_depth_ = 3.0;          // 最大深度（米）
        min_depth_ = 0.6;          // 最小深度（米）
        sample_step_ = 4;          // 採樣步長（像素）
        
        // 新增：每米的像素數設置
        pixels_per_meter_ = 100;    // 每米對應的像素數
        x_width_ = 500;            // 俯視圖的寬度（像素）
        // 根據最大深度動態設置高度
        z_height_ = static_cast<int>(max_depth_ * pixels_per_meter_);

        // 相機內參（根據實際相機調整）
        fx_ = 644.9059;       // 焦距x
        fy_ = 644.9059;       // 焦距y
        cx_ = 640.2595 - 150; // 主點x（偏移150像素）
        cy_ = 352.2614;       // 主點y

        // 創建訂閱者，訂閱深度圖像話題
        sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, 
                            &DepthViewer::imageCallback, this);
        
        // 創建發布者，用於發布生成的上視圖圖像
        top_view_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/top_view", 1);

        // 初始化PCL可視化窗口的背景顏色
        viewer_.setBackgroundColor(0, 0, 0);
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            // 將ROS圖像消息轉換為OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, 
                                         sensor_msgs::image_encodings::TYPE_16UC1);
            
            // 將深度值從無符號16位整數轉換為32位浮點數（米）
            cv::Mat depth_meters;
            cv_ptr->image.convertTo(depth_meters, CV_32F, depth_scale_);
            current_depth_image_ = depth_meters.clone();

            // 創建深度可視化圖像（應用色彩映射）
            cv::Mat depth_colored = createDepthVisualization(depth_meters);

            // 創建俯視圖
            cv::Mat top_view = createTopView(depth_meters, depth_colored);

            // 顯示深度圖和俯視圖
            cv::imshow("Depth Image", depth_colored);
            cv::imshow("Top View", top_view);
            
            // 將俯視圖轉換為ROS消息並發布
            sensor_msgs::ImagePtr top_view_msg = 
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", top_view).toImageMsg();
            top_view_msg->header.stamp = ros::Time::now();
            top_view_msg->header.frame_id = "camera_frame";
            top_view_pub_.publish(top_view_msg);

            // 生成3D點雲並顯示
            createPointCloud(depth_meters, depth_colored);

            // 等待1毫秒，處理UI事件
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    cv::Mat createDepthVisualization(const cv::Mat& depth_meters)
    {
        // 使用固定的深度範圍進行歸一化，避免每一幀的min/max變化
        double alpha = 255.0 / max_depth_; // 固定的縮放因子
        cv::Mat depth_normalized;
        cv::convertScaleAbs(depth_meters, depth_normalized, alpha);

        // 應用Jet色彩映射，將灰度深度圖轉換為彩色圖
        cv::Mat depth_colored;
        cv::applyColorMap(depth_normalized, depth_colored, cv::COLORMAP_JET);
        return depth_colored;
    }

    cv::Mat createTopView(const cv::Mat& depth_meters, const cv::Mat& depth_colored)
    {
        // 初始化一個黑色圖像作為俯視圖
        cv::Mat top_view = cv::Mat::zeros(z_height_, x_width_, CV_8UC3);

        // 遍歷深度圖像，每隔sample_step_像素採樣一次
        for (int y = 0; y < depth_meters.rows; y += sample_step_) {
            for (int x = 0; x < depth_meters.cols; x += sample_step_) {
                float depth = depth_meters.at<float>(y, x);
                
                // 過濾深度值
                if (depth > min_depth_ && depth < max_depth_) {
                    // 將像素點反投影到3D空間
                    cv::Point3f point3d = deprojectPixelToPoint(x, y, depth);
                    
                    // 將3D點轉換為俯視圖中的2D點
                    int X = static_cast<int>(point3d.x * pixels_per_meter_ + x_width_ / 2);
                    int Z = z_height_ - static_cast<int>(point3d.z * pixels_per_meter_);

                    // 確保點在俯視圖範圍內
                    if (X >= 0 && X < x_width_ && Z >= 0 && Z < z_height_) {
                        cv::Vec3b color = depth_colored.at<cv::Vec3b>(y, x);
                        cv::circle(top_view, cv::Point(X, Z), 1, color, -1);
                    }
                }
            }
        }

        // 繪製網格線
        int grid_interval = pixels_per_meter_; // 每米一條線
        for (int i = 0; i < x_width_; i += grid_interval) {
            cv::line(top_view, cv::Point(i, 0), cv::Point(i, z_height_-1), 
                    cv::Scalar(50,50,50), 1);
        }
        for (int i = 0; i < z_height_; i += grid_interval) {
            cv::line(top_view, cv::Point(0, i), cv::Point(x_width_-1, i), 
                    cv::Scalar(50,50,50), 1);
        }

        // 繪製中心線
        cv::line(top_view, 
                cv::Point(x_width_/2, 0), 
                cv::Point(x_width_/2, z_height_-1), 
                cv::Scalar(0,255,0), 2);

        // 繪製深度標籤
        for (int i = 0; i <= static_cast<int>(max_depth_); ++i) {
            int y_pos = i * pixels_per_meter_;
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

    void createPointCloud(const cv::Mat& depth_meters, const cv::Mat& depth_colored)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for (int y = 0; y < depth_meters.rows; y += sample_step_) {
            for (int x = 0; x < depth_meters.cols; x += sample_step_) {
                float depth = depth_meters.at<float>(y, x);
                
                if (depth > min_depth_ && depth < max_depth_) {
                    pcl::PointXYZRGB point;
                    point.x = (x - cx_) / fx_ * depth;
                    point.y = (y - cy_) / fy_ * depth;
                    point.z = depth;

                    cv::Vec3b color = depth_colored.at<cv::Vec3b>(y, x);
                    point.r = color[2];
                    point.g = color[1];
                    point.b = color[0];
                    
                    cloud->points.push_back(point);
                }
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

        viewer_.removeAllPointClouds();
        viewer_.addPointCloud<pcl::PointXYZRGB>(cloud, "depth cloud");
        viewer_.spinOnce();
    }

    cv::Point3f deprojectPixelToPoint(float x, float y, float depth)
    {
        float x_world = (x - cx_) / fx_ * depth;
        float y_world = (y - cy_) / fy_ * depth;
        return cv::Point3f(x_world, y_world, depth);
    }

    // 成員變數
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher top_view_pub_;
    float depth_scale_;
    float max_depth_;
    float min_depth_;
    int x_width_;
    int z_height_;
    int sample_step_;
    int pixels_per_meter_;    // 新增：每米對應的像素數
    float fx_, fy_, cx_, cy_;
    cv::Mat current_depth_image_;
    pcl::visualization::PCLVisualizer viewer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_viewer");
    DepthViewer viewer;
    ros::spin();
    return 0;
}
