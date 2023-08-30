#include <memory>

//ros2 tools
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/image_encodings.hpp"

//opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "sim_seg_node/k_means.hpp"
#include "segmentation_msg/msg/segmentation_info.hpp"


K_Means_Node::K_Means_Node(): Node("k_means_node") 
{
    //param
    declare_parameter("class_num",4);
    declare_parameter("mode","k_means");
    declare_parameter("pub_seg_topic","/SegmentationInfo");
    declare_parameter("sub_img_topic","/wamv/sensors/cameras/front_left_camera_sensor/image_raw");

    class_num = get_parameter("class_num").as_int();
    mode = get_parameter("mode").as_string();
    const auto seg_info_topic = get_parameter("pub_seg_topic").as_string();
    const auto camera_info_topic = get_parameter("sub_img_topic").as_string();
    
    k_means_pub_ = create_publisher<segmentation_msg::msg::SegmentationInfo>(seg_info_topic,10);
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(camera_info_topic,10, [this](const sensor_msgs::msg::Image & img){image_callback(img);});
}

K_Means_Node::~K_Means_Node(){}

void K_Means_Node::image_callback(const sensor_msgs::msg::Image & image_msg) {
        cv_bridge::CvImagePtr cv_image_ptr;
        try
        {
            cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            return;
        }

        // Perform k-means clustering
        cv::Mat image = cv_image_ptr->image; 
        cv::imshow("Original Image", image);
        cv::waitKey(1);
        
        if (mode=="k_means"){
            int class_num = 5; // クラスタ数
            cv::Mat reshapedImage = image.reshape(1, image.rows * image.cols); // 画像を1列のデータに変換
            reshapedImage.convertTo(reshapedImage, CV_32F);

            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.2);
            
            //kmeans param
            cv::Mat centers; //centers クラスター番号に対応するRGBカラー
            cv::Mat labels;  // データポイントごとのクラスタ番号が格納される出力配列
            int attempts = 1;             // 異なる初期中心を試す試行回数

            cv::kmeans(reshapedImage, //入力画像(一次元化済の)
                    class_num, //クラスタ数
                    labels, //
                    criteria, // 収束条件を指定するオブジェクト
                    attempts, 
                    cv::KMEANS_RANDOM_CENTERS, // クラスタ中心の初期化方法を指定
                    centers
                    );

            // クラスタ中心の色を使って画像の色を減色
            for (int i = 0; i < reshapedImage.rows; ++i) {
                int clusterIndex = labels.at<int>(i, 0);
                cv::Vec3b& pixel = image.at<cv::Vec3b>(i / image.cols, i % image.cols);
                pixel[0] = centers.at<float>(clusterIndex, 0);
                pixel[1] = centers.at<float>(clusterIndex, 1);
                pixel[2] = centers.at<float>(clusterIndex, 2);    
                //std::cout <<  clusterIndex << std::endl;
            }
            labels = labels.reshape(1,image.rows);
            cv::imshow("seg_img", image);
            cv::waitKey(1);
            cv::Mat indexColorImage = labels.clone();
            indexColorImage.convertTo(indexColorImage, CV_8UC1);
            
            auto msg = std::make_shared<segmentation_msg::msg::SegmentationInfo>();
            auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", indexColorImage).toImageMsg();
            //msg->header
            msg->segmentation.data = image_msg->data;
            msg->segmentation.width = image_msg->width;
            msg->segmentation.height = image_msg->height;
            msg->segmentation.step = image_msg->step;
            msg->segmentation.encoding = image_msg->encoding;
            k_means_pub_->publish(*msg);
        }


}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<K_Means_Node>());
  rclcpp::shutdown();
  return 0;
}

        // // Create an RGB image with the same size as the index color image
        // cv::Mat rgbImage(labels.size(), CV_8UC3);

        // // Define a mapping of index values to corresponding RGB values
        // cv::Scalar colorMap[] = {
        //     cv::Scalar(255, 255, 255),  // Index 0
        //     cv::Scalar(0, 0, 255),      // Index 1
        //     cv::Scalar(0, 255, 0),      // Index 2
        //     cv::Scalar(255, 0, 0),      // Index 3
        //     cv::Scalar(0, 255, 255)     // Index 4
        // };

        // // Convert the index color image to RGB
        // for (int y = 0; y < labels.rows; ++y) {
        //     for (int x = 0; x < labels.cols; ++x) {
        //         uchar index = labels.at<uchar>(y, x);
        //         cv::Vec3b& rgbPixel = rgbImage.at<cv::Vec3b>(y, x);
        //         cv::Scalar color = colorMap[index];
        //         rgbPixel = cv::Vec3b(color[0], color[1], color[2]);
        //     }
        // }

        // // Display the RGB image
        // cv::imshow("RGB Image", rgbImage);
        // cv::waitKey(1);