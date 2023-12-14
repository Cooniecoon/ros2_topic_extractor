
#include <iostream>
#include <fstream>
#include <cstdlib>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

class RgbTopicExtractor : public rclcpp::Node
{
public:
    RgbTopicExtractor() : Node("rgb_topic_extractor")
    {
        bag_file_path_ = getParam("bag_file_path",std::string("bagfile"));        
        topic_name_ = getParam("topic_name",std::string("/image_raw"));        
        save_step_size_ = getParam("save_step_size",1);
        show_image_ = getParam("show_image",false);


        save_path_ = topic_name_;
        
        // Iterate over each character in the new string
        for (char& c : save_path_) {if (c == '/') {c = '_';}}
        if(save_path_[0]=='_') {save_path_.erase(0, 1);}
        
        std::string cmd = "mkdir -p " + bag_file_path_+"/"+save_path_;
        const int dir_err = system(cmd.c_str());
        if (-1 == dir_err)
        {
            std::cout << "Error creating directory!" << std::endl;;
            return;
        }
        save_path_ = bag_file_path_+"/"+save_path_;
        std::cout << save_path_+"/metadata.txt" << std::endl;
  	    metadata.open(save_path_+"/metadata.txt", std::ios::trunc);
        reader_.open(bag_file_path_);

    }

    void extract()
    {
        // Open the bag file
        while (reader_.has_next()) {
            const auto bag_message = reader_.read_next();
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            auto topic = bag_message->topic_name;
            if (topic.find(topic_name_) != std::string::npos) {
                sensor_msgs::msg::Image msg;
                rclcpp::Serialization<sensor_msgs::msg::Image> serialization_info;
                serialization_info.deserialize_message(&extracted_serialized_msg, &msg);
                try {
                        if(topic_count_%save_step_size_==0)
                        {
                            cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
                            auto header = msg.header;
                            double timestamp = header.stamp.sec + (header.stamp.nanosec * 1e-9);
                            std::string image_name = std::to_string(timestamp)+".png";
                            std::string image_path = save_path_+"/"+image_name;
                            std::cout << image_name << std::endl;
                            metadata <<image_name << "\n";
                            cv::imwrite(image_path, cv_ptr_->image);
                            if(show_image_) {
                                cv::imshow("rgb", cv_ptr_->image);
                                cv::waitKey(1);
                            }
                        }
                        topic_count_++;
                    }
                catch (cv_bridge::Exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                    return;
                }
            }
        }
        reader_.close();
        metadata.close();
    }

protected:
    std::string bag_file_path_;
    int save_step_size_;
    cv_bridge::CvImageConstPtr cv_ptr_;
    std::string topic_name_;
    std::string save_path_;
    rosbag2_cpp::Reader reader_;
    bool show_image_;
    int topic_count_{0};
	std::ofstream metadata;

    template<typename ParamT>
    ParamT getParam(const std::string param_name, ParamT default_value)
    {
        ParamT output{};
        this->declare_parameter(param_name, default_value);
        this->get_parameter(param_name, output);
        return output;
    }
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RgbTopicExtractor>();
    node->extract();

    rclcpp::shutdown();

	return 0;
}