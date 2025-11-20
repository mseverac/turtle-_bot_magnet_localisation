#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;
using std::string;

class DataReplayNode : public rclcpp::Node
{
public:
    DataReplayNode()
        : Node("data_replay")
    {
        // paramètres (valeurs par défaut calquées sur ton script MATLAB)
        this->declare_parameter<string>("file_path", "twoloops.txt");
        this->declare_parameter<double>("sensorRes", 10.0);
        this->declare_parameter<double>("sensorOffset", 4.5);
        this->declare_parameter<double>("sensorPosAlongXm", 80.0);
        this->declare_parameter<int>("nbReedSensors", 8);
        this->declare_parameter<bool>("magnetDetected", false);

        // optionnels (tu avais d0/z dans ton node original)
        this->declare_parameter<double>("d0", 0.0);
        this->declare_parameter<double>("z", 0.0);

        // récupérer paramètres dans des membres
        this->get_parameter("file_path", file_path_);
        this->get_parameter("sensorRes", sensor_res_);
        this->get_parameter("sensorOffset", sensor_offset_);
        this->get_parameter("sensorPosAlongXm", sensor_pos_xm_);
        this->get_parameter("nbReedSensors", nb_reed_sensors_);
        this->get_parameter("magnetDetected", magnet_detected_);
        this->get_parameter("d0", d0_);
        this->get_parameter("z", z_);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        measurement_pub_ = this->create_publisher<geometry_msgs::msg::Point>("measurement_topic", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&DataReplayNode::timerCallback, this));

        // read lab data files
        readExperimentData(file_path_);
    }

private:
    struct JointStateData
    {
        double position_left;
        double position_right;
        double raw_sensor_data;
    };

    // données
    std::vector<double> time_data_;
    std::vector<JointStateData> joint_states_;
    size_t current_index_ = 0;

    // publishers / timer
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr measurement_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // paramètres stockés
    string file_path_;
    double sensor_res_;
    double sensor_offset_;
    double sensor_pos_xm_;
    int nb_reed_sensors_;
    bool magnet_detected_;
    double d0_;
    double z_;

    void readExperimentData(const string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }

        string line;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            double ql, qr, sensor_data, time;
            if (!(iss >> ql >> qr >> sensor_data >> time))
            {
                break;
            }
            time_data_.push_back(time);

            JointStateData joint_state;
            joint_state.position_left = ql;
            joint_state.position_right = qr;
            joint_state.raw_sensor_data = sensor_data;
            joint_states_.push_back(joint_state);
        }
        file.close();

        // Optional: prepend a few copies of the first sample (comme dans ton code original)
        if (!joint_states_.empty() && !time_data_.empty())
        {
            const size_t repeat_count = 50;
            const auto first_js = joint_states_.front();
            const double first_time = time_data_.front();

            joint_states_.insert(joint_states_.begin(), repeat_count, first_js);
            time_data_.insert(time_data_.begin(), repeat_count, first_time);

            RCLCPP_INFO(this->get_logger(), "Prepended %zu copies of the initial sample", repeat_count);
        }
    }

    void timerCallback()
    {
        if (current_index_ >= joint_states_.size())
        {
            return;
        }

        auto joint_state_msg = sensor_msgs::msg::JointState();
        auto measurement_msg = geometry_msgs::msg::Point();

        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name = {"wheel_left_joint", "wheel_right_joint"};
        // Remplissage sécurisé du vecteur position
        joint_state_msg.position.clear();
        joint_state_msg.position.push_back(joint_states_[current_index_].position_left);
        joint_state_msg.position.push_back(joint_states_[current_index_].position_right);
        joint_state_msg.velocity = {0.0, 0.0};

        uint16_t raw_sensor_value = static_cast<uint16_t>(joint_states_[current_index_].raw_sensor_data);
        std::vector<double> measures = ExtractMeasurements(raw_sensor_value, nb_reed_sensors_, magnet_detected_);

        // Pour chaque mesure trouvée on publie Y tel que dans MATLAB :
        // Y(1) = sensorPosAlongXm
        // Y(2) = sensorRes * ( measure - sensorOffset )
        for (double m : measures)
        {
            measurement_msg.x = sensor_pos_xm_;                         // Y(1)
            measurement_msg.y = sensor_res_ * (m - sensor_offset_);     // Y(2)
            measurement_msg.z = 0.0;                                    // inutilisé / 0
            measurement_pub_->publish(measurement_msg);
            RCLCPP_INFO(this->get_logger(), "Published Y: x=%.3f, y=%.3f", measurement_msg.x, measurement_msg.y);
        }

        joint_state_pub_->publish(joint_state_msg);
        RCLCPP_INFO(this->get_logger(), "Joint State: [%.3f, %.3f]", joint_state_msg.position[0], joint_state_msg.position[1]);

        ++current_index_;
    }

    std::vector<double> ExtractMeasurements(uint16_t rawSensorData, int nbReedSensors, bool magnetDetected)
    {
        // bitVector length = nbReedSensors + 2 ; initial fill with !magnetDetected
        std::vector<bool> bitVector(nbReedSensors + 2, !magnetDetected);
        for (int i = 0; i < nbReedSensors; ++i)
        {
            bitVector[i + 1] = ((rawSensorData & (1u << i)) != 0);
        }

        int vectSize = nbReedSensors + 2;
        int i = 1;

        while (i < vectSize && bitVector[i] == !magnetDetected)
        {
            ++i;
        }

        std::vector<double> measurements;
        while (i < vectSize)
        {
            int indBegin = i;
            while (i < vectSize && bitVector[i] == magnetDetected)
            {
                ++i;
            }
            int indEnd = i - 1;
            double midpoint = (indBegin + indEnd) / 2.0 - 1.0; // comme dans ton code MATLAB
            measurements.push_back(midpoint);

            while (i < vectSize && bitVector[i] == !magnetDetected)
            {
                ++i;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Number of measurements detected: %zu", measurements.size());
        return measurements;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataReplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
