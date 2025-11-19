#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

using namespace std;

class DataReplayNode : public rclcpp::Node //building the node data_replay
{
public:
    DataReplayNode()
        : Node("data_replay")
    {
        // A parameter for declaring file path
        this->declare_parameter<string>("file_path", "twoloops.txt");
        this->declare_parameter<double>("d0", 0.0);
        this->declare_parameter<double>("z", 0.0);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        measurement_pub_ = this->create_publisher<geometry_msgs::msg::Point>("measurement_topic", 10);
        timer_ = this->create_wall_timer(
            50ms, bind(&DataReplayNode::timerCallback, this));

        // A parameter for getting file path
        string file_path;
        this->get_parameter("file_path", file_path);

        // read lab data files
        readExperimentData(file_path);
    }

private:
    struct JointStateData // joint state message
    {
        double position_left;
        double position_right;
        double raw_sensor_data;
    };

    vector<double> time_data_;
    vector<JointStateData> joint_states_;
    size_t current_index_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr measurement_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void readExperimentData(const string &filename) //function for reading lab data
    {
        ifstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        } //if fail to open the file, return

        string line;
        while (getline(file, line))
        {
            istringstream iss(line);
            double ql, qr, sensor_data, time;
            if (!(iss >> ql >> qr >> sensor_data >> time))
            {
                break;
            } //read the file

            time_data_.push_back(time);

            JointStateData joint_state;
            joint_state.position_left = ql;
            joint_state.position_right = qr;
            joint_state.raw_sensor_data = sensor_data;
            joint_states_.push_back(joint_state);
        } //put the data of the files in the vectors of joint state

        file.close();

        // Skipping motionless parts after reading the file
        static size_t start_index = 0;
        static size_t stop_index = joint_states_.size() - 1;
        while (start_index < joint_states_.size() && 
               abs(joint_states_[start_index].position_left - joint_states_[start_index + 1].position_left) < 1e-5 &&
               abs(joint_states_[start_index].position_right - joint_states_[start_index + 1].position_right) < 1e-5)
        {
            ++start_index;
        }
        while (stop_index > start_index &&
               abs(joint_states_[stop_index].position_left - joint_states_[stop_index - 1].position_left) < 1e-5 &&
               abs(joint_states_[stop_index].position_right - joint_states_[stop_index - 1].position_right) < 1e-5)
        {
            --stop_index;
        }

        vector<JointStateData> filtered_joint_states;
        vector<double> filtered_time_data;
        //for (size_t i = start_index; i <= stop_index; ++i)
        //{
        //    if (joint_states_[i].raw_sensor_data != 255)
        //    {
        //        filtered_joint_states.push_back(joint_states_[i]);
        //        filtered_time_data.push_back(time_data_[i]);
        //    }
        //}

        //joint_states_ = filtered_joint_states;
        //time_data_ = filtered_time_data;
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
        joint_state_msg.position = {joint_states_[current_index_].position_left, joint_states_[current_index_].position_right};
        joint_state_msg.velocity = {0.0, 0.0}; // Initialize velocity to 0

        double raw_sensor_value = joint_states_[current_index_].raw_sensor_data;
        vector<double> measurements = ExtractMeasurements(raw_sensor_value, 8, false);
        
        for (double y : measurements)
        {
            measurement_msg.x = this->get_parameter("d0").as_double();
            measurement_msg.y = 10 * (y - 4.5);
            measurement_msg.z = this->get_parameter("z").as_double();
            measurement_pub_->publish(measurement_msg);
            RCLCPP_INFO(this->get_logger(), "Measurement: x=%.2f, y=%.2f, z=%.2f", measurement_msg.x, measurement_msg.y, measurement_msg.z);
        }

        joint_state_pub_->publish(joint_state_msg);
        RCLCPP_INFO(this->get_logger(), "Joint State: [%.2f, %.2f]", joint_state_msg.position[0], joint_state_msg.position[1]);

        ++current_index_;
    }

    vector<double> ExtractMeasurements(uint16_t rawSensorData, int nbReedSensors, bool magnetDetected)
    {
        vector<bool> bitVector(nbReedSensors + 2, !magnetDetected);
        for (int i = 0; i < nbReedSensors; ++i)
        {
            bitVector[i + 1] = (rawSensorData & (1 << i)) != 0;
        }

        int vectSize = nbReedSensors + 2;
        int i = 1;

        while (i < vectSize && bitVector[i] == !magnetDetected)
        {
            ++i;
        }

        int nbMeasurements = 0;
        vector<double> measurements;

        while (i < vectSize)
        {
            int indBegin = i;

            while (i < vectSize && bitVector[i] == magnetDetected)
            {
                ++i;
            }

            int indEnd = i - 1;
            double midpoint = (indBegin + indEnd) / 2.0 - 1;
            measurements.push_back(midpoint);
            nbMeasurements++;

            while (i < vectSize && bitVector[i] == !magnetDetected)
            {
                ++i;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Number of measurements: %d", nbMeasurements);
        return measurements;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<DataReplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

