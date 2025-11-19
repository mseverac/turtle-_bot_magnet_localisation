#include <vector>
#include <cstdint>
#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std;
using namespace std::chrono_literals;

class MeasurementNode : public rclcpp::Node
{
public:
    MeasurementNode()
    : Node("measurement_node")
    {
        // Declare parameters
        this->declare_parameter<double>("d0", 0.0);
        this->declare_parameter<double>("z", 0.0);

        // Get parameters
        this->get_parameter("d0", d0_);
        this->get_parameter("z", z_);

        sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "sensor_data", 10, std::bind(&MeasurementNode::sensorDataCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Point>("measurement_results_topic", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&MeasurementNode::timerCallback, this));
    }

private:
    void sensorDataCallback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        rawSensorData = msg->data;
        dataReceived = true;
    }

    void timerCallback()
    {
        if (dataReceived) 
        {
            // Extract measurement
            vector<double> measurements = ExtractMeasurements(rawSensorData, nbReedSensors, magnetDetected);
            cout << "Measurements: ";
            geometry_msgs::msg::Point msg;

            // Output the measurement and publish it
            for (double m : measurements) 
            {
                double transformed_y = 10 * (m - 7.5);   //Offset!
                cout << "(" << d0_ << ", " << transformed_y << ", " << z_ << ") ";
                msg.x = d0_;
                msg.y = transformed_y;
                msg.z = z_;
                pub_->publish(msg);  // Publish the msg into measurement_results_topic
            }
            cout << endl;

            dataReceived = false;  // Restart symbol
        }
    }

    vector<double> ExtractMeasurements(uint16_t rawSensorData, int nbReedSensors, bool magnetDetected)
    {
        cout << "[DEBUG] Enter ExtractMeasurements\n";
        cout << "[DEBUG] rawSensorData=" << rawSensorData
             << " nbReedSensors=" << nbReedSensors
             << " magnetDetected=" << magnetDetected << endl;

        vector<bool> bitVector(nbReedSensors + 2, !magnetDetected);
        for (int i = 0; i < nbReedSensors; ++i)
        {
            bitVector[i + 1] = (rawSensorData & (1u << i)) != 0;  // Extract each bit from raw data
        }

        // Print bitVector for debugging (index 0 .. nbReedSensors+1)
        cout << "[DEBUG] bitVector: ";
        for (size_t j = 0; j < bitVector.size(); ++j)
        {
            cout << (bitVector[j] ? '1' : '0');
            if (j + 1 < bitVector.size()) cout << ",";
        }
        cout << endl;

        int vectSize = nbReedSensors + 2;
        int i = 1;  // start from the first "real" sensor position (index 1 in bitVector)

        // Find the first detected signal
        while (i < vectSize && bitVector[i] == !magnetDetected)
        {
            ++i;
        }
        cout << "[DEBUG] first detected index i=" << i << endl;

        int nbMeasurements = 0;
        vector<double> measurements;

        // Iterate over detected blocks
        while (i < vectSize)
        {
            int indBegin = i;
            cout << "[DEBUG] block start at indBegin=" << indBegin << endl;

            while (i < vectSize && bitVector[i] == magnetDetected)
            {
                ++i;
            } // stop when detection ends

            int indEnd = i - 1;
            cout << "[DEBUG] block end at indEnd=" << indEnd << endl;

            // Calculate midpoint (adjust by -1 because of padding at index 0)
            double midpoint = (indBegin + indEnd) / 2.0 - 1.0;
            cout << "[DEBUG] computed midpoint=" << midpoint << endl;

            measurements.push_back(midpoint);
            nbMeasurements++;

            // Skip the non-detected area until next detected block
            while (i < vectSize && bitVector[i] == !magnetDetected)
            {
                ++i;
            }
        }

        cout << "[DEBUG] Number of measurements: " << nbMeasurements << endl;
        cout << "[DEBUG] Exiting ExtractMeasurements\n";
        return measurements;
    }

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint16_t rawSensorData;
    bool dataReceived = false;
    int nbReedSensors = 16;
    bool magnetDetected = false;
    double d0_;
    double z_;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MeasurementNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


