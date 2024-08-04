#include "axlr_firmware/axlr_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace axlr_firmware
{
    AXLRInterface::AXLRInterface()
    {

    }

    AXLRInterface::~AXLRInterface()
    {
        if(esp_l.IsOpen())
        {
            try
            {
                esp_l.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("AXLRInterface"),"Could not close serial connection on port: " <<port_);
            }
            
        }
    }

    CallbackReturn AXLRInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }
        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AXLRInterface"),"Could not find any Serial port Aborting...: ");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();
        
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> AXLRInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION, &position_states_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AXLRInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn AXLRInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("AXLRInterface"),"Starting AXLR Hardware.....");
        velocity_commands_ = {0.0,0.0};
        position_states_ = {0.0,0.0};
        velocity_states_ = {0.0,0.0};

        try
        {
            esp_l.Open(port_);
            esp_l.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("AXLRInterface"),"Could not connect to serial port: " <<port_);
            return CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("AXLRInterface"),"Starting AXLR Hardware.....");
        RCLCPP_INFO(rclcpp::get_logger("AXLRInterface"),"AXLR hardware successfully started. Ready to take commands!!!");
        return CallbackReturn::SUCCESS;

    }

    CallbackReturn AXLRInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("AXLRInterface"),"Stopping AXLR Hardware.....");
        
        if(esp_l.IsOpen())
        {
            try
            {
                esp_l.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("AXLRInterface"),"Could not close serial connection on port: " <<port_);
                return CallbackReturn::FAILURE;
            }
        }
    }

    hardware_interface::return_type AXLRInterface::read(const rclcpp::Time &time , const rclcpp::Duration &period)
    {
        if(esp_l.IsDataAvailable())
        {
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string msg;
            esp_l.ReadLine(msg);
            std::stringstream ss(msg);
            std::string res;
            int multiplier = 1;
            while(std::getline(ss, res,','))
            {
                multiplier = res.at(1) == 'p' ? 1:-1;
                if(res.at(0) == 'r')
                {
                    velocity_states_.at(1) = multiplier * std::stod(res.substr(2,res.size())); // i can change to match left and right
                    position_states_.at(1) += velocity_states_.at(1) * dt;
                }
                else if(res.at(0) == 'l')
                {
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2,res.size())); // i can change to match left and right
                    position_states_.at(0) += velocity_states_.at(0) * dt;

                }
            }
            last_run_ = rclcpp::Clock().now();
        }

        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type AXLRInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::stringstream msg_stream;
        char r_wheel_sign = velocity_commands_.at(1) >=0 ? 'p' : 'n'; // i can change to match left and right
        char l_wheel_sign = velocity_commands_.at(0) >=0 ? 'p' : 'n'; // i can change to match left and right
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if(std::abs(velocity_commands_.at(1) < 10.0))
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }

        if(std::abs(velocity_commands_.at(0) < 10.0))
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        msg_stream << std::fixed << std::setprecision(2) << r_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(1)) 
            << ",l" << l_wheel_sign << std::abs(velocity_commands_.at(0)) << ",";

        try
        {
            esp_l.Write(msg_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("AXLRInterface"),"An error occured while sending commands: " 
             << msg_stream.str() << "On the port: " << port_);
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(axlr_firmware::AXLRInterface, hardware_interface::SystemInterface)
