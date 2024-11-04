// Copyright 2024 Zixing Jiang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ndi_hardware/ndi_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

namespace ndi_hardware {

// This class is used to redirect std::cout to /dev/null
// to suppress the cout output from the NDI CAPI library
class CoutRedirect {
  public:
    CoutRedirect()
        : original_cout_buffer(std::cout.rdbuf()), null_stream("/dev/null") {
        std::cout.rdbuf(null_stream.rdbuf());
    }

    ~CoutRedirect() { std::cout.rdbuf(original_cout_buffer); }

  private:
    std::streambuf *original_cout_buffer;
    std::ofstream null_stream;
};

hardware_interface::CallbackReturn NdiTrackerHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {

    CoutRedirect redirect; // Redirect std::cout to /dev/null

    if (hardware_interface::SensorInterface::on_init(info) !=
        CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Initializing ...");

    // Get NDI optical tracker system IP from robot description
    ndi_ip_ = info_.hardware_parameters["ip"];
    RCLCPP_INFO(get_logger(), "Get NDI optical tracking system IP: %s",
                ndi_ip_.c_str());

    // Get NDI trackers from robot description
    for (const auto &sensor : info_.sensors) {
        NdiTrackerInfo tracker_info;
        tracker_info.name = sensor.name;
        tracker_info.srom = sensor.parameters.at("srom").c_str();
        ndi_trackers_info_.push_back(tracker_info);
        RCLCPP_INFO(get_logger(), "Get NDI tracker: %s(%s)",
                    tracker_info.name.c_str(), tracker_info.srom.c_str());
    }
    RCLCPP_INFO(get_logger(), "Total number of NDI trackers: %zu",
                ndi_trackers_info_.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

// /* FOR DEBUG USE */
// hardware_interface::CallbackReturn NdiTrackerHardwareInterface::on_configure(
//     const rclcpp_lifecycle::State &previous_state) {

//     CoutRedirect redirect; // Redirect std::cout to /dev/null

//     // Suppress unused variable warning
//     (void)previous_state;

//     // Set initial value for all state interfaces
//     for (const auto &ndi_tracker : ndi_trackers_info_) {
//         set_state(ndi_tracker.name + "/position.x",
//                   std::numeric_limits<double>::quiet_NaN());
//         set_state(ndi_tracker.name + "/position.y",
//                   std::numeric_limits<double>::quiet_NaN());
//         set_state(ndi_tracker.name + "/position.z",
//                   std::numeric_limits<double>::quiet_NaN());
//         set_state(ndi_tracker.name + "/orientation.x",
//                   std::numeric_limits<double>::quiet_NaN());
//         set_state(ndi_tracker.name + "/orientation.y",
//                   std::numeric_limits<double>::quiet_NaN());
//         set_state(ndi_tracker.name + "/orientation.z",
//                   std::numeric_limits<double>::quiet_NaN());
//         set_state(ndi_tracker.name + "/orientation.w",
//                   std::numeric_limits<double>::quiet_NaN());
//     }
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

hardware_interface::CallbackReturn NdiTrackerHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &previous_state) {

    CoutRedirect redirect; // Redirect std::cout to /dev/null

    // Suppress unused variable warning
    (void)previous_state;

    // Set initial value for all state interfaces
    for (const auto &ndi_tracker : ndi_trackers_info_) {
        set_state(ndi_tracker.name + "/position.x",
                  std::numeric_limits<double>::quiet_NaN());
        set_state(ndi_tracker.name + "/position.y",
                  std::numeric_limits<double>::quiet_NaN());
        set_state(ndi_tracker.name + "/position.z",
                  std::numeric_limits<double>::quiet_NaN());
        set_state(ndi_tracker.name + "/orientation.x",
                  std::numeric_limits<double>::quiet_NaN());
        set_state(ndi_tracker.name + "/orientation.y",
                  std::numeric_limits<double>::quiet_NaN());
        set_state(ndi_tracker.name + "/orientation.z",
                  std::numeric_limits<double>::quiet_NaN());
        set_state(ndi_tracker.name + "/orientation.w",
                  std::numeric_limits<double>::quiet_NaN());
    }

    /* The following logic imitates NDI CAPI sample main.cpp */

    ndi_capi_ = CombinedApi();

    // Connect to NDI optical tracker system using TCP
    RCLCPP_INFO(get_logger(), "Connecting to %s ...", ndi_ip_.c_str());
    if (ndi_capi_.connect(ndi_ip_, Protocol::TCP) != 0) {
        RCLCPP_ERROR(get_logger(), "Connection Failed to host: %s",
                     ndi_ip_.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Connected to host: %s", ndi_ip_.c_str());

    // Wait for a second - needed to support connecting to LEMO Vega
    RCLCPP_INFO(get_logger(), "Waiting for a second ...");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Print the firmware version for debugging purposes
    RCLCPP_INFO(
        get_logger(), "Device firmware version: %s",
        ndi_capi_.getUserParameter("Features.Firmware.Version").c_str());

    // Determine if the connected device supports the BX2 command
    RCLCPP_INFO(get_logger(), "Checking BX2 support ...");
    determineApiSupportForBX2();

    // Initialize the system. This clears all previously loaded tools,
    // unsaved settings etc...
    RCLCPP_INFO(get_logger(), "Initializing NDI system ...");
    onErrorPrintDebugMessage("ndi_capi_.initialize()", ndi_capi_.initialize());

    // Load any passive tool definitions from .rom files
    RCLCPP_INFO(get_logger(), "Loading NDI trackers from .rom files ...");
    // Tools are loaded in the order they are defined in the robot description
    for (auto &ndi_tracker : ndi_trackers_info_) {
        loadTool(ndi_tracker.srom.c_str());
        RCLCPP_INFO(get_logger(), "Loaded: %s(%s)", ndi_tracker.name.c_str(),
                    ndi_tracker.srom.c_str());
    }

    // Initialize and enable loaded tools
    ndi_enabled_tools_ = std::vector<ToolData>();
    initializeAndEnableTools(ndi_enabled_tools_);

    // Start tracking
    RCLCPP_INFO(get_logger(), "Starting tracking ...");
    onErrorPrintDebugMessage("ndi_capi_.startTracking()",
                             ndi_capi_.startTracking());

    return hardware_interface::CallbackReturn::SUCCESS;
}

// /* FOR DEBUG USE */
// hardware_interface::CallbackReturn NdiTrackerHardwareInterface::on_cleanup(
//     const rclcpp_lifecycle::State &previous_state) {

//     // Suppress unused variable warning
//     (void)previous_state;
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

hardware_interface::CallbackReturn NdiTrackerHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {

    // Suppress unused variable warning
    (void)previous_state;

    // Stop tracking
    RCLCPP_INFO(get_logger(), "Stopping tracking ...");
    onErrorPrintDebugMessage("ndi_capi_.stopTracking()",
                             ndi_capi_.stopTracking());
    return hardware_interface::CallbackReturn::SUCCESS;
}

// /* FOR DEBUG USE */
// hardware_interface::return_type
// NdiTrackerHardwareInterface::read(const rclcpp::Time &time,
//                                   const rclcpp::Duration &period) {
//     CoutRedirect redirect; // Redirect std::cout to /dev/null
//     // Suppress unused variable warning
//     (void)time;
//     (void)period;

//     for (const auto &ndi_tracker : ndi_trackers_info_) {
//         set_state(ndi_tracker.name + "/position.x", 1);
//         set_state(ndi_tracker.name + "/position.y", 2);
//         set_state(ndi_tracker.name + "/position.z", 3);
//         set_state(ndi_tracker.name + "/orientation.x", 0);
//         set_state(ndi_tracker.name + "/orientation.y", 0);
//         set_state(ndi_tracker.name + "/orientation.z", 0);
//         set_state(ndi_tracker.name + "/orientation.w", 1);
//     }

//     /* FOR DEBUG USE
//      * -------------
//      * print all ros2_control state interfaces and their values
//      * note that sensor_state_interfaces_ is unordered
//      */
//     // for (const auto &[name, descr] : sensor_state_interfaces_) {
//     //     RCLCPP_INFO(get_logger(), "%s: %f", name.c_str(),
//     get_state(name));
//     // }

//     return hardware_interface::return_type::OK;
// }

hardware_interface::return_type
NdiTrackerHardwareInterface::read(const rclcpp::Time &time,
                                  const rclcpp::Duration &period) {
    CoutRedirect redirect; // Redirect std::cout to /dev/null
    // Suppress unused variable warning
    (void)time;
    (void)period;

    std::vector<ToolData> newToolData;

    try {
        // Try to request new tool data
        newToolData =
            ndi_apiSupportsBX2_
                ? ndi_capi_.getTrackingDataBX2(
                      "--6d=tools --3d=tools --sensor=none --1d=buttons")
                : ndi_capi_.getTrackingDataBX(
                      TrackingReplyOption::TransformData |
                      TrackingReplyOption::AllTransforms);
    } catch (const std::exception &e) {
        // Data requests sometimes throw std::exception
        // I guess we can only live with it
        // Given we are tracking in a high frequency
        // Occasional data loss is not a big problem
        RCLCPP_WARN(get_logger(),
                    "Exception caught when requesting new tracking data: %s",
                    e.what());
        // Returning ERROR will make ros2_control deactivate the hardware
        // To avoid this, we return OK
        return hardware_interface::return_type::OK;
    } catch (...) {
        RCLCPP_ERROR(
            get_logger(),
            "Unknown exception caught when requesting new tracking data");
        return hardware_interface::return_type::ERROR;
    }

    // update enabled tools array with new data
    for (auto &enabled_tool : ndi_enabled_tools_) {
        for (auto &newData : newToolData) {
            if (enabled_tool.transform.toolHandle ==
                newData.transform.toolHandle) {
                // copy the new tool data
                newData.toolInfo =
                    enabled_tool.toolInfo; // keep the serial number
                enabled_tool = newData;    // use the new data
            }
        }
    }

    // write enabled tools' pose in ros2 control state interfaces
    // for tools in ndi_enabled_tools_
    for (const auto &tool : ndi_enabled_tools_) {
        // get sensor name
        std::string sensor_name =
            ndi_trackers_info_[tool.transform.toolHandle - 1].name;
        if (tool.transform.isMissing()) {
            // if transform is missing set all values to NaN
            set_state(sensor_name + "/position.x",
                      std::numeric_limits<double>::quiet_NaN());
            set_state(sensor_name + "/position.y",
                      std::numeric_limits<double>::quiet_NaN());
            set_state(sensor_name + "/position.z",
                      std::numeric_limits<double>::quiet_NaN());
            set_state(sensor_name + "/orientation.x",
                      std::numeric_limits<double>::quiet_NaN());
            set_state(sensor_name + "/orientation.y",
                      std::numeric_limits<double>::quiet_NaN());
            set_state(sensor_name + "/orientation.z",
                      std::numeric_limits<double>::quiet_NaN());
            set_state(sensor_name + "/orientation.w",
                      std::numeric_limits<double>::quiet_NaN());
        } else {
            // if transform is not missing, set values to the tool pose
            set_state(sensor_name + "/position.x",
                      tool.transform.tx / 1000.0); // convert to meters
            set_state(sensor_name + "/position.y",
                      tool.transform.ty / 1000.0); // convert to meters
            set_state(sensor_name + "/position.z",
                      tool.transform.tz / 1000.0); // convert to meters
            set_state(sensor_name + "/orientation.x", tool.transform.qx);
            set_state(sensor_name + "/orientation.y", tool.transform.qy);
            set_state(sensor_name + "/orientation.z", tool.transform.qz);
            set_state(sensor_name + "/orientation.w", tool.transform.q0);
        }
    }

    /* FOR DEBUG USE
     * -------------
     * print all ros2_control state interfaces and their values
     * note that sensor_state_interfaces_ is unordered
     */
    // for (const auto &[name, descr] : sensor_state_interfaces_) {
    //     RCLCPP_INFO(get_logger(), "%s: %f", name.c_str(), get_state(name));
    // }

    return hardware_interface::return_type::OK;
}

/* ------------------- private methods ------------------------- */

void NdiTrackerHardwareInterface::determineApiSupportForBX2() {
    // Lookup the API revision
    std::string response = ndi_capi_.getApiRevision();

    // Refer to the API guide for how to interpret the APIREV response
    char deviceFamily = response[0];
    int majorVersion = ndi_capi_.stringToInt(response.substr(2, 3));

    // As of early 2017, the only NDI device supporting BX2 is the Vega
    // Vega is a Polaris device with API major version 003
    if (deviceFamily == 'G' && majorVersion >= 3) {
        ndi_apiSupportsBX2_ = true;
        RCLCPP_INFO(get_logger(), "BX2 is supported by this device, using BX2");
    } else {
        ndi_apiSupportsBX2_ = false;
        RCLCPP_WARN(get_logger(),
                    "BX2 is not supported by this device, using BX");
    }
}

void NdiTrackerHardwareInterface::onErrorPrintDebugMessage(
    std::string methodName, int errorCode) {
    if (errorCode != 0) {
        RCLCPP_ERROR(get_logger(), "%s failed: %d", methodName.c_str(),
                     errorCode);
    }
}

void NdiTrackerHardwareInterface::loadTool(const char *toolDefinitionFilePath) {
    RCLCPP_INFO(get_logger(), "Loading tool from %s ...",
                toolDefinitionFilePath);
    // Request a port handle to load a passive tool into
    int portHandle = ndi_capi_.portHandleRequest();
    // Load the .rom file using the previously obtained port handle
    ndi_capi_.loadSromToPort(toolDefinitionFilePath, portHandle);
}

void NdiTrackerHardwareInterface::initializeAndEnableTools(
    std::vector<ToolData> &enabledTools) {
    RCLCPP_INFO(get_logger(), "Initializing and enabling tools ...");
    // search for all port handles that are not initialized
    std::vector<PortHandleInfo> portHandles = ndi_capi_.portHandleSearchRequest(
        PortHandleSearchRequestOption::NotInit);
    for (size_t i = 0; i < portHandles.size(); i++) {
        // initialize all uninitialized port handles
        onErrorPrintDebugMessage(
            "ndi_capi_.portHandleInitialize()",
            ndi_capi_.portHandleInitialize(portHandles[i].getPortHandle()));
        // enable all initialized port handles
        onErrorPrintDebugMessage(
            "ndi_capi_.portHandleEnable()",
            ndi_capi_.portHandleEnable(portHandles[i].getPortHandle()));
    }
    // search for all port handles that are enabled
    portHandles = ndi_capi_.portHandleSearchRequest(
        PortHandleSearchRequestOption::Enabled);
    // The vector enabledTools is used to store the enabled tools
    // It has the same order as the trackers are defined in the robot
    // description
    for (size_t i = 0; i < portHandles.size(); i++) {
        // for each port handle, create a new ToolData object and add it to
        // the enabledTools vector
        enabledTools.push_back(ToolData());
        // set toolHandle and toolInfo for the new ToolData object
        // .back() returns a reference to the most recently added element
        enabledTools.back().transform.toolHandle =
            (uint16_t)ndi_capi_.stringToInt(portHandles[i].getPortHandle());
        enabledTools.back().toolInfo =
            getToolInfo(portHandles[i].getPortHandle());
        // print enabled tools
        RCLCPP_INFO(get_logger(), "Enabled tool: ID = %d Serial number = %s",
                    enabledTools.back().transform.toolHandle,
                    enabledTools.back().toolInfo.c_str());
    }
}

std::string NdiTrackerHardwareInterface::getToolInfo(std::string toolHandle) {
    // Get the port handle info from PHINF
    PortHandleInfo info = ndi_capi_.portHandleInfo(toolHandle);
    // Return the ID and SerialNumber the desired string format
    std::string outputString = info.getToolId();
    outputString.append(" s/n:").append(info.getSerialNumber());
    return outputString;
}

} // namespace ndi_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ndi_hardware::NdiTrackerHardwareInterface,
                       hardware_interface::SensorInterface)