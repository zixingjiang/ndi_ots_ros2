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

#ifndef ndi_HARDWARE__NDI_HARDWARE_INTERFACE_HPP_
#define ndi_HARDWARE__NDI_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

// From NDI_CAPI_v1.9.7
#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"

namespace ndi_hardware {


struct NdiTrackerInfo {
    std::string name;
    std::string srom;
};

class NdiTrackerHardwareInterface : public hardware_interface::SensorInterface {

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(NdiTrackerHardwareInterface)

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    /* IP of NDI optical tracking system to connect to */
    std::string ndi_ip_;

    /* list of all NDI trackers interested */
    std::vector<NdiTrackerInfo> ndi_trackers_info_;

    /* NDI Combined API object */
    CombinedApi ndi_capi_;

    /* Determines whether an NDI device supports the BX2 command by looking at
     * the API revision */
    void determineApiSupportForBX2();
    bool ndi_apiSupportsBX2_;

    /* Prints a debug message if a method call failed */
    void onErrorPrintDebugMessage(std::string methodName, int errorCode);

    /* Load a tool from a tool definition file (.rom) */
    void loadTool(const char* toolDefinitionFilePath);

    /* Initialize and enable loaded tools */
    void initializeAndEnableTools(std::vector<ToolData> &enabledTools);
    std::vector<ToolData> ndi_enabled_tools_;

    /* Returns the string: "[tool.id] s/n:[tool.serialNUmber]" used in CSV output*/
    std::string getToolInfo(std::string toolHandle);

};

} // namespace ndi_hardware

#endif // ndi_HARDWARE__NDI_HARDWARE_INTERFACE_HPP_