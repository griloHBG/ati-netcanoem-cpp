//
// Created by grilo on 08/01/2021.
//

#include "NetCANOEM_FTSensor.hpp"

namespace ati_ftsensor {
    
    template<typename Arg, typename T>
    void NetCANOEM_FTSensor::readCANFrameResponse(const can_frame &canFrame, Arg &&arg) {
        T ret;
        Axis axis;
        
        switch (canFrame.can_id << 7u) {
            case OpCode::SerialNumber_ans:
                _serialNumber = toSerial(canFrame);
                break;
            case OpCode::FirmwareVersion_ans:
                _firmwareVersion = toFirmwareVersion(canFrame);
                break;
            case OpCode::SetActiveCalibration_ans:
                _calibrationIndex = toCalibrationIndexSelected(canFrame);
                break;
            case OpCode::CountsPerUnits_ans:
                _countsPer = toCountsPerUnit(canFrame);
                break;
            case OpCode::UnitCodes_ans:
                _unit = toForceTorqueUnit(canFrame);
                break;
            case OpCode::DiagADCVoltages_ans:
                
                toDiagnosticADCVoltage(canFrame);
                break;
            case OpCode::SGData_ans1:
                _statusCode = toStatus(canFrame);
                std::tie(_sg[0], _sg[2], _sg[4]) = toSGData<uint16_t>(canFrame);
                break;
            case OpCode::SGData_ans2:
                std::tie(_sg[1], _sg[3], _sg[5]) = toSGData<uint16_t>(canFrame);
                break;
            case OpCode::Matrix_ans1:
            case OpCode::Matrix_ans2:
            case OpCode::Matrix_ans3:
                axis = static_cast<Axis>(arg);
                std::tie(_matrix[6 * (uint8_t) axis + 0], _matrix[6 * (uint8_t) axis + 1]) = toMatrixElements(
                        canFrame);
                break;
        }
    }
    
    [[nodiscard]] std::string NetCANOEM_FTSensor::toSerial(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::SerialNumber_ans)) {
            throw std::runtime_error("This is not a serial number answer frame!");
        }
        
        std::stringstream ssSerial;
        
        for (int i = 0; i < canFrame.can_dlc; i++) {
            ssSerial << canFrame.data[i];
        }
        
        return ssSerial.str();
    }
}
