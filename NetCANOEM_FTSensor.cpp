//
// Created by grilo on 08/01/2021.
//

#include "NetCANOEM_FTSensor.hpp"

namespace ati_ftsensor {
    
    /*template <BaseID baseID, typename Arg, template T>
    void NetCANOEM_FTSensor<baseID>::readCANFrameResponse(const can_frame &canFrame, Arg &&arg) {
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
    }*/
    
    template<BaseID baseID>
    std::string NetCANOEM_FTSensor<baseID>::toSerial(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::SerialNumber_ans)) {
            throw std::runtime_error("This is not a serial number answer frame!");
        }
        
        std::stringstream ssSerial;
        
        for (int i = 0; i < canFrame.can_dlc; i++) {
            ssSerial << canFrame.data[i];
        }
        
        return ssSerial.str();
    }
    
    template<BaseID baseID>
    [[nodiscard]] constexpr FirmwareVersion NetCANOEM_FTSensor<baseID>::toFirmwareVersion(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::FirmwareVersion_ans)) {
            throw std::runtime_error("This is not a firmware version answer frame!");
        }
        
        return FirmwareVersion{canFrame.data[0], canFrame.data[1], (canFrame.data[2] << 8u) | canFrame.data[3]};
    }
    
    template<BaseID baseID>
    [[nodiscard]] constexpr int NetCANOEM_FTSensor<baseID>::toCalibrationIndexSelected(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::SetActiveCalibration_ans)) {
            throw std::runtime_error("This is not a selected calibration index answer frame!");
        }
        return canFrame.data[0];
    }
    
    template<BaseID baseID>
    [[nodiscard]] constexpr CountsPerUnit NetCANOEM_FTSensor<baseID>::toCountsPerUnit(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::CountsPerUnits_ans)) {
            throw std::runtime_error("This is not a counts per unit answer frame!");
        }
        return CountsPerUnit{(float)
                                     ((canFrame.data[0] << 24u) |
                                      (canFrame.data[1] << 16u) |
                                      (canFrame.data[2] << 8u) |
                                      (canFrame.data[3])),
                             (float)
                                     ((canFrame.data[4] << 24u) |
                                      (canFrame.data[5] << 16u) |
                                      (canFrame.data[6] << 8u) |
                                      (canFrame.data[7]))};
    }
    
    template<BaseID baseID>
    constexpr ForceTorqueUnit NetCANOEM_FTSensor<baseID>::toForceTorqueUnit(const CANframe &canFrame) const  {
        if (canFrame.can_id != (_baseIDForUse | OpCode::UnitCodes_ans)) {
            throw std::runtime_error("This is not a force-torque unit answer frame!");
        }
        return ForceTorqueUnit{(ForceUnit) canFrame.data[0], (TorqueUnit) canFrame.data[1]};
    }
    
    template<BaseID baseID>
    constexpr int NetCANOEM_FTSensor<baseID>::toDiagnosticADCVoltage(const CANframe &canFrame) const  {
        if (canFrame.can_id != (_baseIDForUse | OpCode::DiagADCVoltages_ans)) {
            throw std::runtime_error("This is not a DiagADCVoltages_ans answer frame!");
        }
        return ((canFrame.data[0] << 8u) | canFrame.data[1]);
    }
    
    template<BaseID baseID>
    constexpr Status NetCANOEM_FTSensor<baseID>::toStatus(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans1)) {
            throw std::runtime_error(
                    "This is not a SG data first answer frame (this one has the status in its first 2 bytes)!");
        }
    
        unsigned int status = (canFrame.data[0] << 8u) | canFrame.data[1];
    
        return Status{
                static_cast<bool>(status >> 0u & 0x1),
                static_cast<bool>(status >> 1u & 0x1),
                static_cast<bool>(status >> 2u & 0x1),
                static_cast<bool>(status >> 3u & 0x1),
                static_cast<bool>(status >> 4u & 0x1),
                static_cast<bool>(status >> 5u & 0x1),
                static_cast<bool>(status >> 6u & 0x1),
                static_cast<bool>(status >> 7u & 0x1),
                static_cast<bool>(status >> 8u & 0x1),
                //static_cast<bool>(status >> 9u & 0x1),
                //static_cast<bool>(status >> 10u & 0x1),
                static_cast<bool>(status >> 11u & 0x1),
                static_cast<bool>(status >> 12u & 0x1),
                //static_cast<bool>(status >> 13u & 0x1),
                static_cast<bool>(status >> 14u & 0x1),
                static_cast<bool>(status >> 15u),
        
        };
    
    }
    
    template<BaseID baseID>
    constexpr std::tuple<float, float> NetCANOEM_FTSensor<baseID>::toMatrixElements(const CANframe &canFrame) const {
        if (canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans1) &&
            canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans2) &&
            canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans3)) {
            throw std::runtime_error("This is none of the matrix answer (1, 2 nor 3) frame!");
        }
    
        int int0 = canFrame.data[0] << 24u | canFrame.data[1] << 16u | canFrame.data[2] << 8u | canFrame.data[3];
        int int1 = canFrame.data[4] << 24u | canFrame.data[5] << 16u | canFrame.data[6] << 8u | canFrame.data[7];
    
        auto ret = std::make_tuple(reinterpret_cast<float &>(int0), reinterpret_cast<float &>(int1));
    
        return ret;
    }
    
    template<BaseID baseID>
    template<typename T>
    constexpr std::tuple<T, T, T> NetCANOEM_FTSensor<baseID>::toSGData(const CANframe &canFrame) const {
    
        if ((canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans1)) &&
            (canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans2))) {
            throw std::runtime_error("This is not a SG data answer frame!");
        }
    
        std::tuple<T, T, T> ret;
    
        if (canFrame.can_id == (_baseIDForUse | OpCode::SGData_ans1)) {
            //TODO SHOULD BE FLOAT?!
            std::get<0>(ret) = (T) ((canFrame.data[2] << 8u) | canFrame.data[3]);
            std::get<1>(ret) = (T) ((canFrame.data[4] << 8u) | canFrame.data[5]);
            std::get<2>(ret) = (T) ((canFrame.data[6] << 8u) | canFrame.data[7]);
        }
        else if (canFrame.can_id == (_baseIDForUse | OpCode::SGData_ans2)) {
            //TODO SHOULD BE FLOAT?!
            std::get<0>(ret) = (T) ((canFrame.data[0] << 8u) | canFrame.data[1]);
            std::get<1>(ret) = (T) ((canFrame.data[2] << 8u) | canFrame.data[3]);
            std::get<2>(ret) = (T) ((canFrame.data[4] << 8u) | canFrame.data[5]);
        }
    
        return ret;
    }
    
    template<BaseID baseID>
    template<OpCode opCode>
    constexpr CANframe NetCANOEM_FTSensor<baseID>::getCANFrameRequest() const{
        CANframe canFrame;
        canFrame.can_dlc = 0;
        canFrame.can_id = _baseIDForUse | opCode;
    
        return canFrame;
    }
    
    template<BaseID baseID>
    template<OpCode opCode, Axis axis>
    [[nodiscard]] constexpr CANframe  NetCANOEM_FTSensor<baseID>::getCANFrameRequest() const {
            CANframe canFrame = getCANFrameRequest<opCode>();
            canFrame.can_dlc = 1;
            canFrame.data[0] = (int) axis;
            
            return canFrame;
        }
    
    /* {
            if (calibration < 0 || calibration > 15) {
                throw std::runtime_error("Calibration must be in the range [0, 15]. Calibration requested:" +
                                         std::to_string(calibration));
            }
            
            CANframe canFrame = getCANFrameRequest(opCode);
            canFrame.can_dlc = 1;
            canFrame.data[0] = calibration;
            
            return canFrame;
        }*/
    
    /* {
            CANframe canFrame = getCANFrameRequest(opCode);
            canFrame.can_dlc = 1;
            canFrame.data[0] = diag;
            
            return canFrame;
        };*/
    
}
