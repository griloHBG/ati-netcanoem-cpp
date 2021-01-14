//
// Created by grilo on 08/01/2021.
//

#ifndef ATI_NETCANOEM_NETCANOEM_FTSENSOR_HPP
#define ATI_NETCANOEM_NETCANOEM_FTSENSOR_HPP


#include <cstdint>  // uint16_t, uint8_t, etc
#include <vector>   // vector
#include <array>    // array
#include <tuple>    // tie
#include <sstream>  // stringstream
#include "utils.hpp"

namespace ati_ftsensor {
    
    class NetCANOEM_FTSensor {

/// Responses are big-endian!
/// https://www.ati-ia.com/app_content/documents/9610-05-1030.pdf
    public:
        
        NetCANOEM_FTSensor() = default;
        
        
        explicit NetCANOEM_FTSensor(uint16_t baseID = 0x7F) : _baseIDRaw(baseID), _baseIDForUse(baseID << 4u) {
        }
        
        inline void setBaseID(uint16_t newBaseID) {
            _baseIDRaw = newBaseID;
            _baseIDForUse = newBaseID << 4u;
        }
        
        CANframe getCANFrameRequest(const OpCode opCode) const {
            CANframe canFrame;
            canFrame.can_dlc = 0;
            canFrame.can_id = _baseIDForUse | opCode;
            
            return canFrame;
        }
        
        ///To get Matrix
        CANframe getCANFrameRequest(const OpCode opCode, const Axis axis) const {
            CANframe canFrame = getCANFrameRequest(opCode);
            canFrame.can_dlc = 1;
            canFrame.data[0] = (int) axis;
            
            return canFrame;
        }
        
        ///To set calibration
        CANframe getCANFrameRequest(const OpCode opCode, const CalibrationIndex calibration) const {
            if (calibration < 0 || calibration > 15) {
                throw std::runtime_error("Calibration must be in the range [0, 15]. Calibration requested:" +
                                         std::to_string(calibration));
            }
            
            CANframe canFrame = getCANFrameRequest(opCode);
            canFrame.can_dlc = 1;
            canFrame.data[0] = calibration;
            
            return canFrame;
        }
        
        ///To get ADC voltage diagnostic
        CANframe getCANFrameRequest(const OpCode opCode, const DiagnosticADCVoltage& diag) const {
            CANframe canFrame = getCANFrameRequest(opCode);
            canFrame.can_dlc = 1;
            canFrame.data[0] = diag;
            
            return canFrame;
        };

//    CANframe getCANFrameRequest(const OpCode opCode, const uint16_t newBaseID) const
//    {
//
//    };

//    enum BaudRate
//    {
//        BaudRate125000,
//        BaudRate250000,
//        BaudRate500000,
//        BaudRate1000000,
//        BaudRate2000000,
//    };
//    CANframe getCANFrameRequest(const OpCode opCode, const uint16_t baudRateDivisor) const
//    {
//
//    };
        
        template<typename Arg, typename T>
        void readCANFrameResponse(const can_frame &canFrame, Arg &&arg);
        
        [[nodiscard]] std::string toSerial(const CANframe &canFrame) const;
        
        [[nodiscard]] FirmwareVersion toFirmwareVersion(const CANframe &canFrame) const {
            if (canFrame.can_id != (_baseIDForUse | OpCode::FirmwareVersion_ans)) {
                throw std::runtime_error("This is not a firmware version answer frame!");
            }
            
            return FirmwareVersion{canFrame.data[0], canFrame.data[1], (canFrame.data[2] << 8u) | canFrame.data[3]};
        }
        
        [[nodiscard]] int toCalibrationIndexSelected(const CANframe &canFrame) const {
            if (canFrame.can_id != (_baseIDForUse | OpCode::SetActiveCalibration_ans)) {
                throw std::runtime_error("This is not a selected calibration index answer frame!");
            }
            return canFrame.data[0];
        }
        
        [[nodiscard]] CountsPerUnit toCountsPerUnit(const CANframe &canFrame) const {
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
        
        ForceTorqueUnit toForceTorqueUnit(const CANframe &canFrame) const {
            if (canFrame.can_id != (_baseIDForUse | OpCode::UnitCodes_ans)) {
                throw std::runtime_error("This is not a force-torque unit answer frame!");
            }
            return ForceTorqueUnit{(ForceUnit) canFrame.data[0], (TorqueUnit) canFrame.data[1]};
        }
        
        int toDiagnosticADCVoltage(const CANframe &canFrame) const {
            if (canFrame.can_id != (_baseIDForUse | OpCode::DiagADCVoltages_ans)) {
                throw std::runtime_error("This is not a DiagADCVoltages_ans answer frame!");
            }
            return ((canFrame.data[0] << 8u) | canFrame.data[1]);
        }
        
        Status toStatus(const CANframe &canFrame) const {
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
        
        std::tuple<float, float> toMatrixElements(const CANframe &canFrame) const {
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
        
        template<typename T>
        std::tuple<T, T, T> toSGData(const CANframe &canFrame) const {
            
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
    
    private:
        uint16_t _baseIDRaw;
        uint16_t _baseIDForUse;
        Status _statusCode;
        
        std::array<int16_t, 6> _sg{};       // Code:    0x0 (status, sg0, sg2, sg4),
        //          0x1 (sg1, sg3, sg5)
        
        //ROW MAJOR!
        std::array<int32_t, 6 * 6> _matrix{}; // Code:    0x2 (SG0, SG1),
                                              //          0x3 (SG2, SG3),
                                              //          0x4 (SG4, SG5)
        
        std::string _serialNumber{};        // Code:    0x5
        
        uint8_t _calibrationIndex{};        // Code:    0x6 (from 0 to 15)
        
        CountsPerUnit _countsPer{};         // Code:    0x7 (counts per force and per
                                            //          torque)
        
        ForceTorqueUnit _unit{};            // Code:    0x8 (force unit,
        TorqueUnit _torqueUnit{};           //          torque unit)
        
        int16_t _mid_vsg{};                 // Code:    0x9 (some diagnostic responses)
        int16_t _thermistor{};              //
        int16_t _power{};                   //
        int16_t _dac{};                     //
        int16_t _ground{};                  //
        
        FirmwareVersion _firmwareVersion{}; // Code:    0xF (firmware version)
        
    };
    
}

#endif //ATI_NETCANOEM_NETCANOEM_FTSENSOR_HPP
