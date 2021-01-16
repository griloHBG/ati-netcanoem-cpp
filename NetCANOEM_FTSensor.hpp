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
    
    using BaseID = uint16_t;
    
    template<BaseID baseID=0x7F>
    class NetCANOEM_FTSensor {

/// Responses are big-endian!
/// https://www.ati-ia.com/app_content/documents/9610-05-1030.pdf
    public:
        
        constexpr explicit NetCANOEM_FTSensor() : _baseIDRaw(baseID), _baseIDForUse(baseID << 4u) {
        }
        
        /*inline void setBaseID(uint16_t newBaseID) {
            _baseIDForUse = newBaseID << 4u;
        }*/
        
        template<OpCode opCode>
        [[nodiscard]] constexpr CANframe getCANFrameRequest() const;
        
        ///To get Matrix
        template<OpCode opCode, Axis axis>
        [[nodiscard]] constexpr CANframe getCANFrameRequest() const;
        
        ///To set calibration
        template<OpCode opCode, CalibrationIndex calibration>
        [[nodiscard]] constexpr CANframe getCANFrameRequest() const;
        
        ///To get ADC voltage diagnostic
        template<OpCode opCode, DiagnosticADCVoltage& diag>
        [[nodiscard]] constexpr CANframe getCANFrameRequest() const;

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
        void readCANFrameResponse(const can_frame &canFrame, Arg &&arg) = delete;
        
        [[nodiscard]] std::string toSerial(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr FirmwareVersion toFirmwareVersion(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr int toCalibrationIndexSelected(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr CountsPerUnit toCountsPerUnit(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr ForceTorqueUnit toForceTorqueUnit(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr int toDiagnosticADCVoltage(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr Status toStatus(const CANframe &canFrame) const;
        
        [[nodiscard]] constexpr std::tuple<float, float> toMatrixElements(const CANframe &canFrame) const;
        
        template<typename T>
        constexpr std::tuple<T, T, T> toSGData(const CANframe &canFrame) const;
    
    private:
        uint16_t _baseIDRaw;
        uint16_t _baseIDForUse;
        Status _statusCode{};
        
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
