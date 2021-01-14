//
// Created by grilo on 08/01/2021.
//

#ifndef ATI_NETCANOEM_UTILS_HPP
#define ATI_NETCANOEM_UTILS_HPP

#include <linux/can.h>
#include <iostream>
#include <iomanip>

namespace ati_ftsensor {
    enum OpCode {
        ///Read SG Data requisition
        SGData_req = 0x0,
        ///Read SG Data answer 1: status code (2 bytes), sg0 (2 bytes), sg2 (2 bytes) and sg4 (2 bytes)
        SGData_ans1 = 0x0,
        ///Read SG Data answer 2: sg1 (2 bytes), sg3 (2 bytes) and sg5 (2 bytes)
        SGData_ans2 = 0x1,
        ///Read Matrix requisition: 1 byte for the axis (0: Fx, 1: Fy, 2: Fz, 3: Tx, 4: Ty, 5: Tz)
        Matrix_req = 0x2,
        ///Read Matrix answer: SG0 (4 bytes) and SG1 (4 bytes)
        Matrix_ans1 = 0x2,
        ///Read Matrix answer: SG2 (4 bytes) and SG3 (4 bytes)
        Matrix_ans2 = 0x3,
        ///Read Matrix answer: SG4 (4 bytes) and SG5 (4 bytes)
        Matrix_ans3 = 0x4,
        ///Read Force/Torque Serial Number requisition
        SerialNumber_req = 0x5,
        ///Read Force/Torque Serial Number answer: 8 bytes with ASCI string serial number
        SerialNumber_ans = 0x5,
        ///Set active calibration requisition
        SetActiveCalibration_req = 0x6,
        ///Set active calibration answer: 1 byte echoing the selected calibration index
        SetActiveCalibration_ans = 0x6,
        ///Read Counts Per Unit requisition
        CountsPerUnits_req = 0x7,
        ///Read Counts Per Unit answer: 4 bytes for counts per force and 4 bytes for counts per torque
        CountsPerUnits_ans = 0x7,
        ///Read Unit Codes requisition
        UnitCodes_req = 0x8,
        ///Read Unit Codes answer:
        /// 1 byte for Force unit code
        ///     lbf     1
        ///     N       2
        ///     Klbf    3
        ///     kN      4
        ///     kgf     5
        ///     gf      6
        /// and 1 byte for torque unit code
        ///     lbf-in  1
        ///     lbf-ft  2
        ///     N-m     3
        ///     N-mm    4
        ///     kgfcm   5
        ///     kN-m    6
        UnitCodes_ans = 0x8,
        ///Read Diagnostic ADC Voltages requisition: 1 byte for diagnostic index (0: MID_VSG, 1: Unused, 2: Thermistor, 3: Power, 4: DAC, 5: Ground)
        DiagADCVoltages_req = 0x9,
        ///Read Diagnostic ADC Voltages answer: 2 bytes with requested ADC diagnostic
        DiagADCVoltages_ans = 0x9,
        ///Reset
        Reset_req = 0xC,
        ///Read Firmware Version requisition
        FirmwareVersion_req = 0xF,
        ///Read Firmware Version answer: 1 byte major version, 1 byte minor version, 2 bytes build number
        FirmwareVersion_ans = 0xF,
    };
    
    ///Status (2 bytes sent in SG Data Requitision?)
    ///Bit  Critical?        Name                                        .Can occur after firmware-upgrade; replace NETCANOEM if this happens during normal operation
    /// 0                    Watchdog Rest                               .
    /// 1                    DAC/ADC check result too high               .
    /// 2      Yes           DAC/ADC check result too high               .
    /// 3      Yes           Artificial analog ground out of range       .
    /// 4      Yes           Power supply too high                       .
    /// 5      Yes           Power supply too low                        .
    /// 6      Yes           Bad active calibration                      .
    /// 7      Yes           EEPROM failure                              .
    /// 8                    Configuration invalid                       .
    /// 9                    Reserved                                    .
    /// 10                   Reserved                                    .
    /// 11     Yes           Sensor temperature too high                 .
    /// 12     Yes           Sensor temperature too low                  .
    /// 13                   Reserved                                    .
    /// 14                   CAN bus error                               .
    /// 15                   Any error causes this bit to turn on        .
    
    enum Axis {
        Fx = 0x0,
        Fy = 0x1,
        Fz = 0x2,
        Tx = 0x3,
        Ty = 0x4,
        Tz = 0x5,
    };
    
    typedef struct {
        int major;
        int minor;
        int buildNumber;
    } FirmwareVersion;
    
    
    typedef struct {
        ///Bit 0
        bool b00WatchDogReset;
        ///Bit 1 - Critical
        bool b01DAC_ADC_tooHigh;
        ///Bit 2 - Critical
        bool b02DAC_ADC_tooLow;
        ///Bit 3 - Critical
        bool b03ArtificialAnalogGroundOutOfRange;
        ///Bit 4 - Critical
        bool b04PowerSupplyHigh;
        ///Bit 5 - Critical
        bool b05PowerSupplyLow;
        ///Bit 6 - Critical
        bool b06BadActiveCalibration;
        ///Bit 7 - Critical
        bool b07EEPROMFailure;
        ///Bit 8
        bool b08ConfigurationInvalid;
        //Bit 9
        //bool Reserved;
        //Bit 10
        //bool Reserved;
        ///Bit 11 - Critical
        bool b11SensorTempHigh;
        ///Bit 12 - Critical
        bool b12SensorTempLow;
        //Bit 13
        //bool Reserved;
        ///Bit 14
        bool b14CANBusError;
        ///Bit 15
        bool b15AnyError;
    } Status;
    
    typedef struct can_frame CANframe;
    typedef struct ifreq InterfReq;
    typedef struct sockaddr_can SockAddrCan;
    typedef struct timeval TimeVal;
    typedef uint16_t CalibrationIndex;
    
    typedef struct {
        float Force;
        float Torque;
    } CountsPerUnit;
    
    enum DiagnosticADCVoltage {
        MID_VSG = 0x0,
        //Unused = 0x1, //?!
        Thermistor = 0x2,
        Power = 0x3,
        DAC = 0x4,
        Ground = 0x5,
    };
    
    enum ForceUnit {
        lbf = 0x1,
        N = 0x2,
        Klbf = 0x3, //or klbf?
        kN = 0x4,
        kgf = 0x5,
        gf = 0x6,
    };
    
    enum TorqueUnit {
        lbf_in = 0x1,
        lbf_ft = 0x2,
        N_m = 0x3,
        N_mm = 0x4,
        kgf_cm = 0x5,
        kN_m = 0x6,
    };
    
    typedef struct {
        ForceUnit Force;
        TorqueUnit Torque;
    } ForceTorqueUnit;
    
    void printCANFrame(const CANframe* frame);
    
    std::string forceUnitToStr(ForceUnit fu);
    
    std::string torqueUnitToStr(TorqueUnit tu);
    
    std::array<float, 6> multiply(const std::array<std::array<float, 6>, 6>& matrix, const std::array<int16_t, 6>& sg);
}

std::ostream& operator<<(std::ostream &os, std::array<std::array<float, 6>, 6> matrix);

std::ostream& operator<<(std::ostream& os, const ati_ftsensor::FirmwareVersion& fwv);

#endif //ATI_NETCANOEM_UTILS_HPP
