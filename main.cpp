#include <linux/can.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <cstring>
#include <iostream>
#include <unistd.h>

#include <sstream>
#include <array>
#include <tuple>
#include <iomanip>
#include <csignal>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <json.hpp>

#include <cstdlib>

#define CONNECT_TO_KIVY_INTERFACE

using json = nlohmann::json;

typedef struct can_frame CANframe;
typedef struct ifreq InterfReq;
typedef struct sockaddr_can SockAddrCan;
typedef struct timeval TimeVal;

void printCANFrame(const CANframe* frame)
{
    for(int i = 0; i < frame->can_dlc; i++)
    {
        std::cout << frame->data[i] << " ";
    }
    
    std::cout << std::endl;
}


/// Responses are big-endian!
/// https://www.ati-ia.com/app_content/documents/9610-05-1030.pdf
class NetCANOEMCANMsg
{
public:
    
    enum OpCode
    {
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
    /// 8                    Caonfiguration invalid                      .
    /// 9                    Reserved                                    .
    /// 10                   Reserved                                    .
    /// 11     Yes           Sensor temperature too high                 .
    /// 12     Yes           Sensor temperature too low                  .
    /// 13                   Reserved                                    .
    /// 14                   CAN bus error                               .
    /// 15                   Any error causes this bit to turn on        .
    
    enum Axis
    {
        Fx = 0x0,
        Fy = 0x1,
        Fz = 0x2,
        Tx = 0x3,
        Ty = 0x4,
        Tz = 0x5,
    };
    
    explicit NetCANOEMCANMsg(uint16_t baseID = 0x20): _baseIDRaw(baseID), _baseIDForUse(baseID << 4u)
    {
    }
    
    inline void setBaseID(uint16_t newBaseID)
    {
        _baseIDRaw = newBaseID;
        _baseIDForUse = newBaseID << 4u;
    }
    
    CANframe getCANFrameRequest(const OpCode opCode) const
    {
        CANframe canFrame;
        canFrame.can_dlc = 0;
        canFrame.can_id = _baseIDForUse | opCode;
        
        return canFrame;
    }
    
    ///To get Matrix
    CANframe getCANFrameRequest(const OpCode opCode, const Axis axis) const
    {
        CANframe canFrame = getCANFrameRequest(opCode);
        canFrame.can_dlc = 1;
        canFrame.data[0] = axis;
        
        return canFrame;
    }
    
    typedef uint16_t CalibrationIndex;
    
    ///To set calibration
    CANframe getCANFrameRequest(const OpCode opCode, const CalibrationIndex calibration) const
    {
        if (calibration < 0 || calibration > 15)
        {
            throw std::runtime_error("Calibration must be in the range [0, 15]. Calibration requested:" + std::to_string(calibration));
        }
        
        CANframe canFrame = getCANFrameRequest(opCode);
        canFrame.can_dlc = 1;
        canFrame.data[0] = calibration;
    
        return canFrame;
    }
    
    enum DiagnosticADCVoltage
    {
        MID_VSG = 0x0,
        //Unused = 0x1, //?!
        Thermistor = 0x2,
        Power = 0x3,
        DAC = 0x4,
        Ground = 0x5,
    };
    
    ///To get ADC voltage diagnostic
    CANframe getCANFrameRequest( const OpCode opCode, const DiagnosticADCVoltage diag) const
    {
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
    
    std::string toSerial(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::SerialNumber_ans))
        {
            throw std::runtime_error("This is not a serial number answer frame!");
        }
        
        std::stringstream ssSerial;
        
        for (int i = 0; i < canFrame.can_dlc; i++)
        {
            ssSerial << canFrame.data[i];
        }
        
        return ssSerial.str();
    }
    
    typedef struct
    {
        int major;
        int minor;
        int buildNumber;
    } FirmwareVersion;
    
    FirmwareVersion toFirmwareVersion(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::FirmwareVersion_ans))
        {
            throw std::runtime_error("This is not a firmware version answer frame!");
        }
        
        return FirmwareVersion{canFrame.data[0], canFrame.data[1], ( canFrame.data[2] << 8u ) | canFrame.data[3]};
    }
    
    int toCalibrationIndexSelected(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::SetActiveCalibration_ans))
        {
            throw std::runtime_error("This is not a selected calibration index answer frame!");
        }
        return canFrame.data[0];
    }
    
    typedef struct
    {
        float Force;
        float Torque;
    } CountsPerUnit;
    
    CountsPerUnit toCountsPerUnit(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::CountsPerUnits_ans))
        {
            throw std::runtime_error("This is not a counts per unit answer frame!");
        }
        return CountsPerUnit{   (float)
                                ((canFrame.data[0] << 24u) |
                                (canFrame.data[1] << 16u) |
                                (canFrame.data[2] << 8u)  |
                                (canFrame.data[3])),
                                (float)
                                ((canFrame.data[4] << 24u) |
                                (canFrame.data[5] << 16u) |
                                (canFrame.data[6] << 8u)  |
                                (canFrame.data[7]))};
    }
    
    enum ForceUnit
    {
        lbf = 0x1,
        N = 0x2,
        Klbf = 0x3, //or klbf?
        kN = 0x4,
        kgf = 0x5,
        gf = 0x6,
    };
    
    enum TorqueUnit
    {
        lbf_in = 0x1,
        lbf_ft = 0x2,
        N_m = 0x3,
        N_mm = 0x4,
        kgf_cm = 0x5,
        kN_m = 0x6,
    };
    
    typedef struct
    {
        ForceUnit Force;
        TorqueUnit Torque;
    } ForceTorqueUnit;
    
    ForceTorqueUnit toForceTorqueUnit(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::UnitCodes_ans))
        {
            throw std::runtime_error("This is not a force-torque unit answer frame!");
        }
        return ForceTorqueUnit {(ForceUnit)canFrame.data[0], (TorqueUnit)canFrame.data[1]};
    }
    
    int toDiagnosticADCVoltage(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::DiagADCVoltages_ans))
        {
            throw std::runtime_error("This is not a firmware version answer frame!");
        }
        return (( canFrame.data[0] << 8u) | canFrame.data[1] );
    }
    
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
    
    Status toStatus(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans1))
        {
            throw std::runtime_error("This is not a SG data first answer frame (this one has the status in its first 2 bytes)!");
        }
        
        unsigned int status = ( canFrame.data[0] << 8u ) | canFrame.data[1];
        
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
    
    std::tuple<float, float> toMatrixElements(const CANframe &canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans1) &&
            canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans2) &&
            canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans3) )
        {
            throw std::runtime_error("This is none of the matrix answer (1, 2 nor 3) frame!");
        }
        
        std::tuple<float, float> ret;
        int int0 = (canFrame.data[0] << 24u | canFrame.data[1] << 16u | canFrame.data[2] << 8u | canFrame.data[3]);
        int int1 = (canFrame.data[4] << 24u | canFrame.data[5] << 16u | canFrame.data[6] << 8u | canFrame.data[7]);
        
        float float0;
        float float1;
        
        memcpy(&float0, &int0, sizeof(float));
        memcpy(&float1, &int1, sizeof(float));
        
        std::get<0>(ret) = float0;
        std::get<1>(ret) = float1;
        
        return ret;
    }
    
    template<typename T>
    std::tuple<T, T, T> toSGData(const CANframe& canFrame) const
    {
        if( (canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans1)) && (canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans2)))
        {
            throw std::runtime_error("This is not a SG data answer frame!");
        }
        
        std::tuple<T, T, T> ret;
        
        if( canFrame.can_id == (_baseIDForUse | OpCode::SGData_ans1) )
        {
            //TODO SHOULD BE FLOAT?!
            std::get<0>(ret) = (T)((canFrame.data[2] << 8u) | canFrame.data[3]);
            std::get<1>(ret) = (T)((canFrame.data[4] << 8u) | canFrame.data[5]);
            std::get<2>(ret) = (T)((canFrame.data[6] << 8u) | canFrame.data[7]);
        }
        else if ( canFrame.can_id == (_baseIDForUse | OpCode::SGData_ans2) )
        {
            //TODO SHOULD BE FLOAT?!
            std::get<0>(ret) = (T)((canFrame.data[0] << 8u) | canFrame.data[1]);
            std::get<1>(ret) = (T)((canFrame.data[2] << 8u) | canFrame.data[3]);
            std::get<2>(ret) = (T)((canFrame.data[4] << 8u) | canFrame.data[5]);
        }
        
        return ret;
    }

private:
    uint16_t _baseIDRaw;
    uint16_t _baseIDForUse;
};

std::ostream &operator<<(std::ostream &os, NetCANOEMCANMsg::FirmwareVersion fwv)
{
    return os << "v" << fwv.major << "." << fwv.minor << "." << fwv.buildNumber;
}

std::ostream &operator<<(std::ostream &os, std::array<std::array<float,6>,6> matrix)
{
    for(std::array<float,6>& row : matrix)
    {
        os << "\t";
        for(float& el : row)
        {
            os << std::setw(14) << el << "  ";
        }
        
        os << std::endl;
    }
    
    return os;
}

std::string forceUnitToStr(NetCANOEMCANMsg::ForceUnit fu)
{
    std::string ret;
    
    switch (fu)
    {
        case NetCANOEMCANMsg::ForceUnit::N:
            ret = "N";
            break;
        case NetCANOEMCANMsg::ForceUnit::lbf:
            ret = "lbf";
            break;
        case NetCANOEMCANMsg::ForceUnit::kN:
            ret = "kN";
            break;
        case NetCANOEMCANMsg::ForceUnit::Klbf:
            ret = "Klbf";
            break;
        case NetCANOEMCANMsg::ForceUnit::kgf:
            ret = "kgf";
            break;
        case NetCANOEMCANMsg::ForceUnit::gf:
            ret = "gf";
            break;
    }
    
    return ret;
}

std::string torqueUnitToStr(NetCANOEMCANMsg::TorqueUnit tu)
{
    std::string ret;
    
    switch (tu)
    {
        case NetCANOEMCANMsg::TorqueUnit::N_m:
            ret = "N_m";
            break;
        case NetCANOEMCANMsg::TorqueUnit::N_mm:
            ret = "N_mm";
            break;
        case NetCANOEMCANMsg::TorqueUnit::lbf_in:
            ret = "lbf_in";
            break;
        case NetCANOEMCANMsg::TorqueUnit::lbf_ft:
            ret = "lbf_ft";
            break;
        case NetCANOEMCANMsg::TorqueUnit::kN_m:
            ret = "kN_m";
            break;
        case NetCANOEMCANMsg::TorqueUnit::kgf_cm:
            ret = "kgf_cm";
            break;
    }
    
    return ret;
}

std::array<float, 6> multiply(const std::array<std::array<float, 6>, 6>& matrix, const std::array<int16_t, 6>& sg)
{
    std::array<float, 6> ret{};
    
    for(int row = 0; row < 6; row++)
    {
        for(int col = 0; col < 6; col++)
        {
            ret[row] += matrix[row][col] * sg[col];
        }
    }
    
    return ret;
}

int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

int main(int argc, char* argv[])
{
    
    bool printAll = false;
    bool waitForGUIConnection = true;
    
    int c;
    
    while( ( c = getopt (argc, argv, "p:g:") ) != -1 )
    {
        switch(c)
        {
            case 'n':
                printAll = true;
                break;
            case 't':
                waitForGUIConnection = true;
                break;
        }
    }
    
    signal(SIGINT, intHandler);
    
    int socketCan;
    
    if ((socketCan = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return 1;
    }
    
    SockAddrCan addr;
    
    InterfReq ifr;
    strcpy(ifr.ifr_name, "can1" );
    ioctl(socketCan, SIOCGIFINDEX, &ifr);
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socketCan, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }
    
    NetCANOEMCANMsg helper;
    
    CANframe canFrame;
    
    std::cout << "asking for transducer calibration matrix" << std::endl;
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SerialNumber_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    std::string serialNumber = "";
    
    serialNumber = helper.toSerial(canFrame);
    
    std::cout << "serial: " << serialNumber << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::FirmwareVersion_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto firmwareVersion = helper.toFirmwareVersion(canFrame);
    
    std::cout << "firmware version: " << firmwareVersion << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SetActiveCalibration_req, 0);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto calibraiontIndex = helper.toCalibrationIndexSelected(canFrame);
    
    std::cout << "selected calibration index: " << calibraiontIndex << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::CountsPerUnits_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto countsPer = helper.toCountsPerUnit(canFrame);
    
    std::cout << "Counts per force: " << countsPer.Force << std::endl;
    std::cout << "Counts per torque: " << countsPer.Torque << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::UnitCodes_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto unitCode = helper.toForceTorqueUnit(canFrame);
    
    std::cout << "force unit: " << forceUnitToStr(unitCode.Force) << std::endl;
    std::cout << "torque unit: " << torqueUnitToStr(unitCode.Torque) << std::endl;
    
    std::cout << "Diagnostic ADC voltages:" << std::endl;
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::MID_VSG);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int mid_vsg_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tMID_VSG voltage\t\t" << mid_vsg_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::Thermistor);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int thermistor_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tThermistor voltage\t" << thermistor_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::Power);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int power_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tPower voltage\t\t" << power_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::DAC);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int dac_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tDAC voltage\t\t\t" << dac_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::Ground);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int ground_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tGround voltage\t\t" << ground_volt << "V" << std::endl;
    
    std::cout << std::endl << std::endl << "\t\tCOMO SE LÊ ESTA CARAIA?!" << std::endl << std::endl << std::endl ;
    
    
    std::cout << "Reading Matrix" << std::endl;
    
    /// [[FxSG0, FxSG1, FxSG2, FxSG3, FxSG4, FxSG5],
    /// [FySG0, FySG1, FySG2, FySG3, FySG4, FySG5],
    /// [FzSG0, FzSG1, FzSG2, FzSG3, FzSG4, FzSG5],
    /// [TxSG0, TxSG1, TxSG2, TxSG3, TxSG4, TxSG5],
    /// [TySG0, TySG1, TySG2, TySG3, TySG4, TySG5],
    /// [TzSG0, TzSG1, TzSG2, TzSG3, TzSG4, TzSG5]]
    std::array<std::array<float, 6>, 6> matrix{};
    
    for(int row = 0; row < 6; row++)
    {
    
        canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::Matrix_req, (NetCANOEMCANMsg::Axis)row);
    
        write(socketCan, &canFrame, sizeof(canFrame));
        
        for(int col = 0; col < 6; col=col+2)
        {
            read(socketCan, &canFrame, sizeof(canFrame));
            
            std::tie(matrix[row][col], matrix[row][col+1]) = helper.toMatrixElements(canFrame);
        }
    }
    
    std::cout << "Matrix: " << std::endl;
    std::cout << matrix << std::endl;
    
    for(int row = 0; row < 3; row++)
    {
        for(int col = 0; col < 6; col++)
        {
            matrix[row][col] *= 1/countsPer.Force;
        }
    }
    
    for(int row = 3; row < 6; row++)
    {
        for(int col = 0; col < 6; col++)
        {
            matrix[row][col] *= 1/countsPer.Torque;
        }
    }
    
    std::cout << "Matrix divided by counts: " << std::endl;
    std::cout << matrix << std::endl;
    
    
    int sockUDPCommunication;
    
    char recvMessage[1024];
    int maxBufferSize = 1024;
    
    struct sockaddr_in serverAddr, clientAddr;
    
    // Creating socket file descriptor
    if ((sockUDPCommunication = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    int option = 1;
    
    setsockopt(sockUDPCommunication, SOL_SOCKET, (SO_REUSEADDR|SO_REUSEPORT), (char*)&option, sizeof(option));
    
    memset(&serverAddr, 0, sizeof(serverAddr));
    memset(&clientAddr, 0, sizeof(clientAddr));
    
    // Filling server information
    serverAddr.sin_family    = AF_INET; // IPv4
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(8080);
    
    if ( bind(sockUDPCommunication, (const struct sockaddr *)&serverAddr,
              sizeof(serverAddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    
    int len, n;
    
    len = sizeof(clientAddr);  //len is value/resuslt
    
    bool ok = true;
    
    
    std::cout << "Server accepting " << inet_ntoa(serverAddr.sin_addr) << " at port " << htons(serverAddr.sin_port) << " waiting connection from client GUI interface." << std::endl << std::endl;
#ifdef CONNECT_TO_KIVY_INTERFACE
    do
    {
        n = recvfrom(sockUDPCommunication, recvMessage, maxBufferSize,
                     MSG_WAITALL, (struct sockaddr *) &clientAddr,
                     reinterpret_cast<socklen_t *>(&len));

        recvMessage[n] = '\0';

        ok = strcmp(recvMessage, "hello ati-netcanoem") == 0;

        if(!ok)
        {
            std::cout << "wrong hand-shake from client!" << std::endl;
        }
    }
    while(!ok);
    
    std::cout << "Client (" << inet_ntoa(clientAddr.sin_addr) << ", port " << htons(clientAddr.sin_port) << ") GUI interface is connected." << std::endl << std::endl;
#endif
    
    json sensorInfoJson;
    
    sensorInfoJson["Type"] = "sensor_info";
    sensorInfoJson["Serial"] = serialNumber;
    sensorInfoJson["Firmware_version"] = {{"major", firmwareVersion.major},
                                          {"minor", firmwareVersion.minor},
                                          {"build", firmwareVersion.buildNumber}};
    sensorInfoJson["Counts_per"] = {{"force", countsPer.Force}, {"torque", countsPer.Torque}};
    sensorInfoJson["Unit"] = {{"force", forceUnitToStr(unitCode.Force)}, {"torque", torqueUnitToStr(unitCode.Torque)}};
    sensorInfoJson["Last_message"] = "False";
    
    char sendMessage[1024];
    strcpy(sendMessage, sensorInfoJson.dump().c_str());
    
    sendto(sockUDPCommunication, (const char *)sendMessage, strlen(sendMessage), MSG_CONFIRM, (const struct sockaddr *) &clientAddr, len);
    
    
    std::array<int16_t, 6> sg{};
    std::array<float, 6> ft{};
    
    int SGcount = 0;
    
    
    
    json ftDataJson;
    ftDataJson["Type"] = "status_force_torque";
    ftDataJson["Status"] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    ftDataJson["Last_message"] = "False";
    ftDataJson["SGcount"] =  SGcount;
    
    NetCANOEMCANMsg::Status sensorStatus{};
    
    
    /*begin select setup*/
    
    fd_set rfds;
    TimeVal tv;
    int retval;
    /*end select setup*/
    
    int multiplier = 1000;
    
    while(keepRunning == 1)
    {
    
        canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SGData_req);
    
        write(socketCan, &canFrame, sizeof(canFrame));
    
        read(socketCan, &canFrame, sizeof(canFrame));
    
        std::tie(sg[0], sg[2], sg[4]) = helper.toSGData<int16_t>(canFrame);
        
        sensorStatus = helper.toStatus(canFrame);
        
        read(socketCan, &canFrame, sizeof(canFrame));
    
        std::tie(sg[1], sg[3], sg[5]) = helper.toSGData<int16_t>(canFrame);
    
        ft = multiply(matrix, sg);
    
        //std::cout << "FT: ";
        //std::for_each(ft.begin(), ft.end(), [](const float& v) { std::cout << std::setw(10) << std::setprecision(3) << v; });
        //std::cout << "\t\tpower supply low: " << sensorStatus.b05PowerSupplyLow;
        //std::cout << std::endl;
        
        ftDataJson["FT"] = {ft[0],
                            ft[1],
                            ft[2],
                            ft[3],
                            ft[4],
                            ft[5]};
        ftDataJson["Status"] = { sensorStatus.b00WatchDogReset,
                                 sensorStatus.b01DAC_ADC_tooHigh,
                                 sensorStatus.b02DAC_ADC_tooLow,
                                 sensorStatus.b03ArtificialAnalogGroundOutOfRange,
                                 sensorStatus.b04PowerSupplyHigh,
                                 sensorStatus.b05PowerSupplyLow,
                                 sensorStatus.b06BadActiveCalibration,
                                 sensorStatus.b07EEPROMFailure,
                                 sensorStatus.b08ConfigurationInvalid,
                                 sensorStatus.b11SensorTempHigh,
                                 sensorStatus.b12SensorTempLow,
                                 sensorStatus.b14CANBusError,
                                 sensorStatus.b15AnyError};
    
        ftDataJson["SGcount"] =  SGcount;
        
        if(ftDataJson.dump().size() > maxBufferSize)
        {
            throw std::runtime_error("json to be sent is bigger than 1024");
        }
        
        strcpy(sendMessage, ftDataJson.dump().c_str());
        
        sendto(sockUDPCommunication, (const char *)sendMessage, strlen(sendMessage), MSG_CONFIRM, (const struct sockaddr *) &clientAddr, len);
    
        /* Watch socketCAN's fd to see when it has input. */
        FD_ZERO(&rfds);
        FD_SET(sockUDPCommunication, &rfds);
    
        /* Wait no seconds! */
        tv.tv_sec = 0;
        tv.tv_usec = 0;
#ifdef CONNECT_TO_KIVY_INTERFACE
        retval = select(sockUDPCommunication+1, &rfds, NULL, NULL, &tv);
        /* Don't rely on the value of tv now! */

        if (retval == -1)
        {
            perror("select()");
        }
        else if (retval)
        {
            n = recvfrom(sockUDPCommunication, recvMessage, maxBufferSize,
                         MSG_WAITALL, (struct sockaddr *) &clientAddr,
                         reinterpret_cast<socklen_t *>(&len));

            recvMessage[n] = '\0';

            keepRunning = strcmp(recvMessage, "end UDP communication") != 0;
            std::cout << "estou saindo!" << std::endl;
        }
#endif
        
        usleep(10000);
        
        SGcount++;
    }
    
    std::cout << "sai do loop!" << std::endl;
    
    ftDataJson["Last_message"] = "True";
    
    strcpy(sendMessage, ftDataJson.dump().c_str());
    
    sendto(sockUDPCommunication, (const char *)sendMessage, strlen(sendMessage), MSG_CONFIRM, (const struct sockaddr *) &clientAddr, len);
    
    //TODO create a bias?!
    
    std::cout << "fim!" << std::endl;
    
    return 0;
}
