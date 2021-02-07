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

#include "NetCANOEM_FTSensor.hpp"

#undef CONNECT_TO_KIVY_INTERFACE
#define CONNECT_TO_KIVY_INTERFACE

using json = nlohmann::json;

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
            case 'p':
                printAll = true;
                break;
            case 'g':
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
    
    ati_ftsensor::SockAddrCan addr;
    
    ati_ftsensor::InterfReq ifr;
    strcpy(ifr.ifr_name, "can1" );
    ioctl(socketCan, SIOCGIFINDEX, &ifr);
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socketCan, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }
    
    ati_ftsensor::NetCANOEM_FTSensor<> helper;
    
    ati_ftsensor::CANframe canFrame;
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::SerialNumber_req>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    std::string serialNumber;
    
    serialNumber = helper.toSerial(canFrame);
    
    std::cout << "serial: " << serialNumber << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::FirmwareVersion_req>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto firmwareVersion = helper.toFirmwareVersion(canFrame);
    
    std::cout << "firmware version: " << firmwareVersion << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::SetActiveCalibration_req, 0>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto calibraiontIndex = helper.toCalibrationIndexSelected(canFrame);
    
    std::cout << "selected calibration index: " << calibraiontIndex << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::CountsPerUnits_req>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto countsPer = helper.toCountsPerUnit(canFrame);
    
    std::cout << "Counts per force: " << countsPer.Force << std::endl;
    std::cout << "Counts per torque: " << countsPer.Torque << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::UnitCodes_req>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto unitCode = helper.toForceTorqueUnit(canFrame);
    
    std::cout << "force unit: " << forceUnitToStr(unitCode.Force) << std::endl;
    std::cout << "torque unit: " << torqueUnitToStr(unitCode.Torque) << std::endl;
    
    std::cout << "Diagnostic ADC voltages:" << std::endl;
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::DiagADCVoltages_req, ati_ftsensor::DiagnosticADCVoltage::MID_VSG>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int mid_vsg_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tMID_VSG voltage (int32)\t\t" << mid_vsg_volt << "V" << std::endl;
    std::cout << "\tMID_VSG voltage (int16)\t\t" << reinterpret_cast<int16_t &>(mid_vsg_volt) << "V" << std::endl;
    std::cout << "\tMID_VSG voltage (uint16)\t\t" << reinterpret_cast<uint16_t &>(mid_vsg_volt) << "V" << std::endl;
    std::cout << "\tMID_VSG voltage (float)\t\t" << reinterpret_cast<float&>(mid_vsg_volt) << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::DiagADCVoltages_req,ati_ftsensor::DiagnosticADCVoltage::Thermistor>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int thermistor_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tThermistor voltage (int32)\t" << thermistor_volt << "V" << std::endl;
    std::cout << "\tThermistor voltage (int16)\t" << reinterpret_cast<int16_t&>(thermistor_volt) << "V" << std::endl;
    std::cout << "\tThermistor voltage (uint16)\t" << reinterpret_cast<uint16_t&>(thermistor_volt) << "V" << std::endl;
    std::cout << "\tThermistor voltage (float)\t" << reinterpret_cast<float&>(thermistor_volt) << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::DiagADCVoltages_req,ati_ftsensor::DiagnosticADCVoltage::Power>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int power_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tPower voltage (int32)\t\t" << power_volt << "V" << std::endl;
    std::cout << "\tPower voltage (int16)\t\t" << reinterpret_cast<int16_t&>(power_volt) << "V" << std::endl;
    std::cout << "\tPower voltage (uint16)\t\t" << reinterpret_cast<uint16_t &>(power_volt) << "V" << std::endl;
    std::cout << "\tPower voltage (float)\t\t" << reinterpret_cast<float&>(power_volt) << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::DiagADCVoltages_req,ati_ftsensor::DiagnosticADCVoltage::DAC>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int dac_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tDAC voltage (int32)\t\t\t" << dac_volt << "V" << std::endl;
    std::cout << "\tDAC voltage (int16)\t\t\t" << reinterpret_cast<int16_t &>(dac_volt) << "V" << std::endl;
    std::cout << "\tDAC voltage (uint16)\t\t\t" << reinterpret_cast<uint16_t &>(dac_volt) << "V" << std::endl;
    std::cout << "\tDAC voltage (float)\t\t\t" << reinterpret_cast<float&>(dac_volt) << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::DiagADCVoltages_req, ati_ftsensor::DiagnosticADCVoltage::Ground>();
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    int ground_volt = helper.toDiagnosticADCVoltage(canFrame);
    
    std::cout << "\tGround voltage (int32)\t\t" << ground_volt << "V" << std::endl;
    std::cout << "\tGround voltage (int16)\t\t" << reinterpret_cast<int16_t&>(ground_volt) << "V" << std::endl;
    std::cout << "\tGround voltage (uint16)\t\t" << reinterpret_cast<u_int16_t &>(ground_volt) << "V" << std::endl;
    std::cout << "\tGround voltage (int32)\t\t" << reinterpret_cast<float&>(ground_volt) << "V" << std::endl;
    
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
        switch(row) {
            case 0:
                canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::Matrix_req,ati_ftsensor::Axis::Fx>();
                break;
            case 1:
                canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::Matrix_req,ati_ftsensor::Axis::Fy>();
                break;
            case 2:
                canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::Matrix_req,ati_ftsensor::Axis::Fz>();
                break;
            case 3:
                canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::Matrix_req,ati_ftsensor::Axis::Tx>();
                break;
            case 4:
                canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::Matrix_req,ati_ftsensor::Axis::Ty>();
                break;
            case 5:
                canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::Matrix_req,ati_ftsensor::Axis::Tz>();
                break;
            default:
                throw std::runtime_error("Axis " + std::to_string(row) + " doesn't exists!");
        }
    
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
    
    struct sockaddr_in serverAddr{}, clientAddr{};
    
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


#ifdef CONNECT_TO_KIVY_INTERFACE
    std::cout << "Server accepting " << inet_ntoa(serverAddr.sin_addr) << " at port " << htons(serverAddr.sin_port) << " waiting connection from client GUI interface." << std::endl << std::endl;
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
    
    ati_ftsensor::Status sensorStatus{};
    
    
    /*begin select setup*/
    
    fd_set rfds;
    ati_ftsensor::TimeVal tv;
    int retval;
    /*end select setup*/
    
    int multiplier = 1000;
    
    while(keepRunning == 1)
    {
    
        canFrame = helper.getCANFrameRequest<ati_ftsensor::OpCode::SGData_req>();
    
        write(socketCan, &canFrame, sizeof(canFrame));
    
        read(socketCan, &canFrame, sizeof(canFrame));
    
        std::tie(sg[0], sg[2], sg[4]) = helper.toSGData<int16_t>(canFrame);
        
        sensorStatus = helper.toStatus(canFrame);
        
        read(socketCan, &canFrame, sizeof(canFrame));
    
        std::tie(sg[1], sg[3], sg[5]) = helper.toSGData<int16_t>(canFrame);
    
        ft = ati_ftsensor::multiply(matrix, sg);
    
        std::cout << "FT: ";
        std::for_each(ft.begin(), ft.end(), [](const float& v) { std::cout << std::setw(10) << std::setprecision(3) << v; });
        //std::cout << "\t\tpower supply low: " << sensorStatus.b05PowerSupplyLow;
        std::cout << std::endl;
        
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
