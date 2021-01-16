//
// Created by grilo on 08/01/2021.
//

#include "utils.hpp"
    
std::ostream &operator<<(std::ostream &os, std::array<std::array<float, 6>, 6> matrix) {
    for (std::array<float, 6> &row : matrix) {
        os << "\t";
        for (float &el : row) {
            os << std::setw(14) << el << "  ";
        }
        
        os << std::endl;
    }
    
    return os;
}

std::ostream& operator<<(std::ostream& os, const ati_ftsensor::FirmwareVersion& fwv) {
    os << "v" << fwv.major << "." << fwv.minor << "." << fwv.buildNumber;
    return os;
}

namespace ati_ftsensor {
    
    void printCANFrame(const CANframe *frame) {
        for (int i = 0; i < frame->can_dlc; i++) {
            std::cout << frame->data[i] << " ";
        }
        
        std::cout << std::endl;
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
    
    std::string torqueUnitToStr(TorqueUnit tu)
    {
        std::string ret;
        
        switch (tu)
        {
            case TorqueUnit::N_m:
                ret = "N_m";
                break;
            case TorqueUnit::N_mm:
                ret = "N_mm";
                break;
            case TorqueUnit::lbf_in:
                ret = "lbf_in";
                break;
            case TorqueUnit::lbf_ft:
                ret = "lbf_ft";
                break;
            case TorqueUnit::kN_m:
                ret = "kN_m";
                break;
            case TorqueUnit::kgf_cm:
                ret = "kgf_cm";
                break;
        }
        
        return ret;
    }
    
    std::string forceUnitToStr(ForceUnit fu)
    {
        std::string ret;
        
        switch (fu)
        {
            case ForceUnit::N:
                ret = "N";
                break;
            case ForceUnit::lbf:
                ret = "lbf";
                break;
            case ForceUnit::kN:
                ret = "kN";
                break;
            case ForceUnit::Klbf:
                ret = "Klbf";
                break;
            case ForceUnit::kgf:
                ret = "kgf";
                break;
            case ForceUnit::gf:
                ret = "gf";
                break;
        }
        
        return ret;
    }
}
