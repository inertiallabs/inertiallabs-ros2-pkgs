#include "UDDParser.h"
#include "dataStructures.h"
#include <forward_list>
#include <iostream>
#include <chrono>

using namespace std;

namespace IL {

    UDDParser::UDDParser()
        : KA(4000)
        , KG(10)
        , code(0)
        , payloadLen(0)
        , payloadInd(0)
        , dataSet("")
        , high_precision_heave(false)
    {
    }


    UDDParser::~UDDParser()
    {
    }

    int UDDParser::parse()
    {
        txtStream.str("");
        hdrStream.str("");
        statusStream.str("");
        if (payloadLen <= 2)
            return 1; // ACK message
        payloadInd = 0;
        statusStream << "INS data: ";
        SA = 1e6; SG = 1e5; SO = 1e3; SV = 1e2;
        switch (code)
        {
            using namespace PacketType;
        case IL_IMU_Orientation:
            statusStream << "IMU Orientation";
            dataSet = "\x07\x20\x22\x24\xFF\xFF\x53\x50\x52";
            break;
        case IL_Sensors:
            statusStream << "Sensors";
            dataSet = "\x07\x20\x22\x24\xFF\xFF\x53\x50\x52\x30\x34\x32\x01\xF3\xF0\xF4\x41\xFE";
            break;
        case IL_OPVT:
            statusStream << "OPVT";
            dataSet = "\x07\x20\x22\x24\x53\x50\x52\x10\x12\x30\x32\x01\x36\x3B\xF0\x25\x41";
            break;
        case IL_min:
            statusStream << "Minimal";
            dataSet = "\x07\x53\x50\x52\x10\x12\x01\xF5\x3B";
            break;
        case IL_QPVT:
            statusStream << "QPVT";
            dataSet = "\x09\x20\x22\x24\x53\x50\x52\x10\x12\x30\x32\x01\x36\x3B\xF0\x25\x41";
            break;
        case IL_OPVT2A:
            statusStream << "OPVT2A";
            dataSet = "\x07\x20\x22\x24\x53\x50\x52\x10\x12\x30\x32\x01\x36\x3B\x3D\x3A\xF2\xF1\xF7\x25\x41";
            break;
        case IL_OPVT2AHR:
            statusStream << "OPVT2AHR";
            dataSet = "\x07\x21\x23\x24\x53\x50\x52\x11\x12\x31\x32\x01\x36\x3B"
                      "\x3D\x3A\xF2\xF1\xF7\x25\x41";
            break;
        case MRU_OPVTHSSHR:
            statusStream << "OPVTHSSHR";
            dataSet = "\x08\x21\x23\x24\x53\x50\x52\x11\x12\x13\x16\x15\x18\x31"
                      "\x32\x01\x3c\x36\x3B\x3D\x3A\xF2\xF1\xF7\x25\x41";
            if (high_precision_heave) {
                dataSet[9] = '\x14';
                dataSet[10] = '\x17';
            }
            break;
        case IL_OPVT2AW:
            statusStream << "OPVT2AW";
            dataSet = "\x07\x20\x22\x24\x53\x50\x52\x10\x12\x30\x32\x01\x3C\x36\x3B\xF0\x3A\x33\x35\x25\x41";
            break;
        case IL_OPVTAD:
            statusStream << "OPVTAD";
            dataSet = "\x07\x21\x23\x24\x53\x50\x52\x11\x12\x31\x32\x01\x36\x3B\xF0\x3A\xF2\xF1\x25\x41\x60\x61\x62\x63\x64\x65";
            break;
        case IL_OPVT_rawIMU:
            statusStream << "OPVTRawIMU";
            dataSet = "\x02\x03\x21\x23\x53\x08\x11\x12\x36\x3B\x41";
            SA = SG = 1e4;
            break;
        case IL_OPVT_GNSSext:
            statusStream << "OPVTGNSSExt";
            dataSet = "\x01\x08\x11\x12\x21\x23\x24\xF6\x3D\xF7\x41";
            SO = SA = SG = SV = 1e6;
            break;
        case IL_UDD:
            statusStream << "UDD";
            dataSet.clear();
            dataSet.append(reinterpret_cast<const char*>(&payloadBuf[1]), payloadBuf[0]);
            payloadInd = payloadBuf[0] + 1;
            break;
        case IL_INS_TGA:
            statusStream << "TGA";
            dataSet = "\x21\x23\x03";
            break;
        default:
            statusStream << "0x" << hex << setw(2) << setfill('0') << static_cast<uint32_t>(code);
            dataSet = "";
            break;
        }
//		if (dataSet != oldDataSet)
        {
            writeHeader();
            oldDataSet = dataSet;
        }
        writeTxtAndData();
        hdrString = hdrStream.str();
        txtString = txtStream.str();
        return 0;
    }

void UDDParser::writeHeader()
    {
        for (size_t i = 0; i < dataSet.size(); ++i)
        {
            switch (static_cast<uint8_t>(dataSet[i]))
            {
            case 0x01:
                hdrStream << "ms_gps";
                break;
            case 0x02:
                hdrStream << "GPS_INS_Time";
                break;
            case 0x03:
                hdrStream << "GPS_IMU_Time";
                break;
            case 0x04:
                hdrStream << "UTC_Hour\tUTC_Minute\tUTC_Second\tUTC_DecSec\tUTC_Day\tUTC_Month\tUTC_Year";
                break;
            case 0x07:
            case 0x08:
                hdrStream << "Heading\tPitch\tRoll";
                break;
            case 0x09:
            case 0x0a:
                hdrStream << "QW\tQX\tQY\tQZ";
                break;
            case 0x0d:
                hdrStream << "GridHeading";
                break;
            case 0x0e:
                hdrStream << "Pitch1\tYaw1\tRoll1";
                break;
            case 0x10:
            case 0x11:
                hdrStream << "Latitude\tLongitude\tAltitude";
                break;
            case 0x12:
                hdrStream << "V_East\tV_North\tV_Up";
                break;
            case 0x13:
            case 0x14:
                hdrStream << "Heave";
                break;
            case 0x15:
                hdrStream << "Heave_Velocity";
                break;
            case 0x16:
            case 0x17:
                hdrStream << "Surve\tSway";
                break;
            case 0x18:
                hdrStream << "Surve_Velocity\tSway_Velocity";
                break;
            case 0x19:
                hdrStream << "Significant_Wave_Height";
                break;
            case 0x1B:
                hdrStream << "V_East\tV_North\tV_Up";
                break;
            case 0x1d:
                hdrStream << "V_Right\tV_Forward\tV_Up";
                break;
            case 0x1e:
                hdrStream << "A_East\tA_North\tA_Up";
                break;
            case 0x20:
            case 0x21:
                hdrStream << "GX\tGY\tGZ";
                break;
            case 0x22:
            case 0x23:
                hdrStream << "AX\tAY\tAZ";
                break;
            case 0x24:
                hdrStream << "MX\tMY\tMZ";
                break;
            case 0x25:
                hdrStream << "pBar\thBar";
                break;
            case 0x26:
                hdrStream << "GBX\tGBY\tGBZ\tABX\tABY\tABZ\tBReserved";
                break;
            case 0x27:
                hdrStream << "APVPX\tAPVPY\tAPVPZ";
                break;
            case 0x28:
                hdrStream << "DiffP_ArduPilot";
                break;
            case 0x29:
                hdrStream << "GFX\tGFY\tGFZ";
                break;
            case 0x2a:
                hdrStream << "HF4930Stat";
                break;
            case 0x2b:
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        hdrStream << "TM_c" << r << c;
                        if (!(r == 2 && c == 2)) hdrStream << '\t';
                    }
                }
                break;
            case 0x2c:
                for (int r = 0; r < 3; ++r) {
                    hdrStream << "HM_0" << r;
                    if (r != 2) hdrStream << '\t';
                }
                break;
            case 0x30:
            case 0x31:
                hdrStream << "LatGNSS\tLongGNSS\tAltGNSS";
                break;
            case 0x32:
                hdrStream << "V_Hor\tTrk_gnd\tV_ver";
                break;
            case 0x33:
                hdrStream << "Heading_GNSS\tPitch_GNSS";
                break;
            case 0x34:
                hdrStream << "LatGNSSStd\tLongGNSSStd\tAltGNSSStd";
                break;
            case 0x35:
                hdrStream << "HeadingGNSSStd\tPitchGNSSStd";
                break;
            case 0x36:
                hdrStream << "GNSSInfo1\tGNSSInfo2";
                break;
            case 0x37:
                hdrStream << "SVtrack\tSVsol\tSVsolL1\tSVSolMulti\tGalBD\tGPSGlo\tTimeStatus\tExtSolStatus";
                break;
            case 0x38:
                hdrStream << "GNSSSolStatus";
                break;
            case 0x39:
                hdrStream << "GNSSSolType";
                break;
            case 0x3A:
                hdrStream << "AnglesType";
                break;
            case 0x3B:
                hdrStream << "SVSol";
                break;
            case 0x3C:
                hdrStream << "Week";
                break;
            case 0x3D:
                hdrStream << "GNSSVelLatency";
                break;
            case 0x3E:
                hdrStream << "GNSSPosMs";
                break;
            case 0x3F:
                hdrStream << "GNSSVelMs";
                break;
            case 0x40:
                hdrStream << "GNSSHdgMs";
                break;
            case 0x41:
                hdrStream << "NewGPS";
                break;
            case 0x42:
                hdrStream << "GDOP\tPDOP\tHDOP\tVDOP\tTDOP";
                break;
            case 0x43:
                hdrStream << "GNSS_PACC\tGNSS_VACC";
                break;
            case 0x44:
                hdrStream << "GDOP\tPDOP";
                break;
            case 0x45:
                hdrStream << "Trk_gnd";
                break;
            case 0x47:
                hdrStream << "DiffAge";
                break;
            case 0x48:
                hdrStream << "GNSS_ECEF_VXStsd\tGNSS_ECEF_VYStd\tGNSS_ECEF_VZStd";
                break;
            case 0x49:
                hdrStream << "PPPApp\tPPPStore";
                break;
            case 0x4a:
                hdrStream << "GPSFix\tSpoofingDetect";
                break;
            case 0x4e:
                hdrStream << "GLOGPSTimeOffset";
                break;
            case 0x4f:
                hdrStream << "NewGPS2";
                break;
            case 0x50:
                hdrStream << "VSup";
                break;
            case 0x51:
                hdrStream << "VStab";
                break;
            case 0x52:
                hdrStream << "Temp";
                break;
            case 0x53:
                hdrStream << "USW";
                break;
            case 0x54:
                hdrStream << "INSSolStatus";
                break;
            case 0x55:
                hdrStream << "KFLatStd\tKFLonStd\tKFAltStd";
                break;
            case 0x56:
                hdrStream << "KFHdgStd";
                break;
            case 0x57:
                hdrStream << "KFLatStd\tKFLonStd\tKFAltStd";
                break;
            case 0x58:
                hdrStream << "KFVEStd\tKFVNStd\tKFVUStd";
                break;
            case 0x59:
                hdrStream << "CalcTime";
                break;
            case 0x5a:
                hdrStream << "USW2";
                break;
            case 0x5c:
                hdrStream << "KFPitchCov\tKFRollCov";
                break;
            case 0x5e:
                hdrStream << "HeadingCorrSrc";
                break;
            case 0x5f:
                hdrStream << "LatAcc\tLonAcc\tAltAcc\tVEAcc\tVNAcc\tVUpAcc";
                break;
            case 0x60:
                hdrStream << "Odometer";
                break;
            case 0x61:
                hdrStream << "AirSpeed";
                break;
            case 0x62:
                hdrStream << "WindN\tWindE\tWindNStd\tWindEStd";
                break;
            case 0x63:
                hdrStream << "LatExt\tLonExt\tAltExt\tLatExtStd\tLonExtStd\tAltExtStd\tExtPosLatency";
                break;
            case 0x64:
                hdrStream << "LocLat\tLocLon\tLocAlt\tLocDopplerShift\tLocDopplerShiftStd";
                break;
            case 0x65:
                hdrStream << "NewAiding";
                break;
            case 0x66:
                hdrStream << "HgdExt\tHgdExtStd\tHdgExtLatency";
                break;
            case 0x67:
                hdrStream << "DVLRight\tDVLFwd\tDVLUp\tDVLRightStd\tDVLFwdStd\tDVLUpStd\tDVLLatency\tDVLPressure";
                break;
            case 0x68:
                hdrStream << "GBXExt\tGBYExt\tGBZExt\tABXExt\tABYExt\tABZExt\tBExtReserved";
                break;
            case 0x69:
                hdrStream << "PitchExt\tRollExt";
                break;
            case 0x6A:
                hdrStream << "PriAntRightExt\tPriAntForwardExt\tPriAntUpExt\tSecAntRightExt\tSecAntForwardExt\tSecAntUpExt";
                break;
            case 0x6B:
                hdrStream << "RefAirTemp\tRefAirAltitude\tRefAirPressure";
                break;
            case 0x6c:
                hdrStream << "AltExt\tAltExtStd";
                break;
            case 0x6e:
                hdrStream << "LatExt\tLonExt\tLatExtStd\tLonExtStd\tExtPosLatency";
                break;
            case 0x80:
                hdrStream << "PStat";
                break;
            case 0x81:
                hdrStream << "PDyn";
                break;
            case 0x82:
                hdrStream << "PressureRatio";
                break;
            case 0x83:
                hdrStream << "PressureAlt";
                break;
            case 0x84:
                hdrStream << "BHCP";
                break;
            case 0x85:
                hdrStream << "CAS";
                break;
            case 0x86:
                hdrStream << "TAS";
                break;
            case 0x87:
                hdrStream << "MACH";
                break;
            case 0x88:
                hdrStream << "TrueAOA";
                break;
            case 0x89:
                hdrStream << "RateOfClimb";
                break;
            case 0x8a:
                hdrStream << "ADUWindE\tADUWindN\tReserved";
                break;
            case 0x8b:
                hdrStream << "AirDensity";
                break;
            case 0x8c:
                hdrStream << "OutsideAirTemp";
                break;
            case 0x8d:
                hdrStream << "ADUStatus";
                break;
            case 0x97:
                hdrStream << "GFX\tGFY\tGFZ\tAFX\tAFY\tAFZ";
                break;
            case 0x98:
                hdrStream << "GRX\tGRY\tGRZ";
                break;
            case 0x99:
                hdrStream << "BaroTemp";
                break;
            case 0x9a:
                hdrStream << "OTFMagCal";
                break;
            case 0xA1:
                hdrStream << "NewAiding2";
                break;
            case 0xA2:
                hdrStream << "PStat\tPDyn";
                break;
            case 0xA3:
                hdrStream << "AirSpeed2D\tAirDir2D";
                break;
            case 0xA4:
                hdrStream << "BSsol";
                break;
            case 0xA5:
            case 0xA6:
            case 0xA7:
            case 0xA8:
            case 0xA9:
            case 0xAA:
            case 0xAB:
            case 0xAC:
            case 0xAD:
            case 0xAE:
                hdrStream << "BSLat\tBSLon\tBSAlt\tBSRange\tBSRangeStd\tBSLatency";
                break;
            case 0xaf:
                hdrStream << "UAVCANDiffPressure\tUAVCANTemp";
                break;
            case 0xb0:
                hdrStream << "ECEF_X\tECEF_Y\tECEF_Z";
                break;
            case 0xb1:
                hdrStream << "VECEF_X\tVECEF_Y\tVECEF_Z";
                break;
            case 0xb2:
                hdrStream << "Easting\tNorthing\tZone";
                break;
            case 0xc0:
                hdrStream << "ublox_status";
                break;
            case 0xc1:
                hdrStream << "NovAtelReceiverStatus";
                break;
            case 0xc2:
                hdrStream << "ACHReceiverStatus";
                break;
            case 0xc4:
                hdrStream << "GNSSPosLat\tGNSSVelLat";
                break;
            case 0xd0:
                hdrStream << "AltStatus";
                break;
            case 0xd1:
                hdrStream << "MagDecl";
                break;
            case 0xd2:
                hdrStream << "KFOdoYaw\tKFOdoPitch\tKFOdoScale";
                break;
            case 0xd3:
                hdrStream << "KFDVLYaw\tKFDVLPitch\tKFDVLRoll\tKFDVLScale";
                break;
            case 0xd4:
                hdrStream << "KFTunnelGuideYaw\tKFTunnelGuideScale";
                break;
            case 0xd5:
                hdrStream << "ExtendedCmdCode\tExtendedCmdVal\tExtendedCmdReserved0\tExtendedCmdReserved1";
                break;
            case 0xF0:
            case 0xF7:
                hdrStream << "Latency_ms_pos\tLatency_ms_vel";
                break;
            case 0xF1:
                hdrStream << "Latency_ms_head";
                break;
            case 0xF2:
                hdrStream << "HeadingGNSS";
                break;
            case 0xF3:
                hdrStream << "TimeStatus\tGNSSSolStatus\tGNSSPosType\tSVtrack\tSVsol\tSVsolL1\tSVSolMulti\tExtSolStatus\tGalBD\tGPSGlo";
                break;
            case 0xF4:
                hdrStream << "UP\tUT";
                break;
            case 0xF5:
                hdrStream << "GNSSInfo1";
                break;
            case 0xF6:
                hdrStream << "pBar\tTemp\tUSW\tPosType\tGNSS_ECEF_X\tGNSS_ECEF_Y\tGNSS_ECEF_Z\tGNSS_PACC\tGNSS_ECEF_VX\tGNSS_ECEF_VY\tGNSS_ECEF_VZ\tGNSS_SACC\
\tLatGNSS\tLongGNSS\tAltGNSS\tV_Hor\tTrk_gnd\tV_ver\tAnglesType\tHeadingGNSS\tSVSol\tGNSSInfo1\tGNSSInfo2\tGDOP\tPDOP\tHDOP\tVDOP\tTDOP\tDiffAge\
\tUTC_Hour\tUTC_Minute\tUTC_Second\tUTC_DecSec\tUTC_Day\tUTC_Month\tUTC_Year\tUTC\tLatencyECEF";
                break;
            case 0xFE:
            case 0xFF:
                hdrStream << "Reserved";
                break;
            default:
                break;
            }
            if (i < dataSet.size() - 1)
                hdrStream << "\t";
        }
        hdrStream << "\n";
    }

    void UDDParser::writeTxtAndData()
    {
        for (size_t i = 0; i < dataSet.size(); ++i)
        {
            const auto udd_id = static_cast<uint8_t>(dataSet[i]);
            switch (udd_id)
            {
            case 0x01:
                outData.ms_gps = readScaled<uint32_t>(1, false);
                break;
            case 0x02:
                outData.GPS_INS_Time = readScaled<uint64_t>(1e9, false);
                break;
            case 0x03:
                outData.GPS_IMU_Time = readScaled<uint64_t>(1e9, false);
                break;
            case 0x04:
                outData.UTC_Hour = readScaled<uint8_t>(1, true);
                outData.UTC_Minute = readScaled<uint8_t>(1, true);
                outData.UTC_Second = readScaled<uint8_t>(1, true);
                outData.UTC_DecSec = readScaled<uint16_t>(1e3, true);
                outData.UTC_Day = readScaled<uint8_t>(1, true);
                outData.UTC_Month = readScaled<uint8_t>(1, true);
                outData.UTC_Year = readScaled<uint16_t>(1, false);
                break;
            case 0x07:
                outData.Heading = readScaled<uint16_t>(100, true);
                outData.Pitch = readScaled<int16_t>(100, true);
                outData.Roll = readScaled<int16_t>(100, false);
                break;
            case 0x08:
                outData.Heading = readScaled<uint32_t>(SO, true);
                outData.Pitch = readScaled<int32_t>(SO, true);
                outData.Roll = readScaled<int32_t>(SO, false);
                break;
            case 0x09:
                for (int ind = 0; ind < 4; ++ind)
                    outData.Quat[ind] = readScaled<int16_t>(1e4, ind < 3);
                break;
            case 0x0a:
                for (int ind = 0; ind < 4; ++ind)
                    outData.Quat[ind] = readScaled<int32_t>(1e6, ind < 3);
                break;
            case 0x0d:
                outData.grid_heading = readScaled<uint16_t>(1e2, false);
                break;
            case 0x0e:
                for (int ind = 0; ind < 3; ++ind)
                    readScaled<int16_t>(1e2, ind < 2);
                break;
            case 0x10:
                outData.Latitude = readScaled<int32_t>(1e7, true);
                outData.Longitude = readScaled<int32_t>(1e7, true);
                outData.Altitude = readScaled<int32_t>(1e2, false);
                break;
            case 0x11:
                outData.Latitude = readScaled<int64_t>(1e9, true);
                outData.Longitude = readScaled<int64_t>(1e9, true);
                outData.Altitude = readScaled<int32_t>(1e3, false);
                break;
            case 0x12:
                for (int ind = 0; ind < 3; ++ind)
                    outData.VelENU[ind] = readScaled<int32_t>(SV, ind < 2);
                break;
            case 0x13:
                outData.Heave = readScaled<int32_t>(100.0, false);
                break;
            case 0x14:
                outData.Heave = readScaled<int32_t>(10000.0, false);
                break;
            case 0x15:
                outData.Heave_velocity = readScaled<int16_t>(100.0,false);
                break;
            case 0x16:
                outData.Surge = readScaled<int16_t>(100.0, true);
                outData.Sway = readScaled<int16_t>(100.0, false);
                break;
            case 0x17:
                outData.Surge = readScaled<int16_t>(1000.0, true);
                outData.Sway = readScaled<int16_t>(1000.0, false);
                break;
            case 0x18:
                outData.Surge_velocity = readScaled<int16_t>(100.0,true);
                outData.Sway_velocity = readScaled<int16_t>(100.0,false);
                break;
            case 0x19:
                outData.significant_wave_height = readScaled<uint16_t>(100.0, false);
                break;
            case 0x1B:
                for (int ind = 0; ind < 3; ++ind)
                    outData.VelENU[ind] = readScaled<int32_t>(1e6, ind < 2);
                break;
            case 0x1d:
                for (int j = 0; j < 3; ++j)
                    readScaled<int32_t>(1e2, j < 2);
                break;
            case 0x1e:
                for (int j = 0; j < 3; ++j)
                    readScaled<int32_t>(1e6, j < 2);
                break;
            case 0x20:
                for (int ind = 0; ind < 3; ++ind)
                {
                    outData.Gyro[ind] = readScaled<int16_t>(KG, ind < 2);
                }
                break;
            case 0x21:
                for (int ind = 0; ind < 3; ++ind)
                    outData.Gyro[ind] = readScaled<int32_t>(SG, ind < 2);
                break;
            case 0x22:
                for (int ind = 0; ind < 3; ++ind)
                    outData.Acc[ind] = readScaled<int16_t>(KA, ind < 2);
                break;
            case 0x23:
                for (int ind = 0; ind < 3; ++ind)
                    outData.Acc[ind] = readScaled<int32_t>(SA, ind < 2);
                break;
            case 0x24:
                for (int ind = 0; ind < 3; ++ind)
                    outData.Mag[ind] = readScaled<int16_t>(0.1, ind < 2);
                break;
            case 0x25:
                outData.pBar = readScaled<uint16_t>(0.5, true);
                outData.hBar = readScaled<int32_t>(100, false);
                break;
            case 0x26:
                for (int ind = 0; ind < 3; ++ind)
                    outData.GBias[ind] = readScaled<int8_t>(5e3, true);
                for (int ind = 0; ind < 3; ++ind)
                    outData.ABias[ind] = readScaled<int8_t>(5e4, true);
                readScaled<uint8_t>(1, false);
                break;
            case 0x27:
                for (int ind = 0; ind < 3; ++ind)
                    outData.AccPVPoint[ind] = readScaled<int32_t>(1e5, ind < 2);
                break;
            case 0x28:
                readScaled<int32_t>(1e4, false);
                break;
            case 0x29:
                for (int j = 0; j < 3; ++j)
                    readScaled<int32_t>(1e5, j < 2);
                break;
            case 0x2a:
                readScaled<uint8_t>(1, false);
                break;
            case 0x2b:
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        readScaled<float>(1, r < 2 || c < 2);
                break;
            case 0x2c:
                for (int r = 0; r < 3; ++r)
                    readScaled<float>(1, r < 2);
                break;
            case 0x30:
                outData.LatGNSS = readScaled<int32_t>(1e7, true);
                outData.LonGNSS = readScaled<int32_t>(1e7, true);
                outData.AltGNSS = readScaled<int32_t>(1e2, false);
                break;
            case 0x31:
                outData.LatGNSS = readScaled<int64_t>(1e9, true);
                outData.LonGNSS = readScaled<int64_t>(1e9, true);
                outData.AltGNSS = readScaled<int32_t>(1e3, false);
                break;
            case 0x32:
                outData.V_Hor = readScaled<int32_t>(100, true);
                outData.Trk_gnd = readScaled<uint16_t>(100, true);
                outData.V_ver = readScaled<int32_t>(100, false);
                break;
            case 0x33:
                outData.Heading_GNSS = readScaled<uint16_t>(100, true);
                outData.Pitch_GNSS = readScaled<int16_t>(100, false);
                break;
            case 0x34:
                outData.LatGNSSStd = readScaled<uint16_t>(1000, true);
                outData.LonGNSSStd = readScaled<uint16_t>(1000, true);
                outData.AltGNSSStd = readScaled<uint16_t>(1000, false);
                break;
            case 0x35:
                outData.HeadingGNSSStd = readScaled<uint16_t>(100, true);
                outData.PitchGNSSStd = readScaled<uint16_t>(100, false);
                break;
            case 0x36:
                outData.GNSSInfo1 = readScaled<uint8_t>(1, true, 16);
                outData.GNSSInfo2 = readScaled<uint8_t>(1, false, 16);
                break;
            case 0x37:
                outData.SVtrack = readScaled<uint8_t>(1, true);
                outData.SVsol = readScaled<uint8_t>(1, true);
                outData.SVsolL1 = readScaled<uint8_t>(1, true);
                outData.SVSolMulti = readScaled<uint8_t>(1, true);
                outData.GalBD = readScaled<uint8_t>(1, true, 16);
                outData.GPSGlo = readScaled<uint8_t>(1, true, 16);
                outData.TimeStatus = readScaled<uint8_t>(1, true);
                outData.ExtSolStatus = readScaled<uint8_t>(1, false, 16);
                break;
            case 0x38:
                outData.GNSSSolStatus = readScaled<uint8_t>(1, false, 16);
                break;
            case 0x39:
                outData.GNSSSolType = readScaled<uint8_t>(1, false);
                break;
            case 0x3A:
                outData.AnglesType = readScaled<uint8_t>(1, false);
                break;
            case 0x3B:
                outData.SVsol = readScaled<uint8_t>(1, false);
                break;
            case 0x3C:
                outData.Week = readScaled<uint16_t>(1, false);
                break;
            case 0x3D:
                outData.GNSSVelLatency = readScaled<uint16_t>(1, false);
                break;
            case 0x3E:
                outData.GNSSPosMs = readScaled<uint32_t>(1, false);
                break;
            case 0x3F:
                outData.GNSSVelMs = readScaled<uint32_t>(1, false);
                break;
            case 0x40:
                outData.GNSSHdgMs = readScaled<uint32_t>(1, false);
                break;
            case 0x41:
                outData.NewGPS = readScaled<uint8_t>(1, false, 16);
                break;
            case 0x42:
                outData.GDOP = readScaled<uint16_t>(1000, true);
                outData.PDOP = readScaled<uint16_t>(1000, true);
                outData.HDOP = readScaled<uint16_t>(1000, true);
                outData.VDOP = readScaled<uint16_t>(1000, true);
                outData.TDOP = readScaled<uint16_t>(1000, false);
                break;
            case 0x43:
                outData.GNSS_PACC = readScaled<uint16_t>(100, true);
                outData.GNSS_VACC = readScaled<uint16_t>(100, false);
                break;
            case 0x44:
                outData.GDOP = readScaled<uint16_t>(1000, true);
                outData.PDOP = readScaled<uint16_t>(1000, false);
                break;
            case 0x45:
                outData.Trk_gnd = readScaled<uint16_t>(100, false);
                break;
            case 0x47:
                outData.DiffAge = readScaled<uint16_t>(10, false);
                break;
            case 0x48:
                outData.GNSS_ECEF_VXStd = readScaled<uint16_t>(1000, true);
                outData.GNSS_ECEF_VYStd = readScaled<uint16_t>(1000, true);
                outData.GNSS_ECEF_VZStd = readScaled<uint16_t>(1000, true);
                break;
            case 0x49:
                outData.PPPApp = readScaled<uint8_t>(1, true);
                outData.PPPStore = readScaled<uint8_t>(1, false);
                break;
            case 0x4a:
                readScaled<uint8_t>(1, true);
                readScaled<uint8_t>(1, false);
                break;
            case 0x4e:
                readScaled<int32_t>(1e3, false);
                break;
            case 0x4f:
                readScaled<uint16_t>(1, false, 2);
                break;
            case 0x50:
                outData.VSup = readScaled<uint16_t>(100, false);
                break;
            case 0x51:
                outData.VStab = readScaled<uint16_t>(1000, false);
                break;
            case 0x52:
                outData.Temp = readScaled<int16_t>(10, false);
                break;
            case 0x53:
                outData.USW = readScaled<uint16_t>(1, false, 16);
                break;
            case 0x54:
                outData.INSSolStatus = readScaled<uint8_t>(1, false);
                break;
            case 0x55:
                outData.KFLatStd = readScaled<uint8_t>(100, true);
                outData.KFLonStd = readScaled<uint8_t>(100, true);
                outData.KFAltStd = readScaled<uint8_t>(100, false);
                break;
            case 0x56:
                outData.KFHdgStd = readScaled<uint8_t>(100, false);
                break;
            case 0x57:
                outData.KFLatStd = readScaled<uint16_t>(1000, true);
                outData.KFLonStd = readScaled<uint16_t>(1000, true);
                outData.KFAltStd = readScaled<uint16_t>(1000, false);
                break;
            case 0x58:
                outData.KFVelStd[0] = readScaled<uint8_t>(1000, true);
                outData.KFVelStd[1] = readScaled<uint8_t>(1000, true);
                outData.KFVelStd[2] = readScaled<uint8_t>(1000, false);
                break;
            case 0x59:
                readScaled<int32_t>(1e3, false);
                break;
            case 0x5a:
                outData.USW2 = readScaled<uint16_t>(1, false);
                break;
            case 0x5c:
                readScaled<uint16_t>(1e3, true);
                readScaled<uint16_t>(1e3, false);
                break;
            case 0x5e:
                readScaled<uint8_t>(1, false);
                break;
            case 0x5f:
                for (int j = 0; j < 6; ++j)
                    readScaled<int32_t>(1e3, j < 5);
                break;
            case 0x60:
                outData.Odometer = readScaled<int32_t>(1000, false);
                break;
            case 0x61:
                outData.AirSpeed = readScaled<int16_t>(1000, false);
                break;
            case 0x62:
                outData.WindN = readScaled<int16_t>(100, true);
                outData.WindE = readScaled<int16_t>(100, true);
                outData.WindNStd = readScaled<uint16_t>(100, true);
                outData.WindNStd = readScaled<uint16_t>(100, false);
                break;
            case 0x63:
                outData.LatExt = readScaled<int32_t>(1e7, true);
                outData.LonExt = readScaled<int32_t>(1e7, true);
                outData.AltExt = readScaled<int32_t>(1e3, true);
                outData.LatExtStd = readScaled<uint16_t>(100, true);
                outData.LonExtStd = readScaled<uint16_t>(100, true);
                outData.AltExtStd = readScaled<uint16_t>(100, true);
                outData.ExtPosLatency = readScaled<uint16_t>(1000, false);
                break;
            case 0x64:
                outData.LocLat = readScaled<int32_t>(1e7, true);
                outData.LocLon = readScaled<int32_t>(1e7, true);
                outData.LocAlt = readScaled<int32_t>(1e3, true);
                outData.LocDopplerShift = readScaled<int16_t>(100, true);
                outData.LocDopplerShiftStd = readScaled<uint16_t>(100, false);
                break;
            case 0x65:
                outData.NewAiding = readScaled<uint16_t>(1, false, 16);
                break;
            case 0x66:
                outData.HdgExt = readScaled<uint16_t>(100, true);
                outData.HdgExtStd = readScaled<uint16_t>(100, true);
                outData.HdgExtLatency = readScaled<uint16_t>(1000, false);
                break;
            case 0x67:
                outData.DVLRight = readScaled<int32_t>(1e3, true);
                outData.DVLFwd = readScaled<int32_t>(1e3, true);
                outData.DVLUp = readScaled<int32_t>(1e3, true);
                outData.DVLRightStd = readScaled<uint16_t>(1000, true);
                outData.DVLFwdStd = readScaled<uint16_t>(1000, true);
                outData.DVLUpStd = readScaled<uint16_t>(1000, true);
                outData.DVLLatency = readScaled<uint16_t>(1000, true);
                outData.DVLPressure = readScaled<uint32_t>(0.1, false);
                break;
            case 0x68:
                for (int ind = 0; ind < 3; ++ind)
                    outData.GBExt[ind] = readScaled<int8_t>(5e3, true);
                for (int ind = 0; ind < 3; ++ind)
                    outData.ABExt[ind] = readScaled<int8_t>(5e4, true);
                readScaled<uint8_t>(1, false);
                break;
            case 0x69:
                outData.PitchExt = readScaled<int16_t>(100, true);
                outData.RollExt = readScaled<int16_t>(100, false);
                break;
            case 0x6A:
                for (int ind = 0; ind < 3; ++ind)
                    outData.ExtAntPri[ind] = readScaled<int16_t>(1e3, true);
                for (int ind = 0; ind < 3; ++ind)
                    outData.ExtAntSec[ind] = readScaled<int16_t>(1e3, ind < 2);
                break;
            case 0x6B:
                outData.RefAirTemp = readScaled<int16_t>(10, true);
                outData.RefAltitude = readScaled<int32_t>(100, true);
                outData.RefAirPressure = readScaled<uint16_t>(0.5, false);
                break;
            case 0x6c:
                outData.AltExt = readScaled<int32_t>(1e3, true);
                outData.AltExtStd = readScaled<uint16_t>(100, false);
                break;
            case 0x6e:
                outData.LatExt = readScaled<int32_t>(1e7, true);
                outData.LonExt = readScaled<int32_t>(1e7, true);
                outData.LatExtStd = readScaled<uint16_t>(100, true);
                outData.LonExtStd = readScaled<uint16_t>(100, true);
                outData.ExtPosLatency = readScaled<uint16_t>(1000, false);
                break;
            case 0x80:
                outData.PStatic = readScaled<uint32_t>(1, false);
                break;
            case 0x81:
                outData.PDynamic = readScaled<int32_t>(100, false);
                break;
            case 0x82:
                readScaled<int32_t>(1e6, false);
                break;
            case 0x83:
                readScaled<int32_t>(1e3, false);
                break;
            case 0x84:
                outData.BHCP = readScaled<int32_t>(1000, false);
                break;
            case 0x85:
                readScaled<int16_t>(1e2, false);
                break;
            case 0x86:
                outData.TAS = readScaled<int16_t>(100, false);
                break;
            case 0x87:
                readScaled<uint16_t>(1e4, false);
                break;
            case 0x88:
                readScaled<int16_t>(1e2, false);
                break;
            case 0x89:
                readScaled<int16_t>(1e2, false);
                break;
            case 0x8A:
                for (int ind = 0; ind < 3; ++ind)
                    outData.ADUWind[ind] = readScaled<int16_t>(100, ind < 2);
                break;
            case 0x8b:
                readScaled<uint16_t>(1e4, false);
                break;
            case 0x8c:
                readScaled<int16_t>(1e2, false);
                break;
            case 0x8d:
                outData.ADUStatus = readScaled<uint16_t>(1, false);
                break;
            case 0x97:
                for (int j = 0; j < 3; ++j)
                    readScaled<int16_t>(KG, true);
                for (int j = 0; j < 3; ++j)
                    readScaled<int16_t>(KA, j < 2);
                break;
            case 0x98:
                for (int j = 0; j < 3; ++j)
                    readScaled<int32_t>(1e5, j < 2);
                break;
            case 0x99:
                readScaled<int16_t>(1e1, false);
                break;
            case 0x9a:
                readScaled<uint8_t>(1e1, false);
                break;
            case 0xA1:
                outData.NewAiding2 = readScaled<uint16_t>(1, false);
                break;
            case 0xA2:
                outData.PStatic = readScaled<uint32_t>(1, true);
                outData.PDynamic = readScaled<int32_t>(10, false);
                break;
            case 0xA3:
                outData.WNDspeed = readScaled<int32_t>(100, true);
                outData.WNDdir = readScaled<uint16_t>(100, false);
                break;
            case 0xA4:
                outData.BSsol = readScaled<uint8_t>(1, false);
                break;
            case 0xA5:
            case 0xA6:
            case 0xA7:
            case 0xA8:
            case 0xA9:
            case 0xAA:
            case 0xAB:
            case 0xAC:
            case 0xAD:
            case 0xAE:
            {
                auto& bs_range = outData.bs_ranges[udd_id - 0xa5];
                bs_range.lat = readScaled<int32_t>(1e7, true);
                bs_range.lon = readScaled<int32_t>(1e7, true);
                bs_range.alt = readScaled<int32_t>(1e3, true);
                bs_range.range = readScaled<uint32_t>(1e3, true);
                bs_range.std = readScaled<uint16_t>(1e2, true);
                bs_range.latency = readScaled<uint16_t>(1e3, false);
                break;
            }
            case 0xaf:
                readScaled<int32_t>(1e2, true);
                readScaled<int16_t>(1e1, false);
                break;
            case 0xb0:
                for (int j = 0; j < 3; ++j)
                    readScaled<int32_t>(1e2, j < 2);
                break;
            case 0xb1:
                for (int j = 0; j < 3; ++j)
                    readScaled<int32_t>(1e6, j < 2);
                break;
            case 0xb2:
                readScaled<int32_t>(1e2, true);
                readScaled<int32_t>(1e2, true);
                readScaled<int8_t>(1, false);
                break;
            case 0xc0:
                readScaled<uint8_t>(1, false, 2);
                break;
            case 0xc1:
                readScaled<uint32_t>(1, false);
                break;
            case 0xc2:
                readScaled<uint8_t>(1, false, 2);
                break;
            case 0xc4:
                readScaled<uint8_t>(1, true);
                readScaled<uint8_t>(1, false);
                break;
            case 0xd0:
                readScaled<uint8_t>(1, false);
                break;
            case 0xd1:
                readScaled<int16_t>(1e2, false);
                break;
            case 0xd2:
                for (int j = 0; j < 3; ++j)
                    readScaled<int16_t>(1e3, j < 2);
                break;
            case 0xd3:
                for (int j = 0; j < 4; ++j)
                    readScaled<int16_t>(1e3, j < 3);
                break;
            case 0xd4:
                for (int j = 0; j < 2; ++j)
                    readScaled<int16_t>(1e3, j < 1);
                break;
            case 0xd5:
                outData.extended_cmd_code = readScaled<uint8_t>(1, true);
                outData.extended_cmd_val = readScaled<uint8_t>(1, true);
                readScaled<uint8_t>(1, true);
                readScaled<uint8_t>(1, false);
                break;
            case 0xF0:
                outData.Latency_ms_pos = readScaled<uint8_t>(1, true);
                outData.Latency_ms_vel = readScaled<uint8_t>(1, false);
                break;
            case 0xF1:
                outData.Latency_ms_head = readScaled<uint16_t>(1, false);
                break;
            case 0xF2:
                outData.Heading_GNSS = readScaled<uint16_t>(100, false);
                break;
            case 0xF3:
                outData.TimeStatus = readScaled<uint8_t>(1, true);
                outData.GNSSSolStatus = readScaled<uint8_t>(1, true);
                outData.GNSSSolType = readScaled<uint8_t>(1, true);
                outData.SVtrack = readScaled<uint8_t>(1, true);
                outData.SVsol = readScaled<uint8_t>(1, true);
                outData.SVsolL1 = readScaled<uint8_t>(1, true);
                outData.SVSolMulti = readScaled<uint8_t>(1, true);
                outData.ExtSolStatus = readScaled<uint8_t>(1, true, 16);
                outData.GalBD = readScaled<uint8_t>(1, true);
                outData.GPSGlo = readScaled<uint8_t>(1, false);
                break;
            case 0xF4:
                outData.UP = readScaled<uint16_t>(1, true);
                outData.UT = readScaled<uint16_t>(1, false);
                break;
            case 0xF5:
                outData.GNSSInfo1 = readScaled<uint8_t>(1, false, 16);
                break;
            case 0xF6:
                outData.pBar = readScaled<uint16_t>(0.5, true);
                outData.Temp = readScaled<uint16_t>(100, true);
                outData.USW = readScaled<uint16_t>(1, true, 16);
                outData.GNSSSolType = readScaled<uint8_t>(1, true);
                outData.GNSS_ECEF_X = readScaled<int32_t>(100, true);
                outData.GNSS_ECEF_Y = readScaled<int32_t>(100, true);
                outData.GNSS_ECEF_Z = readScaled<int32_t>(100, true);
                outData.GNSS_PACC = readScaled<uint16_t>(100, true);
                outData.GNSS_ECEF_VX = readScaled<int32_t>(1e6, true);
                outData.GNSS_ECEF_VY = readScaled<int32_t>(1e6, true);
                outData.GNSS_ECEF_VZ = readScaled<int32_t>(1e6, true);
                outData.GNSS_VACC = readScaled<uint16_t>(100, true);
                outData.LatGNSS = readScaled<int64_t>(1e9, true);
                outData.LonGNSS = readScaled<int64_t>(1e9, true);
                outData.AltGNSS = readScaled<int32_t>(1e3, true);
                outData.V_Hor = readScaled<int32_t>(1e6, true);
                outData.Trk_gnd = readScaled<int32_t>(1e6, true);
                outData.V_ver = readScaled<int32_t>(1e6, true);
                outData.AnglesType = readScaled<uint8_t>(1, true);
                outData.Heading_GNSS = readScaled<uint16_t>(100, true);
                outData.SVsol = readScaled<uint8_t>(1, true);
                outData.GNSSInfo1 = readScaled<uint8_t>(1, true, 16);
                outData.GNSSInfo2 = readScaled<uint8_t>(1, true, 16);
                outData.GDOP = readScaled<uint16_t>(1e3, true);
                outData.PDOP = readScaled<uint16_t>(1e3, true);
                outData.HDOP = readScaled<uint16_t>(1e3, true);
                outData.VDOP = readScaled<uint16_t>(1e3, true);
                outData.TDOP = readScaled<uint16_t>(1e3, true);
                outData.DiffAge = readScaled<uint16_t>(10, true);
                outData.UTC_Hour = readScaled<uint8_t>(1, true);
                outData.UTC_Minute = readScaled<uint8_t>(1, true);
                outData.UTC_Second = readScaled<uint8_t>(1, true);
                outData.UTC_DecSec = readScaled<uint16_t>(1e3, true);
                outData.UTC_Day = readScaled<uint8_t>(1, true);
                outData.UTC_Month = readScaled<uint8_t>(1, true);
                outData.UTC_Year = readScaled<uint16_t>(1, true);
                outData.UTCSecSinceEpoch = readScaled<int64_t>(1, true);
                outData.LatencyECEF = readScaled<uint8_t>(1, false);
                break;
            case 0xF7:
                outData.Latency_ms_pos = readScaled<int16_t>(1, true);
                outData.Latency_ms_vel = readScaled<int16_t>(1, false);
                break;
            case 0xFE:
                readScaled<uint8_t>(1, false, 16);
                break;
            case 0xFF:
                readScaled<uint16_t>(1, false, 16);
                break;
            default:
                //			cout << "Unknown data!" << endl;
                break;
            }
            if (i < dataSet.size() - 1)
                txtStream << "\t";
        }
        txtStream << "\n";
    }

    void UDDParser::finish()
    {
    }
}
