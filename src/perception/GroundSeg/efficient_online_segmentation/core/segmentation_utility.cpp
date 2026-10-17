/*
 * Copyright 2021 Guanhua WANG
 */

#include "segmentation_utility.h"
#include <iostream>


SegmentationParams::SegmentationParams() {}

SegmentationParams& SegmentationParams::operator=(const SegmentationParams& other)
{
    kLidarRows = other.kLidarRows;
    kLidarCols = other.kLidarCols;
    kLidarHorizRes = other.kLidarHorizRes;
    kLidarVertRes = other.kLidarVertRes;
    kLidarVertFovMax = other.kLidarVertFovMax;
    kLidarVertFovMin = other.kLidarVertFovMin;
    kLidarProjectionError = other.kLidarProjectionError;

    kLidarHorizResInv = other.kLidarHorizResInv;
    kLidarVertResInv = other.kLidarVertResInv;
    kLidarVertAngMin = other.kLidarVertAngMin;
    kLidarVertAngMax = other.kLidarVertAngMax;

    kNumSectors = other.kNumSectors;
    kColsPerSector = other.kColsPerSector;
    kSensorHeight = other.kSensorHeight;
    kSensorRoll = other.kSensorRoll;
    kSensorPitch = other.kSensorPitch;
    kExtrinsicTrans = other.kExtrinsicTrans;
    kExtrinsicRot = other.kExtrinsicRot;
    kExtrinsicTF = other.kExtrinsicTF;
    kBaseToSensor = other.kBaseToSensor;

    kGroundSameLineTolerance = other.kGroundSameLineTolerance;
    kGroundSlopeTolerance = other.kGroundSlopeTolerance;
    kGroundYInterceptTolerance = other.kGroundYInterceptTolerance;
    kGroundPointLineDistThres = other.kGroundPointLineDistThres;

    kWallSameLineTolerance = other.kWallSameLineTolerance;
    kWallSlopeTolerance = other.kWallSlopeTolerance;
    kWallLineMinBinNum = other.kWallLineMinBinNum;
    kWallPointLineDistThres = other.kWallPointLineDistThres;

    // PrintAllParams("Copied parameters");
    return *this;
}

bool SegmentationParams::UpdateInternalParams()
{
    if (kLidarRows <= 0 || kLidarCols <= 0 || kNumSectors <= 0 || kNumSectors > kLidarCols ||
        !std::isfinite(kLidarHorizRes) || kLidarHorizRes <= 0 ||
        !std::isfinite(kLidarVertRes) || kLidarVertRes <= 0 ||
        !std::isfinite(kLidarVertFovMin) || !std::isfinite(kLidarVertFovMax) ||
        kLidarVertFovMin >= kLidarVertFovMax || !std::isfinite(kLidarProjectionError) ||
        kLidarProjectionError < 0 || !kExtrinsicRot.allFinite() || !kExtrinsicTrans.allFinite())
        return false;
    for (float limit : {kGroundSameLineTolerance, kGroundSlopeTolerance,
                        kGroundYInterceptTolerance, kGroundPointLineDistThres,
                        kWallSameLineTolerance, kWallSlopeTolerance, kWallPointLineDistThres})
        if (!std::isfinite(limit) || limit <= 0) return false;
    if (kWallLineMinBinNum < 2) return false;
    bool is_user_params_correct = true;
    kLidarHorizResInv = 1/kLidarHorizRes;
    kLidarVertResInv = 1/kLidarVertRes;
    kLidarVertAngMin = kLidarVertFovMin - kLidarProjectionError;
    kLidarVertAngMax = kLidarVertFovMax + kLidarProjectionError;
    kColsPerSector = kLidarCols/kNumSectors;

    // Check: the rotation matrix must be a special orthogonal matrix!
    {
        std::cout << std::endl;
        std::cout << "###################### CHECK ROTATION MATRIX ###################### " << std::endl;
        float det = kExtrinsicRot.determinant();
        std::cout << "## CHECK: RotMatrix.determinant(): " << det << std::endl << std::endl;
        Eigen::Matrix3f multipMat = kExtrinsicRot * kExtrinsicRot.transpose();
        std::cout << "## CHECK: RotMatrix*RotMatrix.transpose(): " << std::endl 
                  << multipMat << std::endl;
        multipMat = multipMat - Eigen::Matrix3f::Identity();
        float multipValue = std::abs(multipMat(0,0)) + std::abs(multipMat(0,1)) + std::abs(multipMat(0,2))
                            + std::abs(multipMat(1,0)) + std::abs(multipMat(1,1)) + std::abs(multipMat(1,2))
                            + std::abs(multipMat(2,0)) + std::abs(multipMat(2,1)) + std::abs(multipMat(2,2));
        std::cout << "## CHECK: RotMatrix*RotMatrix.transpose() abs sum: " << multipValue << std::endl;
        if (multipValue>0.01 || std::abs(det-1)>0.01 ) {
            is_user_params_correct = false;
            std::cout << std::endl;
            std::cout << " ############################ ERROR! ############################ " << std::endl;
            std::cout << " the extrinsic rotation matrix must be a SO3 matrix, which is not!" << std::endl;
            std::cout << " ############################ ERROR! ############################" << std::endl;
            std::cout << std::endl;
        }
        else {
            std::cout << "## CHECK: rotation matrix is cool. " << std::endl;
        }
        std::cout << "###################### CHECK ROTATION MATRIX ###################### " << std::endl;
        std::cout << std::endl;
    }

    kExtrinsicTF = Eigen::Matrix4f::Identity();
    kExtrinsicTF.block<3,3>(0,0) = kExtrinsicRot;
    kExtrinsicTF.block<3,1>(0,3) = kExtrinsicTrans;
    kBaseToSensor = kExtrinsicTF;

    PrintAllParams("Updated parameters");

    if (is_user_params_correct) {return true;}
    else {return false;}
}

void SegmentationParams::PrintAllParams(const std::string& title)
{
    std::cout << "########## " << title << ": " << std::endl
            << "kLidarRows: " << kLidarRows << std::endl
            << "kLidarCols: " << kLidarCols << std::endl
            << "kLidarHorizRes: " << kLidarHorizRes << std::endl
            << "kLidarVertRes: " << kLidarVertRes << std::endl
            << "kLidarVertFovMax: " << kLidarVertFovMax << std::endl
            << "kLidarVertFovMin: " << kLidarVertFovMin << std::endl
            << "kLidarProjectionError: " << kLidarProjectionError << std::endl
            << " " << std::endl
            << "kLidarHorizResInv: " << kLidarHorizResInv << std::endl
            << "kLidarVertResInv: " << kLidarVertResInv << std::endl
            << "kLidarVertAngMin: " << kLidarVertAngMin << std::endl
            << "kLidarVertAngMax: " << kLidarVertAngMax << std::endl
            << " " << std::endl
            << "kNumSectors: " << kNumSectors << std::endl
            << "kColsPerSector: " << kColsPerSector << std::endl
            << "kSensorHeight: " << kSensorHeight << std::endl
            << "kSensorRoll: " << kSensorRoll << std::endl
            << "kSensorPitch: " << kSensorPitch << std::endl
            << "kExtrinsicTrans: " << std::endl 
            << kExtrinsicTrans << std::endl 
            << "kExtrinsicRot: " << std::endl 
            << kExtrinsicRot << std::endl 
            << "kBaseToSensor: " << std::endl 
            << kBaseToSensor.matrix() << std::endl 
            << " " << std::endl
            << "kGroundSameLineTolerance: " << kGroundSameLineTolerance << std::endl 
            << "kGroundSlopeTolerance: " << kGroundSlopeTolerance << std::endl 
            << "kGroundYInterceptTolerance: " << kGroundYInterceptTolerance << std::endl 
            << "kGroundPointLineDistThres: " << kGroundPointLineDistThres << std::endl 
            << " " << std::endl
            << "kWallSameLineTolerance: " << kWallSameLineTolerance << std::endl 
            << "kWallSlopeTolerance: " << kWallSlopeTolerance << std::endl 
            << "kWallLineMinBinNum: " << kWallLineMinBinNum << std::endl 
            << "kWallPointLineDistThres: " << kWallPointLineDistThres << std::endl 
            << std::endl;
}


