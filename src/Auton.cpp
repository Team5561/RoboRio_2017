/*
  Auton.cpp

   Created on: Mar 3, 2017
       Author: biggs
 */

#include "RobotConstCal.hpp"

Auton_Mode3              V_AutonMode3;
bool                     V_AutonControlComplete;
double                   V_Auton3_SerachTime;
double                   V_StopDebounceTimer;
double                   V_ControlDeboucneTimer;
double                   V_DriveStraightTimer;
bool                     V_CameraDataValid;
bool                     V_CameraLED;
double                   V_Distance1;
double                   V_Distance2;
double                   V_Area1;
double                   V_Area2;
double                   V_Height1;
double                   V_Height2;
double                   V_Center;
double                   V_CenterRaw;
double                   V_TimeCenterDataValid;
double                   V_CenterDB_High;
double                   V_CenterDB_Low;
double                   V_BlobsDetected;
double                   V_AutonMode3_CenterSearchError;
double                   V_AutonMode3_CenterSearchDrvPwr;
double                   V_AutonMode3_CenterSearchMoveTmr;
Auton_Mode3_CenterSearch V_AutonMode3_CenterSearch;
bool                     V_AutonLightReq;
bool                     V_AutonDriveCntrlCmpt;
bool                     V_AutonFieldOriented;

/******************************************************************************
 * Function:     AutonStatesInit
 *
 * Description:  Initializes auton variables.
 ******************************************************************************/
void AutonStatesInit(void)
  {
  V_AutonMode3 = C_Auton3_Forward1;
  V_AutonControlComplete = false;
  V_StopDebounceTimer = 0.0;
  V_ControlDeboucneTimer = 0.0;
  V_DriveStraightTimer = 0.0;
  V_CameraDataValid = false;
  V_TimeCenterDataValid = 0;
  V_CenterDB_High = 0.0;
  V_CenterDB_Low = 0.0;
  V_AutonLightReq = false;
  V_AutonFieldOriented = false;
  V_AutonDriveCntrlCmpt = false;
  V_AutonMode3_CenterSearchError = 0.0;
  V_AutonMode3_CenterSearch = Auton_Mode3_CenterSearchOff;
  V_AutonMode3_CenterSearchMoveTmr = 0.0;
  V_AutonMode3_CenterSearchDrvPwr = 0.0;
  V_CameraLED = false;
  }


/******************************************************************************
 * Function:     CenterPoint
 *
 * Description:  Determines the center point from two blobs.
 ******************************************************************************/
double CenterPoint(double L_Distance1,
                   double L_Distance2,
                   double L_BlobCount)
  {
  double L_Center;
  double L_CenterVld = C_ImageOutsideRange;

  if (L_Distance1 < C_ImageOutsideRange &&
      L_Distance2 < C_ImageOutsideRange &&
      L_BlobCount == K_CameraExpectedBlobCount)
    {
    L_Center = ((fabs(L_Distance1) + fabs(L_Distance2))/2);

    V_CenterRaw = L_Center;

    if ((V_TimeCenterDataValid <= 0.0) ||
        (L_Center > V_CenterDB_High ||
         L_Center < V_CenterDB_Low))
      {
      // This is the first loop the image is valid:
      V_CenterDB_High = L_Center + K_CameraCenterDB_High;
      V_CenterDB_Low  = L_Center - K_CameraCenterDB_Low;
      V_TimeCenterDataValid = C_ControllerUpdateRate;
      }
    else if (L_Center <= V_CenterDB_High &&
             L_Center >= V_CenterDB_Low)
      {
      V_TimeCenterDataValid += C_ControllerUpdateRate;
      if (V_TimeCenterDataValid >= K_CameraCenterDB_Time)
        {
        V_TimeCenterDataValid = K_CameraCenterDB_Time;
        L_CenterVld = L_Center;
        }
      }
    }
  else
    {
    L_Center = C_ImageOutsideRange;
    V_CenterRaw = C_ImageOutsideRange;
    V_TimeCenterDataValid = 0.0;
    V_CenterDB_High = 0.0;
    V_CenterDB_Low = 0.0;
    }

  return L_CenterVld;
  }


/******************************************************************************
 * Function:     CameraProcessing
 *
 * Description:  Houses the camera processing functions.
 ******************************************************************************/
void CameraProcessing()
  {
#ifdef VISION
  int    L_Index;
  double L_Distance1 = C_ImageOutsideRange;
  double L_Distance2 = C_ImageOutsideRange;
  double L_Area1 = 0;
  double L_Area2 = 0;
  double L_Height1 = C_ImageOutsideRange;
  double L_Height2 = C_ImageOutsideRange;
  double L_BlobsDetected = 0;
  double L_Center = 0;

  std::vector<double> L_ImageInfo = table->GetNumberArray("centerX", llvm::ArrayRef<double>());

  L_BlobsDetected = L_ImageInfo.size();

   for (L_Index = 0; L_Index < L_BlobsDetected; L_Index++)
   {
     if(L_Index == 0)
       {
       L_Distance1 = L_ImageInfo[L_Index];
       }
     else if (L_Index == 1)
       {
       L_Distance2 = L_ImageInfo[L_Index];
       }
   }

//   L_ImageInfo = table->GetNumberArray("area", llvm::ArrayRef<double>());
//   for (L_Index = 0; L_Index < L_BlobsDetected; L_Index++)
//   {
//     if(L_Index == 0)
//       {
//       L_Area1 = L_ImageInfo[L_Index];
//       }
//     else if (L_Index == 1)
//       {
//       L_Area2 = L_ImageInfo[L_Index];
//       }
//   }
//
//   L_ImageInfo = table->GetNumberArray("height", llvm::ArrayRef<double>());
//
//   for (L_Index = 0; L_Index < L_BlobsDetected; L_Index++)
//   {
//     if(L_Index == 0)
//       {
//       L_Height1 = L_ImageInfo[L_Index];
//       }
//     else if (L_Index == 1)
//       {
//       L_Height2 = L_ImageInfo[L_Index];
//       }
//   }

   L_Center =  CenterPoint(L_Distance1,
                           L_Distance2,
                           L_BlobsDetected);

//   V_Distance1     = L_Distance1;
//   V_Distance2     = L_Distance2;
//   V_Area1         = L_Area1;
//   V_Area2         = L_Area2;
//   V_Height1       = L_Height1;
//   V_Height2       = L_Height2;
   V_Distance1     = 0.0;
   V_Distance2     = 0.0;
   V_Area1         = 0.0;
   V_Area2         = 0.0;
   V_Height1       = 0.0;
   V_Height2       = 0.0;
   V_BlobsDetected = L_BlobsDetected;
   V_Center        = L_Center;
#else
   V_Distance1     = 0;
   V_Distance2     = 0;
   V_Area1         = 0;
   V_Area2         = 0;
   V_Height1       = 0;
   V_Height2       = 0;
   V_BlobsDetected = 0;
   V_Center        = 0;
#endif
  }


/******************************************************************************
 * Function:     DriveCntrl
 *
 * Description:  Provides proportional control.
 ******************************************************************************/
double DriveCntrl(double L_CurrentState,
                  double L_DesiredState,
                  double L_ErrorDb,
                  double L_Gain,
                  double L_MaxPos,
                  double L_MaxNeg,
                  double L_MinPos,
                  double L_MinNeg,
                  double L_DebounceTime)
  {
  double L_ControlOutput = 0.0;
  double L_Error = 0.0;

  L_Error = L_DesiredState - L_CurrentState;

  if (fabs(L_Error) > L_ErrorDb)
    {
    V_ControlDeboucneTimer = 0.0;

    L_ControlOutput = L_Error * L_Gain;

    if (L_ControlOutput > L_MaxPos)
      {
      L_ControlOutput = L_MaxPos;
      }
    else if (L_ControlOutput < L_MaxNeg)
      {
      L_ControlOutput = L_MaxNeg;
      }

    if (L_ControlOutput > 0.0 &&
        L_ControlOutput < L_MinPos)
      {
      L_ControlOutput = L_MinPos;
      }
    else if (L_ControlOutput < 0.0 &&
             L_ControlOutput > L_MinNeg)
      {
      L_ControlOutput = L_MinNeg;
      }
    }
  else
    {
    L_ControlOutput = 0.0;
    V_ControlDeboucneTimer += C_ControllerUpdateRate;

    if (V_ControlDeboucneTimer >= L_DebounceTime)
      {
      V_ControlDeboucneTimer = 0.0;
      V_AutonDriveCntrlCmpt = true;
      }
    }

  return (L_ControlOutput);
  }


/******************************************************************************
 * Function:     DtrmnWhenStopped
 *
 * Description:  Determines when the bot has come to a stop.
 ******************************************************************************/
bool DtrmnWhenStopped(double L_YawRate,
                      double L_X_Velocity,
                      double L_Y_Velocity,
                      double L_DebounceThreshold)
  {
  bool   L_BotStopped = false;

  if (fabs(L_YawRate)    < K_YawRateStopThresh &&
      fabs(L_X_Velocity) < K_X_AccelStopThresh &&
      fabs(L_Y_Velocity) < K_Y_AccelStopThresh)
    {
    V_StopDebounceTimer += C_ControllerUpdateRate;

    if (V_StopDebounceTimer > L_DebounceThreshold)
      {
      L_BotStopped = true;
      V_StopDebounceTimer = 0.0;
      }
    }

  return (L_BotStopped);
  }


/******************************************************************************
 * Function:     AutonOption3
 *
 * Description:  This houses the 3 autonomous controls for left, middle and right.
 ******************************************************************************/
void AutonOption3(double      L_GyroAngle,
                  double      L_YawRate,
                  double      L_X_Velocity,
                  double      L_X_Accel,
                  double      L_Y_Velocity,
                  double      L_Y_Accel)
  {
#ifdef VISION
    double   L_X_Drive = 0.0;
    double   L_Y_Drive = 0.0;
    double   L_RotationCmnd = 0.0;
    bool     L_AutonFieldOriented = false;

    switch (V_AutonMode3)
      {
        case C_Auton3_Forward1:
          L_AutonFieldOriented = true;

          if (V_DriveStraightTimer <= K_Auton3_ForwardTime[V_AutonPosition])
            {
            V_DriveStraightTimer += C_ControllerUpdateRate;
            L_Y_Drive = K_Auton3_ForwardSpd1[V_AutonPosition];
            }
          else if (V_DriveStraightTimer > K_Auton3_ForwardTime[V_AutonPosition])
            {
            V_AutonControlComplete =  DtrmnWhenStopped(L_YawRate,
                                                       L_X_Velocity,
                                                       L_Y_Velocity,
                                                       K_StopWaitTime);
            }


          if (V_AutonControlComplete == true)
            {
            V_AutonControlComplete = false;
            V_DriveStraightTimer = 0.0;
            V_AutonMode3 = C_Auton3_Rotate;
            }
          break;

        case C_Auton3_Rotate:
          L_AutonFieldOriented = true;
          if (V_AutonDriveCntrlCmpt == false)
            {
            L_RotationCmnd = DriveCntrl(L_GyroAngle,
                                        K_Auton3_Angle[V_AutonPosition],
                                        K_RotationErrorDb,
                                        K_RotationGain,
                                        K_MaxPosRotation,
                                        K_MaxNegRotation,
                                        K_RotationMinPos,
                                        K_RotationMinNeg,
                                        K_CntrlDebounceTime);
            }
          else
            {
            V_AutonControlComplete =  DtrmnWhenStopped(L_YawRate,
                                                       L_X_Velocity,
                                                       L_Y_Velocity,
                                                       K_StopWaitTime);
            }

          if (V_AutonControlComplete == true)
            {
            V_CameraLED = true;
            if (V_AutonPosition == C_AutonPositionMiddle)
              {
              // For the middle case, jump to search, no need to drive forward again
              V_AutonMode3 = C_Auton3_CenterSearch;
              }
            else
              {
              V_AutonMode3 = C_Auton3_Forward2;
              }
            V_AutonControlComplete = false;
            }
          break;

        case C_Auton3_Forward2:
          if (V_DriveStraightTimer <= K_Auton3_ForwardTime2[V_AutonPosition])
            {
            V_DriveStraightTimer += C_ControllerUpdateRate;
            L_Y_Drive = K_Auton3_ForwardSpd2[V_AutonPosition];
            }
          else if (V_DriveStraightTimer > K_Auton3_ForwardTime2[V_AutonPosition])
            {
            V_AutonControlComplete =  DtrmnWhenStopped(L_YawRate,
                                                       L_X_Velocity,
                                                       L_Y_Velocity,
                                                       K_StopWaitTime);
            }

          if (V_AutonControlComplete == true)
            {
            V_DriveStraightTimer = 0.0;
            V_AutonControlComplete = false;
            V_AutonMode3 = C_Auton3_CenterSearch;
            }
          break;

        case C_Auton3_CenterSearch:
          V_Auton3_SerachTime += C_ControllerUpdateRate;

          switch (V_AutonMode3_CenterSearch)
            {
              case Auton_Mode3_CenterSearchOff:
              case Auton_Mode3_CenterSearchWaitingForCamera:
                if (V_Center < C_ImageOutsideRange)
                  {
                  if (V_Center <= K_Auton3_CenterDeadbandHigh &&
                      V_Center >= K_Auton3_CenterDeadbandLow)
                    {
                    V_AutonMode3_CenterSearch = Auton_Mode3_CenterSearchComplete;
                    }
                  else
                    {
                    V_AutonMode3_CenterSearchError = V_Center - K_Auton3_DsrdCenter;
                    V_AutonMode3_CenterSearchMoveTmr = fabs(V_AutonMode3_CenterSearchError *
                                                            K_Auton3_SearchTimeGain[V_AutonPosition]);
                    if (V_AutonMode3_CenterSearchMoveTmr > K_Auton3_SearchTime)
                      {
                      V_AutonMode3_CenterSearchMoveTmr = K_Auton3_SearchTime;
                      }
                    else if (V_AutonMode3_CenterSearchMoveTmr < K_Auton3_MinSearchTm[V_AutonPosition])
                      {
                      V_AutonMode3_CenterSearchMoveTmr = K_Auton3_MinSearchTm[V_AutonPosition];
                      }
                    if (V_AutonMode3_CenterSearchError > 0.0)
                      {
                      V_AutonMode3_CenterSearchDrvPwr = K_Auton3_SearchNeg;
                      }
                    else
                      {
                      V_AutonMode3_CenterSearchDrvPwr = K_Auton3_SearchPos;
                      }
                    V_AutonMode3_CenterSearch = Auton_Mode3_CenterSearchMoving;
                    }
                  }
                break;
              case Auton_Mode3_CenterSearchMoving:
                if (V_AutonMode3_CenterSearchMoveTmr > 0.0)
                  {
                  L_X_Drive = V_AutonMode3_CenterSearchDrvPwr;
                  V_AutonMode3_CenterSearchMoveTmr -= C_ControllerUpdateRate;
                  }
                else
                  {
                  V_AutonMode3_CenterSearchMoveTmr = 0.0;
                  V_AutonMode3_CenterSearch = Auton_Mode3_CenterSearchRotate;
                  }
                break;
              case Auton_Mode3_CenterSearchRotate:
                if (V_AutonDriveCntrlCmpt == false)
                  {
                  L_RotationCmnd = DriveCntrl(L_GyroAngle,
                                              K_Auton3_Angle[V_AutonPosition],
                                              K_RotationErrorDb,
                                              K_RotationGain,
                                              K_MaxPosRotation,
                                              K_MaxNegRotation,
                                              K_RotationMinPos,
                                              K_RotationMinNeg,
                                              K_CntrlDebounceTime);
                  }
                else
                  {
                  if (DtrmnWhenStopped(L_YawRate,
                                       L_X_Velocity,
                                       L_Y_Velocity,
                                       K_StopWaitTime))
                    {
                    V_AutonMode3_CenterSearch = Auton_Mode3_CenterSearchWaitingForCamera;
                    }
                  }
                break;
              case Auton_Mode3_CenterSearchComplete:
                V_AutonControlComplete = true;
                break;
            }

          if (V_AutonControlComplete == true)
            {
              V_AutonControlComplete = false;
              V_AutonMode3 = C_Auton3_Forward3;
            }
          break;

        case C_Auton3_Forward3:
          if (V_DriveStraightTimer <= K_Auton3_ForwardTime3[V_AutonPosition])
            {
            V_DriveStraightTimer += C_ControllerUpdateRate;
            L_Y_Drive = K_Auton3_ForwardSpd3[V_AutonPosition];
            }
          else if (V_DriveStraightTimer > K_Auton3_ForwardTime3[V_AutonPosition])
            {
            V_AutonControlComplete =  DtrmnWhenStopped(L_YawRate,
                                                       L_X_Velocity,
                                                       L_Y_Velocity,
                                                       K_StopWaitTime);
            }

          if (V_AutonControlComplete == true)
            {
            V_DriveStraightTimer = 0.0;
            V_AutonControlComplete = false;
            V_AutonMode3 = C_Auton3_Off;
            }
          break;

        case C_Auton3_Off:
          V_AutonControlComplete = true;
          V_AutonLightReq = true;
          V_CameraLED = false;
          break;
      }

    V_AutonFieldOriented = L_AutonFieldOriented;
    V_DriveX_Auton = L_X_Drive;
    V_DriveY_Auton = L_Y_Drive;
    V_DriveZ_Auton = L_RotationCmnd;
#else
    V_AutonFieldOriented = 0;
    V_DriveX_Auton = 0;
    V_DriveY_Auton = 0;
    V_DriveZ_Auton = 0;
#endif
  }


/******************************************************************************
 * Function:     AutonDefault
 *
 * Description:  This is the default auton option.
 ******************************************************************************/
void AutonDefault(double L_MatchTime)
  {
    double   L_X_Drive;
    double   L_Y_Drive;
    double   L_RotationCmnd;

    L_X_Drive = 0.0;
    L_RotationCmnd = 0.0;

    if (L_MatchTime > K_Auton3_ForwardTime[C_AutonPositionDefault])
      {
      L_Y_Drive = K_Auton3_ForwardSpd1[C_AutonPositionDefault];
      L_X_Drive = -0.0045;
      V_AutonLightReq = false;
      }
    else
      {
      L_Y_Drive = 0.0;
	  L_X_Drive = 0.0;
      V_AutonLightReq = true;
      }

    V_DriveX_Auton = L_X_Drive;
    V_DriveY_Auton = L_Y_Drive;
    V_DriveZ_Auton = L_RotationCmnd;
    V_AutonFieldOriented = false;
    V_CameraLED = false;
  }
