/*
    MaxDome II
    Copyright (C) 2009 Ferran Casarramona (ferran.casarramona@gmail.com)

    Migrated to INDI::Dome by Jasem Mutlaq (2014)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "config.h"
#include "maxdomeii.h"
#include "maxdomeiidriver.h"  // MaxDome II Command Set

#include <connectionplugins/connectionserial.h>

#include <memory>
#include <math.h>
#include <string.h>
#include <unistd.h>


// We declare an auto pointer to dome.
std::unique_ptr<MaxDomeII> dome(new MaxDomeII());

void ISGetProperties(const char *dev)
{
    dome->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    dome->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num)
{
    dome->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    dome->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle *root)
{
    dome->ISSnoopDevice(root);
}

MaxDomeII::MaxDomeII()
{
    nTicksPerTurn               = 360;
    nCurrentTicks               = 0;
    nHomeAzimuth                = 0.0;
    nHomeTicks                  = 0;
    nCloseShutterBeforePark     = 0;
    nTimeSinceShutterStart      = -1; // No movement has started
    nTimeSinceAzimuthStart      = -1; // No movement has started
    nTargetAzimuth              = -1; //Target azimuth not established
    nTimeSinceLastCommunication = 0;

    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_ABS_MOVE | DOME_HAS_SHUTTER
                      | DOME_CAN_PARK);

    setVersion(INDI_MAXDOMEII_VERSION_MAJOR, INDI_MAXDOMEII_VERSION_MINOR);
}

bool MaxDomeII::SetupParms()
{
    DomeAbsPosN[0].value = 0;

    loadConfig();

    IDSetNumber(&DomeAbsPosNP, nullptr);
    IDSetNumber(&DomeParamNP, nullptr);

    //int defaultParkPos = floor(0.5 + DEFAULT_PARK_AZ * nTicksPerTurn / 360.0);
    SetAxis1ParkDefault(DEFAULT_PARK_AZ);

    // If loading parking data is successful, we just set the default parking values.
    // Otherwise, we set all parking data to default in case no parking data is found
    if (!InitPark())
        SetAxis1Park(DEFAULT_PARK_AZ);

    return true;
}

bool MaxDomeII::Handshake()
{
    driver.SetDevice(getDeviceName());
    driver.SetPortFD(PortFD);

    return !driver.Ack();
}

MaxDomeII::~MaxDomeII()
{
}

const char *MaxDomeII::getDefaultName()
{
    return (char *)"MaxDome II";
}

bool MaxDomeII::initProperties()
{
    INDI::Dome::initProperties();

    // Ticks per turn
    IUFillNumber(&TicksPerTurnN[0], "TICKS_PER_TURN", "Ticks per turn", "%5.2f", 100., 2000., 0., nTicksPerTurn);
    IUFillNumberVector(&TicksPerTurnNP, TicksPerTurnN, NARRAY(TicksPerTurnN), getDeviceName(),
                       "TICKS_PER_TURN", "Ticks per turn", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Home azimuth
    IUFillNumber(&HomeAzimuthN[0], "HOME_AZIMUTH", "Home azimuth", "%5.2f", 0., 360., 0., nHomeAzimuth);
    IUFillNumberVector(&HomeAzimuthNP, HomeAzimuthN, NARRAY(HomeAzimuthN), getDeviceName(),
                       "HOME_AZIMUTH", "Home azimuth", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Park on shutter
    IUFillSwitch(&ParkOnShutterS[0], "MOVE", "Move", ISS_ON);
    IUFillSwitch(&ParkOnShutterS[1], "NO_MOVE", "No move", ISS_OFF);
    IUFillSwitchVector(&ParkOnShutterSP, ParkOnShutterS, NARRAY(ParkOnShutterS), getDeviceName(),
                       "PARK_ON_SHUTTER", "Park when operating shutter", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Shutter mode
    IUFillSwitch(&ShutterModeS[0], "FULL", "Open full", ISS_ON);
    IUFillSwitch(&ShutterModeS[1], "UPPER", "Open upper only", ISS_OFF);
    IUFillSwitchVector(&ShutterModeSP, ShutterModeS, NARRAY(ShutterModeS), getDeviceName(),
                       "SHUTTER_MODE", "Shutter open mode", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Home - Home command
    IUFillSwitch(&HomeS[0], "HOME", "Home", ISS_OFF);
    IUFillSwitchVector(&HomeSP, HomeS, NARRAY(HomeS), getDeviceName(),
                       "HOME_MOTION", "Home dome", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // Watch Dog
    IUFillNumber(&WatchDogN[0], "WATCH_DOG_TIME", "Watch dog time", "%5.2f", 0., 3600., 0., 0.);
    IUFillNumberVector(&WatchDogNP, WatchDogN, NARRAY(WatchDogN), getDeviceName(),
                       "WATCH_DOG_TIME_SET", "Watch dog time (seconds)", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    SetParkDataType(PARK_AZ);
    addAuxControls();

    // Set default baud rate to 19200
    serialConnection->setDefaultBaudRate(Connection::Serial::B_19200);

    return true;
}

bool MaxDomeII::updateProperties()
{
    INDI::Dome::updateProperties();

    if (isConnected()) {
        defineNumber(&HomeAzimuthNP);
        defineNumber(&TicksPerTurnNP);
        defineSwitch(&ParkOnShutterSP);
        defineSwitch(&ShutterModeSP);
        defineSwitch(&HomeSP);
        defineNumber(&WatchDogNP);

        SetupParms();
    } else {
        deleteProperty(HomeAzimuthNP.name);
        deleteProperty(TicksPerTurnNP.name);
        deleteProperty(ParkOnShutterSP.name);
        deleteProperty(ShutterModeSP.name);
        deleteProperty(HomeSP.name);
        deleteProperty(WatchDogNP.name);
    }

    return true;
}

bool MaxDomeII::saveConfigItems(FILE *fp)
{
    IUSaveConfigNumber(fp, &HomeAzimuthNP);
    IUSaveConfigNumber(fp, &TicksPerTurnNP);
    IUSaveConfigNumber(fp, &WatchDogNP);
    IUSaveConfigSwitch(fp, &ParkOnShutterSP);
    IUSaveConfigSwitch(fp, &ShutterModeSP);

    return INDI::Dome::saveConfigItems(fp);
}

bool MaxDomeII::Disconnect()
{
    driver.Disconnect();

    return INDI::Dome::Disconnect();
}

void MaxDomeII::TimerHit()
{
    ShStatus shutterSt;
    AzStatus nAzimuthStatus;
    unsigned nHomePosition;
    float nAz;
    int nError;
    int nRetry = 1;

    if (isConnected() == false)
        return; //  No need to reset timer if we are not connected anymore

    nError = driver.Status(&shutterSt, &nAzimuthStatus, &nCurrentTicks, &nHomePosition);
    handle_driver_error(&nError, &nRetry); // This is a timer, we will not repeat in order to not delay the execution.

    // Increment movment time counter
    if (nTimeSinceShutterStart >= 0)
        nTimeSinceShutterStart++;
    if (nTimeSinceAzimuthStart >= 0)
        nTimeSinceAzimuthStart++;

    // Watch dog
    nTimeSinceLastCommunication++;
    if (WatchDogNP.np[0].value > 0 && WatchDogNP.np[0].value <= nTimeSinceLastCommunication) {
        // Close Shutter if it is not
        if (shutterSt != SS_CLOSED) {
            DomeShutterSP.s = ControlShutter(SHUTTER_CLOSE);
            IDSetSwitch(&DomeShutterSP, "Closing shutter due to watch dog");
        }
    }

    if (getWeatherState() == IPS_ALERT) {
        // Close Shutter if it is not
        if (shutterSt != SS_CLOSED) {
            DomeShutterSP.s = ControlShutter(SHUTTER_CLOSE);
            IDSetSwitch(&DomeShutterSP, "Closing shutter due to weather conditions");
        }
    }

    if (!nError) {
        // Shutter
        switch (shutterSt) {
        case SS_CLOSED:
            if (DomeShutterS[1].s == ISS_ON) { // Close Shutter
                if (DomeShutterSP.s == IPS_BUSY || DomeShutterSP.s == IPS_ALERT) {
                    DomeShutterSP.s        = IPS_OK; // Shutter close movement ends.
                    nTimeSinceShutterStart = -1;
                    IDSetSwitch(&DomeShutterSP, "Shutter is closed");
                }
            } else {
                if (nTimeSinceShutterStart >= 0) {
                    // A movement has started. Warn but don't change
                    if (nTimeSinceShutterStart >= 4) {
                        DomeShutterSP.s = IPS_ALERT; // Shutter close movement ends.
                        IDSetSwitch(&DomeShutterSP, "Shutter still closed");
                    }
                } else {
                    // For some reason (manual operation?) the shutter has closed
                    DomeShutterSP.s   = IPS_IDLE;
                    DomeShutterS[1].s = ISS_ON;
                    DomeShutterS[0].s = ISS_OFF;
                    IDSetSwitch(&DomeShutterSP, "Unexpected shutter closed");
                }
            }
            break;
        case SS_OPENING:
            if (DomeShutterS[0].s == ISS_OFF) { // not opening Shutter
                // For some reason the shutter is opening (manual operation?)
                DomeShutterSP.s   = IPS_ALERT;
                DomeShutterS[1].s = ISS_OFF;
                DomeShutterS[0].s = ISS_OFF;
                IDSetSwitch(&DomeShutterSP, "Unexpected shutter opening");
            } else if (nTimeSinceShutterStart < 0) {
                // For some reason the shutter is opening (manual operation?)
                DomeShutterSP.s        = IPS_ALERT;
                nTimeSinceShutterStart = 0;
                IDSetSwitch(&DomeShutterSP, "Unexpected shutter opening");
            } else if (DomeShutterSP.s == IPS_ALERT) {
                // The alert has corrected
                DomeShutterSP.s = IPS_BUSY;
                IDSetSwitch(&DomeShutterSP, "Shutter is opening");
            }
            break;
        case SS_OPEN:
            if (DomeShutterS[0].s == ISS_ON) { // Open Shutter
                if (DomeShutterSP.s == IPS_BUSY || DomeShutterSP.s == IPS_ALERT) {
                    DomeShutterSP.s        = IPS_OK; // Shutter open movement ends.
                    nTimeSinceShutterStart = -1;
                    IDSetSwitch(&DomeShutterSP, "Shutter is open");
                }
            } else {
                if (nTimeSinceShutterStart >= 0) {
                    // A movement has started. Warn but don't change
                    if (nTimeSinceShutterStart >= 4) {
                        DomeShutterSP.s = IPS_ALERT; // Shutter open movement alert.
                        IDSetSwitch(&DomeShutterSP, "Shutter still open");
                    }
                } else {
                    // For some reason (manual operation?) the shutter has open
                    DomeShutterSP.s   = IPS_IDLE;
                    DomeShutterS[1].s = ISS_ON;
                    DomeShutterS[0].s = ISS_OFF;
                    IDSetSwitch(&DomeShutterSP, "Unexpected shutter open");
                }
            }
            break;
        case SS_CLOSING:
            if (DomeShutterS[1].s == ISS_OFF) { // Not closing Shutter
                // For some reason the shutter is closing (manual operation?)
                DomeShutterSP.s   = IPS_ALERT;
                DomeShutterS[1].s = ISS_ON;
                DomeShutterS[0].s = ISS_OFF;
                IDSetSwitch(&DomeShutterSP, "Unexpected shutter closing");
            } else if (nTimeSinceShutterStart < 0) {
                // For some reason the shutter is opening (manual operation?)
                DomeShutterSP.s        = IPS_ALERT;
                nTimeSinceShutterStart = 0;
                IDSetSwitch(&DomeShutterSP, "Unexpected shutter closing");
            } else if (DomeShutterSP.s == IPS_ALERT) {
                // The alert has corrected
                DomeShutterSP.s = IPS_BUSY;
                IDSetSwitch(&DomeShutterSP, "Shutter is closing");
            }
            break;
        case SS_ERROR:
            DomeShutterSP.s   = IPS_ALERT;
            DomeShutterS[1].s = ISS_OFF;
            DomeShutterS[0].s = ISS_OFF;
            IDSetSwitch(&DomeShutterSP, "Shutter error");
            break;
        case SS_ABORTED:
        default:
            if (nTimeSinceShutterStart >= 0) {
                DomeShutterSP.s        = IPS_ALERT; // Shutter movement aborted.
                DomeShutterS[1].s      = ISS_OFF;
                DomeShutterS[0].s      = ISS_OFF;
                nTimeSinceShutterStart = -1;
                IDSetSwitch(&DomeShutterSP, "Unknown shutter status");
            }
            break;
        }

        // Azimuth
        nAz = TicksToAzimuth(nCurrentTicks);
        if (DomeAbsPosN[0].value != nAz) {  // Only refresh position if it has changed
            DomeAbsPosN[0].value = nAz;
            IDSetNumber(&DomeAbsPosNP, nullptr);
        }

        switch (nAzimuthStatus) {
        case AS_IDLE:
        case AS_IDLE2:
            if (nTimeSinceAzimuthStart > 3) {
                if (nTargetAzimuth >= 0 &&
                        AzimuthDistance(nTargetAzimuth, nCurrentTicks) > 3) { // Maximum difference allowed: 3 ticks
                    DomeAbsPosNP.s         = IPS_ALERT;
                    nTimeSinceAzimuthStart = -1;
                    IDSetNumber(&DomeAbsPosNP, "Could not position right");
                } else {
                    // Succesfull end of movement
                    if (DomeAbsPosNP.s != IPS_OK) {
                        int domeState = getDomeState();
                        if (domeState == DOME_PARKING) {
                            SetParked(true);
                        } else if (domeState == DOME_UNPARKING) {
                            SetParked(false);
                            LOG_INFO("Dome is unparked");
                        } else {
                            setDomeState(DOME_SYNCED);
                            LOG_INFO("Dome is on target position");
                        }
                        nTimeSinceAzimuthStart = -1;
                    }
                    if (HomeS[0].s == ISS_ON) {
                        HomeS[0].s             = ISS_OFF;
                        HomeSP.s               = IPS_OK;
                        nTimeSinceAzimuthStart = -1;
                        IDSetSwitch(&HomeSP, "Dome is homed");
                    }
                }
            }
            break;
        case AS_MOVING_WE:
        case AS_MOVING_EW:
            if (nTimeSinceAzimuthStart < 0) {
                nTimeSinceAzimuthStart = 0;
                nTargetAzimuth         = -1;
                DomeAbsPosNP.s         = IPS_ALERT;
                IDSetNumber(&DomeAbsPosNP, "Unexpected dome moving");
            }
            break;
        case AS_ERROR:
            if (nTimeSinceAzimuthStart >= 0) {
                DomeAbsPosNP.s         = IPS_ALERT;
                nTimeSinceAzimuthStart = -1;
                nTargetAzimuth         = -1;
                IDSetNumber(&DomeAbsPosNP, "Dome Error");
            }
        default:
            break;
        }
    } else {
        LOGF_DEBUG("Error: %s. Please reconnect and try again.", ErrorMessages[-nError]);
        return;
    }

    SetTimer(POLLMS);
    return;
}

IPState MaxDomeII::MoveAbs(double newAZ)
{
    double currAZ = 0;
    int newPos = 0, nDir = 0;
    int error;
    int nRetry = 3;

    currAZ = DomeAbsPosN[0].value;

    // Take the shortest path
    if (newAZ > currAZ) {
        if (newAZ - currAZ > 180.0)
            nDir = MAXDOMEII_WE_DIR;
        else
            nDir = MAXDOMEII_EW_DIR;
    } else {
        if (currAZ - newAZ > 180.0)
            nDir = MAXDOMEII_EW_DIR;
        else
            nDir = MAXDOMEII_WE_DIR;
    }

    newPos = AzimuthToTicks(newAZ);

    while (nRetry) {
        error = driver.GotoAzimuth(nDir, newPos);
        handle_driver_error(&error, &nRetry);
    }

    if (error != 0)
        return IPS_ALERT;

    nTargetAzimuth = newPos;
    nTimeSinceAzimuthStart = 0; // Init movement timer

    // It will take a few cycles to reach final position
    return IPS_BUSY;
}

IPState MaxDomeII::Move(DomeDirection dir, DomeMotionCommand operation)
{
    int error;
    int nRetry = 3;

    if (operation == MOTION_START) {
        LOGF_DEBUG("Move dir=%d", dir);
        double currAZ = DomeAbsPosN[0].value;
        double newAZ = currAZ > 180 ? currAZ - 180 : currAZ + 180;
        int newPos = AzimuthToTicks(newAZ);
        int nDir = dir ? MAXDOMEII_WE_DIR : MAXDOMEII_EW_DIR;

        while (nRetry) {
            error = driver.GotoAzimuth(nDir, newPos);
            handle_driver_error(&error, &nRetry);
        }

        if (error != 0)
            return IPS_ALERT;

        nTargetAzimuth = newPos;
        nTimeSinceAzimuthStart = 0; // Init movement timer
        return IPS_BUSY;
    } else {
        LOG_DEBUG("Stop movement");
        while (nRetry) {
            error = driver.AbortAzimuth();
            handle_driver_error(&error, &nRetry);
        }

        if (error != 0)
            return IPS_ALERT;

        DomeAbsPosNP.s = IPS_IDLE;
        IDSetNumber(&DomeAbsPosNP, NULL);
        nTimeSinceAzimuthStart = -1;
    }

    return IPS_OK;
}

bool MaxDomeII::Abort()
{
    int error  = 0;
    int nRetry = 3;

    while (nRetry) {
        error = driver.AbortAzimuth();
        handle_driver_error(&error, &nRetry);
    }

    while (nRetry) {
        error = driver.AbortShutter();
        handle_driver_error(&error, &nRetry);
    }

    DomeAbsPosNP.s = IPS_IDLE;
    IDSetNumber(&DomeAbsPosNP, nullptr);

    // If we abort while in the middle of opening/closing shutter, alert.
    if (DomeShutterSP.s == IPS_BUSY) {
        DomeShutterSP.s = IPS_ALERT;
        IDSetSwitch(&DomeShutterSP, "Shutter operation aborted.");
        return false;
    }

    return true;
}


bool MaxDomeII::OnSetTicksPerTurn(double value)
{
    int error;
    int nRetry = 3;

    if (value < 100 || value > 2000) {
        TicksPerTurnNP.s = IPS_ALERT;
        IDSetNumber(&TicksPerTurnNP, "Invalid Ticks Per Turn");
        return false;
    }

    while (nRetry) {
        error = driver.SetTicksPerTurn(value);
        handle_driver_error(&error, &nRetry);
    }
    if (error < 0) {
        LOGF_ERROR("MAX DOME II: %s", ErrorMessages[-error]);
        TicksPerTurnNP.s = IPS_ALERT;
        IDSetNumber(&TicksPerTurnNP, nullptr);
        return false;
    }

    char cLog[255];
    sprintf(cLog, "New Ticks Per Turn set: %lf", value);
    nTicksPerTurn = value;
    nHomeTicks = floor(0.5 + nHomeAzimuth * nTicksPerTurn / 360.0); // Calculate Home ticks again
    TicksPerTurnNP.s = IPS_OK;
    TicksPerTurnNP.np[0].value = value;
    IDSetNumber(&TicksPerTurnNP, "%s", cLog);
    return true;
}


bool MaxDomeII::OnSetHomeAzimuth(double value)
{
    if (value < 0 || value > 360) {
        HomeAzimuthNP.s = IPS_ALERT;
        IDSetNumber(&HomeAzimuthNP, "Invalid home azimuth");
        return false;
    }

    char cLog[255];
    sprintf(cLog, "New home azimuth set: %lf", value);
    nHomeAzimuth = value;
    nHomeTicks = floor(0.5 + nHomeAzimuth * nTicksPerTurn / 360.0);
    HomeAzimuthNP.s = IPS_OK;
    HomeAzimuthNP.np[0].value = value;
    IDSetNumber(&HomeAzimuthNP, "%s", cLog);
    return true;
}


bool MaxDomeII::OnSetWatchdog(double value)
{
    if (value < 0 || value > 3600) {
        WatchDogNP.s = IPS_ALERT;
        IDSetNumber(&WatchDogNP, "Invalid watch dog time");
        return false;
    }

    char cLog[255];
    sprintf(cLog, "New watch dog set: %lf", value);
    WatchDogNP.s = IPS_OK;
    WatchDogNP.np[0].value = value;
    IDSetNumber(&WatchDogNP, "%s", cLog);
    return true;
}

/**************************************************************************************
**
***************************************************************************************/
bool MaxDomeII::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    // Ignore if not ours
    if (strcmp(dev, getDeviceName()))
        return false;

    nTimeSinceLastCommunication = 0;

    // Ticks per turn
    if (!strcmp(name, TicksPerTurnNP.name)) {
        if (IUUpdateNumber(&TicksPerTurnNP, values, names, n) < 0)
            return false;
        return OnSetTicksPerTurn(values[0]);
    }

    // Home azimuth
    if (!strcmp(name, HomeAzimuthNP.name)) {
        if (IUUpdateNumber(&HomeAzimuthNP, values, names, n) < 0)
            return false;
        return OnSetHomeAzimuth(values[0]);
    }

    // Watch dog
    if (!strcmp(name, WatchDogNP.name)) {
        if (IUUpdateNumber(&WatchDogNP, values, names, n) < 0)
            return false;
        return OnSetWatchdog(values[0]);
    }

    return INDI::Dome::ISNewNumber(dev, name, values, names, n);
}

bool MaxDomeII::OnHomeSwitch()
{
    int error;
    int nRetry = 3;

    while (nRetry) {
        error = driver.HomeAzimuth();
        handle_driver_error(&error, &nRetry);
    }
    nTimeSinceAzimuthStart = 0;
    nTargetAzimuth = -1;
    if (error) {
        LOGF_ERROR("Error Homing Azimuth (%s).", ErrorMessages[-error]);
        HomeSP.s = IPS_ALERT;
        IDSetSwitch(&HomeSP, "Error Homing Azimuth");
        return false;
    }
    HomeSP.s = IPS_BUSY;
    IDSetSwitch(&HomeSP, "Homing dome");

    return true;
}

bool MaxDomeII::OnSetParkOnShutter(int csbp)
{
    int pos = floor(0.5 + GetAxis1Park() * nTicksPerTurn / 360.0);
    int error = ConfigurePark(csbp, pos);

    if (error == IPS_OK) {
        ParkOnShutterSP.s = IPS_OK;
        IDSetSwitch(&ParkOnShutterSP, "New shutter operation conflict set");
    } else {
        ParkOnShutterSP.s = IPS_ALERT;
        IDSetSwitch(&ParkOnShutterSP, "%s", ErrorMessages[-error]);
    }
    return true;
}

bool MaxDomeII::OnSetShutterOpeningMode()
{
    ShutterModeSP.s = IPS_OK;
    IDSetSwitch(&ShutterModeSP, "Shutter opening mode set");
    return true;
}

/**************************************************************************************
**
***************************************************************************************/
bool MaxDomeII::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    // Ignore if not ours
    if (strcmp(dev, getDeviceName()))
        return false;

    nTimeSinceLastCommunication = 0;

    // Home
    if (!strcmp(name, HomeSP.name)) {
        if (IUUpdateSwitch(&HomeSP, states, names, n) < 0)
            return false;
        return OnHomeSwitch();
    }

    // Park on shutter
    if (!strcmp(name, ParkOnShutterSP.name)) {
        if (IUUpdateSwitch(&ParkOnShutterSP, states, names, n) < 0)
            return false;
        return OnSetParkOnShutter(ParkOnShutterS[0].s == ISS_ON ? 1 : 0);
    }

    // Sutter opening mode
    if (!strcmp(name, ShutterModeSP.name)) {
        if (IUUpdateSwitch(&ShutterModeSP, states, names, n) < 0)
            return false;
        return OnSetShutterOpeningMode();
    }

    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
**
***************************************************************************************/
int MaxDomeII::AzimuthDistance(int nPos1, int nPos2)
{
    int nDif = std::abs(nPos1 - nPos2);

    if (nDif > nTicksPerTurn / 2)
        nDif = nTicksPerTurn - nDif;

    return nDif;
}

/**************************************************************************************
 **
 ***************************************************************************************/
double MaxDomeII::TicksToAzimuth(int nTicks)
{
    double nAz;

    nAz = nHomeAzimuth + nTicks * 360.0 / nTicksPerTurn;
    while (nAz < 0)
        nAz += 360;
    while (nAz >= 360)
        nAz -= 360;

    return nAz;
}
/**************************************************************************************
 **
 ***************************************************************************************/
int MaxDomeII::AzimuthToTicks(double nAzimuth)
{
    int nTicks;

    nTicks = floor(0.5 + (nAzimuth - nHomeAzimuth) * nTicksPerTurn / 360.0);
    while (nTicks > nTicksPerTurn)
        nTicks -= nTicksPerTurn;
    while (nTicks < 0)
        nTicks += nTicksPerTurn;

    return nTicks;
}

/**************************************************************************************
 **
 ***************************************************************************************/

int MaxDomeII::handle_driver_error(int *error, int *nRetry)
{
    (*nRetry)--;
    switch (*error) {
    case 0: // No error. Continue
        *nRetry = 0;
        break;
    case -5: // can't connect
        // This error can happen if port connection is lost, i.e. a usb-serial reconnection
        // Reconnect
        LOG_ERROR("MAX DOME II: Reconnecting ...");
        Connect();
        if (PortFD < 0)
            *nRetry = 0; // Can't open the port. Don't retry anymore.
        break;

    default: // Do nothing in all other errors.
        LOGF_ERROR("Error on command: (%s).", ErrorMessages[-*error]);
        break;
    }

    return *nRetry;
}

/************************************************************************************
*
* ***********************************************************************************/
IPState MaxDomeII::ConfigurePark(int csbp, int parkingPos)
{
    int error  = 0;
    int nRetry = 3;

    while (nRetry) {
        error = driver.SetPark(csbp, parkingPos);
        handle_driver_error(&error, &nRetry);
    }

    if (error >= 0) {
        nCloseShutterBeforePark = csbp;
        LOGF_INFO("Close shutter before parking: %s", csbp ? "yes" : "no");
        LOGF_INFO("New park position set: %d ticks", parkingPos);
    } else {
        LOGF_ERROR("MAX DOME II: %s", ErrorMessages[-error]);
        return IPS_ALERT;
    }
    return IPS_OK;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool MaxDomeII::SetCurrentPark()
{
    // Set current angle as parking position
    SetAxis1Park(DomeAbsPosN[0].value);

    int pos = floor(0.5 + GetAxis1Park() * nTicksPerTurn / 360.0);
    IPState error = ConfigurePark(nCloseShutterBeforePark, pos);
    return error == IPS_OK;
}
/************************************************************************************
 *
* ***********************************************************************************/

bool MaxDomeII::SetDefaultPark()
{
    // Set default parking position
    SetAxis1Park(GetAxis1ParkDefault());

    int pos = floor(0.5 + GetAxis1Park() * nTicksPerTurn / 360.0);
    IPState error = ConfigurePark(nCloseShutterBeforePark, pos);
    return error == IPS_OK;
}

/************************************************************************************
 *
* ***********************************************************************************/
IPState MaxDomeII::Park()
{
    DomeShutterSP.s = ControlShutter(SHUTTER_CLOSE);
    IDSetSwitch(&DomeShutterSP, "Closing shutter while parking");

    nTargetAzimuth = GetAxis1Park();
    return MoveAbs(nTargetAzimuth);
}

/************************************************************************************
 *
* ***********************************************************************************/
IPState MaxDomeII::UnPark()
{
    return IPS_OK;
}

/************************************************************************************
 *
* ***********************************************************************************/
IPState MaxDomeII::ControlShutter(ShutterOperation operation)
{
    int error  = 0;
    int nRetry = 3;

    if (operation == SHUTTER_CLOSE) {
        while (nRetry) {
            error = driver.CloseShutter();
            handle_driver_error(&error, &nRetry);
        }
        nTimeSinceShutterStart = 0; // Init movement timer
        if (error) {
            LOGF_ERROR("Error closing shutter (%s).", ErrorMessages[-error]);
            return IPS_ALERT;
        }
        return IPS_BUSY;
    } else {
        if (ShutterModeS[0].s == ISS_ON) {
            // Open Shutter
            while (nRetry) {
                error = driver.OpenShutter();
                handle_driver_error(&error, &nRetry);
            }
            nTimeSinceShutterStart = 0; // Init movement timer
            if (error) {
                LOGF_ERROR("Error opening shutter (%s).", ErrorMessages[-error]);
                return IPS_ALERT;
            }
            return IPS_BUSY;
        } else {
            // Open upper shutter only
            while (nRetry) {
                error = driver.OpenUpperShutterOnly();
                handle_driver_error(&error, &nRetry);
            }
            nTimeSinceShutterStart = 0; // Init movement timer
            if (error) {
                LOGF_ERROR("Error opening upper shutter only (%s).", ErrorMessages[-error]);
                return IPS_ALERT;
            }
            return IPS_BUSY;
        }
    }

    return IPS_ALERT;
}
