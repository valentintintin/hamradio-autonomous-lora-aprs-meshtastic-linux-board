#include <pgmspace.h>
#include <EEPROM.h>
#include "ArduinoLog.h"

#include "Command.h"
#include "System.h"

System* Command::system;

Command::Command(System *system) {
    Command::system = system;

    parser.registerCommand(PSTR("relay1"), PSTR("u"), doRelay1);
    parser.registerCommand(PSTR("relay2"), PSTR("u"), doRelay2);
    parser.registerCommand(PSTR("position"), PSTR(""), doPosition);
    parser.registerCommand(PSTR("telem"), PSTR(""), doTelemetry);
    parser.registerCommand(PSTR("telemParams"), PSTR(""), doTelemetryParams);
    parser.registerCommand(PSTR("dog"), PSTR("u"), doMpptWatchdog);
    parser.registerCommand(PSTR("pow"), PSTR("uu"), doMpptPower);
    parser.registerCommand(PSTR("lora"), PSTR("s"), doLora);
    parser.registerCommand(PSTR("reset"), PSTR(""), doReset);
    parser.registerCommand(PSTR("set"), PSTR("uu"), doSetEeprom);
    parser.registerCommand(PSTR("time"), PSTR("u"), doSetTime);
    parser.registerCommand(PSTR("sleep"), PSTR("u"), doSleep);
    parser.registerCommand(PSTR("jsons"), PSTR(""), doGetJson);
    parser.registerCommand(PSTR("ping"), PSTR(""), doPing);
}

bool Command::processCommand(const char *command) {
    if (strlen(command) < 3) {
        Log.traceln(F("[COMMAND] Command received length %d : %s"), strlen(command), command);
        return false;
    }

    Log.traceln(F("[COMMAND] Process : %s"), command);

    if (!parser.processCommand(command, response)) {
        return false;
    }

    Log.infoln(F("[COMMAND] %s = %s"), command, response);

    return true;
}

void Command::doRelay1(MyCommandParser::Argument *args, char *response) {
    bool state = args[0].asInt64 > 0;

    system->gpio.setRelay1(state);

    sprintf_P(response, PSTR("OK state %d"), state);
}

void Command::doRelay2(MyCommandParser::Argument *args, char *response) {
    bool state = args[0].asInt64 > 0;

    system->gpio.setRelay2(state);

    sprintf_P(response, PSTR("OK state %d"), state);
}

void Command::doTelemetry(MyCommandParser::Argument *args, char *response) {
    system->forceSendTelemetry = true;

    sprintf_P(response, PSTR("OK"));
}

void Command::doPosition(MyCommandParser::Argument *args, char *response) {
    system->forceSendPosition = true;

    sprintf_P(response, PSTR("OK"));
}

void Command::doTelemetryParams(MyCommandParser::Argument *args, char *response) {
    system->communication->shouldSendTelemetryParams = true;
    system->forceSendTelemetry = true;

    sprintf_P(response, PSTR("OK"));
}

void Command::doMpptWatchdog(MyCommandParser::Argument *args, char *response) {
    uint64_t watchdog = args[0].asUInt64;

    bool ok = system->mpptMonitor.setWatchdog(watchdog);
    system->mpptMonitor.watchdogManagedByUser = true;

    sprintf_P(response, ok ? PSTR("OK") : PSTR("KO"));
}

void Command::doLora(MyCommandParser::Argument *args, char *response) {
    char *message = args[0].asString;

    // todo remake
    system->communication->sendMessage(PSTR(APRS_DESTINATION), message);

    sprintf_P(response, PSTR("OK"));
}

void Command::doReset(MyCommandParser::Argument *args, char *response) {
    CySoftwareReset();

    sprintf_P(response, PSTR("OK"));
}

void Command::doMpptPower(MyCommandParser::Argument *args, char *response) {
    uint64_t powerOnVoltage = args[0].asUInt64;
    uint64_t powerOffVoltage = args[1].asUInt64;

    bool ok = system->mpptMonitor.setPowerOnOff(powerOnVoltage, powerOffVoltage);

    sprintf_P(response, ok ? PSTR("OK") : PSTR("KO"));
}

void Command::doSetEeprom(MyCommandParser::Argument *args, char *response) {
    byte function = (byte) args[0].asUInt64;
    bool allowed = args[1].asUInt64 == 1;

    system->setFunctionAllowed(function, allowed);

    sprintf_P(response, EEPROM.read((int const) args[0].asUInt64) ? PSTR("OK") : PSTR("KO"));
}

void Command::doSetTime(MyCommandParser::Argument *args, char *response) {
    uint64_t epoch = args[0].asUInt64;

    system->RTC.setEpoch((long long) epoch, true);
    system->setTimeFromRTcToInternalRtc(epoch);

    System::nowToString(response);
}

void Command::doSleep(MyCommandParser::Argument *args, char *response) {
    uint64_t time = args[0].asUInt64;

    if (time >= 1) {
        system->sleep(time * 1000);
        sprintf_P(response, PSTR("OK"));
        return;
    }

    sprintf_P(response, PSTR("KO"));
}

void Command::doGetJson(MyCommandParser::Argument *args, char *response) {
    system->resetTimerJson();
    sprintf_P(response, PSTR("OK"));
}

void Command::doPing(MyCommandParser::Argument *args, char *response) {
    sprintf_P(response, PSTR("Pong!"));
}
