# -*- coding: utf-8 -*-
#
# Based on bitcraze example project:
# https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/step-by-step/sbs_motion_commander.py

from fileinput import filename
import logging
import time

import csv

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

logging.basicConfig(level=logging.ERROR)

# Log variables
class LogVaraible():
    def __init__(self, name, type):
        self.name = name
        self.type = type

# File logger
class FileLogger(object):
    def __init__(self, logConf, fileName, logVariables):
        self._logConf = logConf

        fieldnames = ['timestamp']
        for logVar in logVariables:
            self._logConf.add_variable(logVar.name, logVar.type)
            fieldnames.append(logVar.name)

        
        self._csvfile = open(fileName, 'w')
        self._data_writer = csv.DictWriter(self._csvfile, fieldnames=fieldnames)
        self._data_writer.writeheader()
            
        self._logConf.data_received_cb.add_callback(self._log_callback)

    def _log_callback(self, timestamp, data, logconf):
            data['timestamp']=timestamp
            self._data_writer.writerow(data)
            print(data)
    
    def start(self, crazyflie):
        crazyflie.log.add_config(self._logConf)
        self._logConf.start()

    def stop(self):
        self._logConf.stop()
        self._logConf.data_received_cb.remove_callback(self._log_callback)
        self._csvfile.close()    

def init_loggers():
    loggers = []

    logconf = LogConfig(name='extrx', period_in_ms=100)
    logvars = [
        LogVaraible('extrx.AltHold', 'uint8_t'),
        LogVaraible('extrx.Arm', 'uint8_t'),
        LogVaraible('extrx.thrust', 'float'),
        LogVaraible('extrx.roll', 'float'),
        LogVaraible('extrx.pitch', 'float'),
        LogVaraible('extrx.yawRate', 'float'),
        LogVaraible('extrx.zVel', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'extrx.csv', logvars))

    logconf = LogConfig(name='extrx_raw', period_in_ms=100)
    logvars = [
        LogVaraible('extrx_raw.ch0', 'uint16_t'),
        LogVaraible('extrx_raw.ch1', 'uint16_t'),
        LogVaraible('extrx_raw.ch2', 'uint16_t'),
        LogVaraible('extrx_raw.ch3', 'uint16_t'),
        LogVaraible('extrx_raw.ch4', 'uint16_t'),
        LogVaraible('extrx_raw.ch5', 'uint16_t'),
        LogVaraible('extrx_raw.ch6', 'uint16_t'),
        LogVaraible('extrx_raw.ch7', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'extrx_raw.csv', logvars))


    logconf = LogConfig(name='gyro1', period_in_ms=100)
    logvars = [
        LogVaraible('gyro.xVariance', 'float'),
        LogVaraible('gyro.yVariance', 'float'),
        LogVaraible('gyro.zVariance', 'float'),
        LogVaraible('gyro.x', 'float'),
        LogVaraible('gyro.y', 'float'),
        LogVaraible('gyro.z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'gyro1.csv', logvars))

    logconf = LogConfig(name='gyro2', period_in_ms=100)
    logvars = [
        LogVaraible('gyro.xRaw', 'int16_t'),
        LogVaraible('gyro.yRaw', 'int16_t'),
        LogVaraible('gyro.zRaw', 'int16_t'),
    ]
    loggers.append(FileLogger(logconf, 'gyro2.csv', logvars))

    logconf = LogConfig(name='health1', period_in_ms=100)
    logvars = [
        LogVaraible('health.motorVarXM1', 'float'),
        LogVaraible('health.motorVarYM1', 'float'),
        LogVaraible('health.motorVarXM2', 'float'),
        LogVaraible('health.motorVarYM2', 'float'),
        LogVaraible('health.motorVarXM3', 'float'),
        LogVaraible('health.motorVarYM3', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'health1.csv', logvars))

    logconf = LogConfig(name='health2', period_in_ms=100)
    logvars = [
        LogVaraible('health.motorVarXM4', 'float'),
        LogVaraible('health.motorVarYM4', 'float'),
        LogVaraible('health.motorPass', 'uint8_t'),
        LogVaraible('health.batterySag', 'float'),
        LogVaraible('health.batteryPass', 'uint8_t'),
        LogVaraible('health.motorTestCount', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'health2.csv', logvars))

    logconf = LogConfig(name='kalman1', period_in_ms=100)
    logvars = [
        LogVaraible('kalman.inFlight', 'uint8_t'),
        LogVaraible('kalman.stateX', 'float'),
        LogVaraible('kalman.stateY', 'float'),
        LogVaraible('kalman.stateZ', 'float'),
        LogVaraible('kalman.statePX', 'float'),
        LogVaraible('kalman.statePY', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'kalman1.csv', logvars))

    logconf = LogConfig(name='kalman2', period_in_ms=100)
    logvars = [
        LogVaraible('kalman.statePZ', 'float'),
        LogVaraible('kalman.stateD0', 'float'),
        LogVaraible('kalman.stateD1', 'float'),
        LogVaraible('kalman.stateD2', 'float'),
        LogVaraible('kalman.varX', 'float'),
        LogVaraible('kalman.varY', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'kalman2.csv', logvars))

    logconf = LogConfig(name='kalman3', period_in_ms=100)
    logvars = [
        LogVaraible('kalman.varZ', 'float'),
        LogVaraible('kalman.varPX', 'float'),
        LogVaraible('kalman.varPY', 'float'),
        LogVaraible('kalman.varPZ', 'float'),
        LogVaraible('kalman.varD0', 'float'),
        LogVaraible('kalman.varD1', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'kalman3.csv', logvars))

    logconf = LogConfig(name='kalman4', period_in_ms=100)
    logvars = [
        LogVaraible('kalman.varD2', 'float'),
        LogVaraible('kalman.q0', 'float'),
        LogVaraible('kalman.q1', 'float'),
        LogVaraible('kalman.q2', 'float'),
        LogVaraible('kalman.q3', 'float'),
        LogVaraible('kalman.rtUpdate', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'kalman4.csv', logvars))

    logconf = LogConfig(name='kalman5', period_in_ms=100)
    logvars = [
        LogVaraible('kalman.rtPred', 'float'),
        LogVaraible('kalman.rtFinal', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'kalman5.csv', logvars))

    logconf = LogConfig(name='kalman_pred', period_in_ms=100)
    logvars = [
        LogVaraible('kalman_pred.predNX', 'float'),
        LogVaraible('kalman_pred.predNY', 'float'),
        LogVaraible('kalman_pred.measNX', 'float'),
        LogVaraible('kalman_pred.measNY', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'kalman_pred.csv', logvars))

    logconf = LogConfig(name='lighthouse1', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.validAngles', 'uint8_t'),
        LogVaraible('lighthouse.rawAngle0x', 'float'),
        LogVaraible('lighthouse.rawAngle0y', 'float'),
        LogVaraible('lighthouse.rawAngle1x', 'float'),
        LogVaraible('lighthouse.rawAngle1y', 'float'),
        LogVaraible('lighthouse.angle0x', 'float'),
        LogVaraible('lighthouse.angle0y', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse1.csv', logvars))

    logconf = LogConfig(name='lighthouse2', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.angle1x', 'float'),
        LogVaraible('lighthouse.angle1y', 'float'),
        LogVaraible('lighthouse.angle0x_1', 'float'),
        LogVaraible('lighthouse.angle0y_1', 'float'),
        LogVaraible('lighthouse.angle1x_1', 'float'),
        LogVaraible('lighthouse.angle1y_1', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse2.csv', logvars))

    logconf = LogConfig(name='lighthouse3', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.angle0x_2', 'float'),
        LogVaraible('lighthouse.angle0y_2', 'float'),
        LogVaraible('lighthouse.angle1x_2', 'float'),
        LogVaraible('lighthouse.angle1y_2', 'float'),
        LogVaraible('lighthouse.angle0x_3', 'float'),
        LogVaraible('lighthouse.angle0y_3', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse3.csv', logvars))

    logconf = LogConfig(name='lighthouse4', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.angle1x_3', 'float'),
        LogVaraible('lighthouse.angle1y_3', 'float'),
        LogVaraible('lighthouse.rawAngle0xlh2', 'float'),
        LogVaraible('lighthouse.rawAngle0ylh2', 'float'),
        LogVaraible('lighthouse.rawAngle1xlh2', 'float'),
        LogVaraible('lighthouse.rawAngle1ylh2', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse4.csv', logvars))



    return loggers

def start_logging(cf, loggers):
    for logger in loggers:
        logger.start(cf)    

def stop_logging(loggers):
    for logger in loggers:
        logger.stop() 

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    
    with SyncCrazyflie(uri, cf=cf) as scf:
        loggers = init_loggers()
        start_logging(cf, loggers)

        with PositionHlCommander(scf, default_height=0.5, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            pc.go_to( x=0.0, y=0.0, velocity=0.3)
            pc.go_to( x=1.5, y=0.0, velocity=0.3)
            pc.land()
            time.sleep(5)
            pc.take_off( height=0.5, velocity=0.3)
            pc.go_to( x=0.0, y=0.0, velocity=0.3)
            pc.land()

        time.sleep(1)
        stop_logging(loggers)
    