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


    logconf = LogConfig(name='pwm', period_in_ms=100)
    logvars = [
        LogVaraible('pwm.m1_pwm', 'uint32_t'),
        LogVaraible('pwm.m2_pwm', 'uint32_t'),
        LogVaraible('pwm.m3_pwm', 'uint32_t'),
        LogVaraible('pwm.m4_pwm', 'uint32_t'),
    ]
    loggers.append(FileLogger(logconf, 'pwm.csv', logvars))

    logconf = LogConfig(name='radio', period_in_ms=100)
    logvars = [
        LogVaraible('radio.rssi', 'uint8_t'),
        LogVaraible('radio.isConnected', 'uint8_t'),
    ]
    loggers.append(FileLogger(logconf, 'radio.csv', logvars))

    logconf = LogConfig(name='range', period_in_ms=100)
    logvars = [
        LogVaraible('range.front', 'uint16_t'),
        LogVaraible('range.back', 'uint16_t'),
        LogVaraible('range.up', 'uint16_t'),
        LogVaraible('range.left', 'uint16_t'),
        LogVaraible('range.right', 'uint16_t'),
        LogVaraible('range.zrange', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'range.csv', logvars))

    logconf = LogConfig(name='ranging', period_in_ms=100)
    logvars = [
        LogVaraible('ranging.state', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'ranging.csv', logvars))

    logconf = LogConfig(name='ring', period_in_ms=100)
    logvars = [
        LogVaraible('ring.fadeTime', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ring.csv', logvars))

    logconf = LogConfig(name='sensfusion6_1', period_in_ms=100)
    logvars = [
        LogVaraible('sensfusion6.qw', 'float'),
        LogVaraible('sensfusion6.qx', 'float'),
        LogVaraible('sensfusion6.qy', 'float'),
        LogVaraible('sensfusion6.qz', 'float'),
        LogVaraible('sensfusion6.gravityX', 'float'),
        LogVaraible('sensfusion6.gravityY', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'sensfusion6_1.csv', logvars))

    logconf = LogConfig(name='sensfusion6_2', period_in_ms=100)
    logvars = [
        LogVaraible('sensfusion6.gravityZ', 'float'),
        LogVaraible('sensfusion6.accZbase', 'float'),
        LogVaraible('sensfusion6.isInit', 'uint8_t'),
        LogVaraible('sensfusion6.isCalibrated', 'uint8_t'),
    ]
    loggers.append(FileLogger(logconf, 'sensfusion6_2.csv', logvars))

    logconf = LogConfig(name='stabilizer', period_in_ms=100)
    logvars = [
        LogVaraible('stabilizer.roll', 'float'),
        LogVaraible('stabilizer.pitch', 'float'),
        LogVaraible('stabilizer.yaw', 'float'),
        LogVaraible('stabilizer.thrust', 'float'),
        LogVaraible('stabilizer.rtStab', 'float'),
        LogVaraible('stabilizer.intToOut', 'uint32_t'),
    ]
    loggers.append(FileLogger(logconf, 'stabilizer.csv', logvars))

    logconf = LogConfig(name='stateEstimate1', period_in_ms=100)
    logvars = [
        LogVaraible('stateEstimate.x', 'float'),
        LogVaraible('stateEstimate.y', 'float'),
        LogVaraible('stateEstimate.z', 'float'),
        LogVaraible('stateEstimate.vx', 'float'),
        LogVaraible('stateEstimate.vy', 'float'),
        LogVaraible('stateEstimate.vz', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'stateEstimate1.csv', logvars))

    logconf = LogConfig(name='stateEstimate2', period_in_ms=100)
    logvars = [
        LogVaraible('stateEstimate.ax', 'float'),
        LogVaraible('stateEstimate.ay', 'float'),
        LogVaraible('stateEstimate.az', 'float'),
        LogVaraible('stateEstimate.roll', 'float'),
        LogVaraible('stateEstimate.pitch', 'float'),
        LogVaraible('stateEstimate.yaw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'stateEstimate2.csv', logvars))

    logconf = LogConfig(name='stateEstimate3', period_in_ms=100)
    logvars = [
        LogVaraible('stateEstimate.qx', 'float'),
        LogVaraible('stateEstimate.qy', 'float'),
        LogVaraible('stateEstimate.qz', 'float'),
        LogVaraible('stateEstimate.qw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'stateEstimate3.csv', logvars))

    logconf = LogConfig(name='stateEstimateZ1', period_in_ms=100)
    logvars = [
        LogVaraible('stateEstimateZ.x', 'int16_t'),
        LogVaraible('stateEstimateZ.y', 'int16_t'),
        LogVaraible('stateEstimateZ.z', 'int16_t'),
        LogVaraible('stateEstimateZ.vx', 'int16_t'),
        LogVaraible('stateEstimateZ.vy', 'int16_t'),
        LogVaraible('stateEstimateZ.vz', 'int16_t'),
    ]
    loggers.append(FileLogger(logconf, 'stateEstimateZ1.csv', logvars))

    logconf = LogConfig(name='stateEstimateZ2', period_in_ms=100)
    logvars = [
        LogVaraible('stateEstimateZ.ax', 'int16_t'),
        LogVaraible('stateEstimateZ.ay', 'int16_t'),
        LogVaraible('stateEstimateZ.az', 'int16_t'),
        LogVaraible('stateEstimateZ.quat', 'uint32_t'),
        LogVaraible('stateEstimateZ.rateRoll', 'int16_t'),
        LogVaraible('stateEstimateZ.ratePitch', 'int16_t'),
        LogVaraible('stateEstimateZ.rateYaw', 'int16_t'),
    ]
    loggers.append(FileLogger(logconf, 'stateEstimateZ2.csv', logvars))

    logconf = LogConfig(name='sys', period_in_ms=100)
    logvars = [
        LogVaraible('sys.canfly', 'uint8_t'),
        LogVaraible('sys.isFlying', 'uint8_t'),
        LogVaraible('sys.isTumbled', 'uint8_t'),
        LogVaraible('sys.armed', 'int8_t'),
    ]
    loggers.append(FileLogger(logconf, 'sys.csv', logvars))

    logconf = LogConfig(name='usd', period_in_ms=100)
    logvars = [
        LogVaraible('usd.spiWrBps', 'float'),
        LogVaraible('usd.spiReBps', 'float'),
        LogVaraible('usd.fatWrBps', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'usd.csv', logvars))

    logconf = LogConfig(name='twr', period_in_ms=100)
    logvars = [
        LogVaraible('twr.rangingSuccessRate0', 'uint8_t'),
        LogVaraible('twr.rangingPerSec0', 'uint8_t'),
        LogVaraible('twr.rangingSuccessRate1', 'uint8_t'),
        LogVaraible('twr.rangingPerSec1', 'uint8_t'),
        LogVaraible('twr.rangingSuccessRate2', 'uint8_t'),
        LogVaraible('twr.rangingPerSec2', 'uint8_t'),
        LogVaraible('twr.rangingSuccessRate3', 'uint8_t'),
        LogVaraible('twr.rangingPerSec3', 'uint8_t'),
        LogVaraible('twr.rangingSuccessRate4', 'uint8_t'),
        LogVaraible('twr.rangingPerSec4', 'uint8_t'),
        LogVaraible('twr.rangingSuccessRate5', 'uint8_t'),
        LogVaraible('twr.rangingPerSec5', 'uint8_t'),
    ]
    loggers.append(FileLogger(logconf, 'twr.csv', logvars))

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
    