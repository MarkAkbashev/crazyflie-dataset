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

    
    logconf = LogConfig(name='lighthouse5', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.angle0x_0lh2', 'float'),
        LogVaraible('lighthouse.angle0y_0lh2', 'float'),
        LogVaraible('lighthouse.angle1x_0lh2', 'float'),
        LogVaraible('lighthouse.angle1y_0lh2', 'float'),
        LogVaraible('lighthouse.serRt', 'float'),
        LogVaraible('lighthouse.frmRt', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse5.csv', logvars))

    logconf = LogConfig(name='lighthouse6', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.cycleRt', 'float'),
        LogVaraible('lighthouse.bs0Rt', 'float'),
        LogVaraible('lighthouse.bs1Rt', 'float'),
        LogVaraible('lighthouse.width0', 'uint16_t'),
        LogVaraible('lighthouse.width1', 'uint16_t'),
        LogVaraible('lighthouse.width2', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse6.csv', logvars))

    logconf = LogConfig(name='lighthouse7', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.width3', 'uint16_t'),
        LogVaraible('lighthouse.comSync', 'uint8_t'),
        LogVaraible('lighthouse.bsReceive', 'uint16_t'),
        LogVaraible('lighthouse.bsActive', 'uint16_t'),
        LogVaraible('lighthouse.bsCalUd', 'uint16_t'),
        LogVaraible('lighthouse.bsCalCon', 'uint16_t'),
        LogVaraible('lighthouse.status', 'uint8_t'),
        LogVaraible('lighthouse.posRt', 'float'),
        LogVaraible('lighthouse.estBs0Rt', 'float'),
        LogVaraible('lighthouse.estBs1Rt', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse7.csv', logvars))

    logconf = LogConfig(name='lighthouse8', period_in_ms=100)
    logvars = [
        LogVaraible('lighthouse.x', 'float'),
        LogVaraible('lighthouse.y', 'float'),
        LogVaraible('lighthouse.z', 'float'),
        LogVaraible('lighthouse.delta', 'float'),
        LogVaraible('lighthouse.bsGeoVal', 'uint16_t'),
        LogVaraible('lighthouse.bsCalVal', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'lighthouse8.csv', logvars))

    logconf = LogConfig(name='loco', period_in_ms=100)
    logvars = [
        LogVaraible('loco.mode', 'uint8_t'),
        LogVaraible('loco.spiWr', 'float'),
        LogVaraible('loco.spiRe', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'loco.csv', logvars))

    logconf = LogConfig(name='locSrv1', period_in_ms=100)
    logvars = [
        LogVaraible('locSrv.x', 'float'),
        LogVaraible('locSrv.y', 'float'),
        LogVaraible('locSrv.z', 'float'),
        LogVaraible('locSrv.qx', 'float'),
        LogVaraible('locSrv.qy', 'float'),
        LogVaraible('locSrv.qz', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'locSrv1.csv', logvars))

    logconf = LogConfig(name='locSrv2', period_in_ms=100)
    logvars = [
        LogVaraible('locSrv.qw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'locSrv2.csv', logvars))

    logconf = LogConfig(name='locSrvZ', period_in_ms=100)
    logvars = [
        LogVaraible('locSrvZ.tick', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'locSrvZ.csv', logvars))

    logconf = LogConfig(name='mag', period_in_ms=100)
    logvars = [
        LogVaraible('mag.x', 'float'),
        LogVaraible('mag.y', 'float'),
        LogVaraible('mag.z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'mag.csv', logvars))

    logconf = LogConfig(name='memTst', period_in_ms=100)
    logvars = [
        LogVaraible('memTst.errCntW', 'uint32_t'),
    ]
    loggers.append(FileLogger(logconf, 'memTst.csv', logvars))

    logconf = LogConfig(name='motion', period_in_ms=100)
    logvars = [
        LogVaraible('motion.motion', 'uint8_t'),
        LogVaraible('motion.deltaX', 'int16_t'),
        LogVaraible('motion.deltaY', 'int16_t'),
        LogVaraible('motion.shutter', 'uint16_t'),
        LogVaraible('motion.maxRaw', 'uint8_t'),
        LogVaraible('motion.minRaw', 'uint8_t'),
        LogVaraible('motion.Rawsum', 'uint8_t'),
        LogVaraible('motion.outlierCount', 'uint8_t'),
        LogVaraible('motion.squal', 'uint8_t'),
        LogVaraible('motion.std', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'motion.csv', logvars))

    logconf = LogConfig(name='Motor', period_in_ms=100)
    logvars = [
        LogVaraible('motor.m1', 'uint32_t'),
        LogVaraible('motor.m2', 'uint32_t'),
        LogVaraible('motor.m3', 'uint32_t'),
        LogVaraible('motor.m4', 'uint32_t'),
    ]
    loggers.append(FileLogger(logconf, 'motor.csv', logvars))

    logconf = LogConfig(name='oa', period_in_ms=100)
    logvars = [
        LogVaraible('oa.front', 'uint16_t'),
        LogVaraible('oa.back', 'uint16_t'),
        LogVaraible('oa.up', 'uint16_t'),
        LogVaraible('oa.left', 'uint16_t'),
        LogVaraible('oa.right', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'oa.csv', logvars))

    logconf = LogConfig(name='outlierf1', period_in_ms=100)
    logvars = [
        LogVaraible('outlierf.lhWin', 'int32_t'),
        LogVaraible('outlierf.bucket0', 'int32_t'),
        LogVaraible('outlierf.bucket1', 'int32_t'),
        LogVaraible('outlierf.bucket2', 'int32_t'),
        LogVaraible('outlierf.bucket3', 'int32_t'),
        LogVaraible('outlierf.bucket4', 'int32_t'),
    ]
    loggers.append(FileLogger(logconf, 'outlierf1.csv', logvars))

    logconf = LogConfig(name='outlierf2', period_in_ms=100)
    logvars = [
        LogVaraible('outlierf.accLev', 'float'),
        LogVaraible('outlierf.errD', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'outlierf2.csv', logvars))

    logconf = LogConfig(name='pid_attitude1', period_in_ms=100)
    logvars = [
        LogVaraible('pid_attitude.roll_outP', 'float'),
        LogVaraible('pid_attitude.roll_outI', 'float'),
        LogVaraible('pid_attitude.roll_outD', 'float'),
        LogVaraible('pid_attitude.pitch_outP', 'float'),
        LogVaraible('pid_attitude.pitch_outI', 'float'),
        LogVaraible('pid_attitude.pitch_outD', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'pid_attitude1.csv', logvars))


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
    