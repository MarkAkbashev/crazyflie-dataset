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


    logconf = LogConfig(name='pid_attitude2', period_in_ms=100)
    logvars = [
        LogVaraible('pid_attitude.yaw_outP', 'float'),
        LogVaraible('pid_attitude.yaw_outI', 'float'),
        LogVaraible('pid_attitude.yaw_outD', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'pid_attitude2.csv', logvars))

    logconf = LogConfig(name='pid_rate1', period_in_ms=100)
    logvars = [
        LogVaraible('pid_rate.roll_outP', 'float'),
        LogVaraible('pid_rate.roll_outI', 'float'),
        LogVaraible('pid_rate.roll_outD', 'float'),
        LogVaraible('pid_rate.pitch_outP', 'float'),
        LogVaraible('pid_rate.pitch_outI', 'float'),
        LogVaraible('pid_rate.pitch_outD', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'pid_rate1.csv', logvars))

    logconf = LogConfig(name='pid_rate2', period_in_ms=100)
    logvars = [
        LogVaraible('pid_rate.yaw_outP', 'float'),
        LogVaraible('pid_rate.yaw_outI', 'float'),
        LogVaraible('pid_rate.yaw_outD', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'pid_rate2.csv', logvars))

    logconf = LogConfig(name='pm', period_in_ms=100)
    logvars = [
        LogVaraible('pm.vbat', 'float'),
        LogVaraible('pm.vbatMV', 'uint16_t'),
        LogVaraible('pm.extVbat', 'float'),
        LogVaraible('pm.extVbatMV', 'uint16_t'),
        LogVaraible('pm.extCurr', 'float'),
        LogVaraible('pm.chargeCurrent', 'float'),
        LogVaraible('pm.state', 'int8_t'),
        LogVaraible('pm.batteryLevel', 'uint8_t'),
    ]
    loggers.append(FileLogger(logconf, 'pm.csv', logvars))

    logconf = LogConfig(name='posCtl1', period_in_ms=100)
    logvars = [
        LogVaraible('posCtl.targetVX', 'float'),
        LogVaraible('posCtl.targetVY', 'float'),
        LogVaraible('posCtl.targetVZ', 'float'),
        LogVaraible('posCtl.targetX', 'float'),
        LogVaraible('posCtl.targetY', 'float'),
        LogVaraible('posCtl.targetZ', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtl1.csv', logvars))

    logconf = LogConfig(name='posCtl2', period_in_ms=100)
    logvars = [
        LogVaraible('posCtl.bodyVX', 'float'),
        LogVaraible('posCtl.bodyVY', 'float'),
        LogVaraible('posCtl.bodyX', 'float'),
        LogVaraible('posCtl.bodyY', 'float'),
        LogVaraible('posCtl.Yp', 'float'),
        LogVaraible('posCtl.Yi', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtl2.csv', logvars))

    logconf = LogConfig(name='posCtl3', period_in_ms=100)
    logvars = [
        LogVaraible('posCtl.Yd', 'float'),
        LogVaraible('posCtl.Zp', 'float'),
        LogVaraible('posCtl.Zi', 'float'),
        LogVaraible('posCtl.Zd', 'float'),
        LogVaraible('posCtl.VXp', 'float'),
        LogVaraible('posCtl.VXi', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtl3.csv', logvars))

    logconf = LogConfig(name='posCtl4', period_in_ms=100)
    logvars = [
        LogVaraible('posCtl.VXd', 'float'),
        LogVaraible('posCtl.VZp', 'float'),
        LogVaraible('posCtl.VZi', 'float'),
        LogVaraible('posCtl.VZd', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtl4.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi1', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.posRef_x', 'float'),
        LogVaraible('posCtrlIndi.posRef_y', 'float'),
        LogVaraible('posCtrlIndi.posRef_z', 'float'),
        LogVaraible('posCtrlIndi.velS_x', 'float'),
        LogVaraible('posCtrlIndi.velS_y', 'float'),
        LogVaraible('posCtrlIndi.velS_z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi1.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi2', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.velRef_x', 'float'),
        LogVaraible('posCtrlIndi.velRef_y', 'float'),
        LogVaraible('posCtrlIndi.velRef_z', 'float'),
        LogVaraible('posCtrlIndi.angS_roll', 'float'),
        LogVaraible('posCtrlIndi.angS_pitch', 'float'),
        LogVaraible('posCtrlIndi.angS_yaw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi2.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi3', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.angF_roll', 'float'),
        LogVaraible('posCtrlIndi.angF_pitch', 'float'),
        LogVaraible('posCtrlIndi.angF_yaw', 'float'),
        LogVaraible('posCtrlIndi.accRef_x', 'float'),
        LogVaraible('posCtrlIndi.accRef_y', 'float'),
        LogVaraible('posCtrlIndi.accRef_z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi3.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi4', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.accS_x', 'float'),
        LogVaraible('posCtrlIndi.accS_y', 'float'),
        LogVaraible('posCtrlIndi.accS_z', 'float'),
        LogVaraible('posCtrlIndi.accF_x', 'float'),
        LogVaraible('posCtrlIndi.accF_y', 'float'),
        LogVaraible('posCtrlIndi.accF_z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi4.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi5', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.accFT_x', 'float'),
        LogVaraible('posCtrlIndi.accFT_y', 'float'),
        LogVaraible('posCtrlIndi.accFT_z', 'float'),
        LogVaraible('posCtrlIndi.accErr_x', 'float'),
        LogVaraible('posCtrlIndi.accErr_y', 'float'),
        LogVaraible('posCtrlIndi.accErr_z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi5.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi6', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.phi_tilde', 'float'),
        LogVaraible('posCtrlIndi.theta_tilde', 'float'),
        LogVaraible('posCtrlIndi.T_tilde', 'float'),
        LogVaraible('posCtrlIndi.T_inner', 'float'),
        LogVaraible('posCtrlIndi.T_inner_f', 'float'),
        LogVaraible('posCtrlIndi.T_incremented', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi6.csv', logvars))

    logconf = LogConfig(name='posCtrlIndi7', period_in_ms=100)
    logvars = [
        LogVaraible('posCtrlIndi.cmd_phi', 'float'),
        LogVaraible('posCtrlIndi.cmd_theta', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posCtrlIndi7.csv', logvars))

    logconf = LogConfig(name='posEstAlt', period_in_ms=100)
    logvars = [
        LogVaraible('posEstAlt.estimatedZ', 'float'),
        LogVaraible('posEstAlt.estVZ', 'float'),
        LogVaraible('posEstAlt.velocityZ', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'posEstAlt.csv', logvars))

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
    