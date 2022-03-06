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
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

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

    logconf = LogConfig(name='Acc', period_in_ms=100)
    logvars = [
        LogVaraible('acc.x', 'float'),
        LogVaraible('acc.y', 'float'),
        LogVaraible('acc.z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'acc.csv', logvars))

    logconf = LogConfig(name='ActiveMarker', period_in_ms=100)
    logvars = [
        LogVaraible('activeMarker.btSns', 'uint8_t'),
        LogVaraible('activeMarker.i2cOk', 'uint8_t'),
    ]
    loggers.append(FileLogger(logconf, 'activeMarker.csv', logvars))

    # logconf = LogConfig(name='aideck', period_in_ms=100)
    # logvars = [
    #     LogVaraible('aideck.receivebyte', 'uint8_t'),
    # ]
    # loggers.append(FileLogger(logconf, 'aideck.csv', logvars))

    # logconf = LogConfig(name='aidecktest', period_in_ms=100)
    # logvars = [
    #     LogVaraible('aidecktest.testresult', 'uint32_t'),
    #     LogVaraible('aidecktest.done', 'uint8_t'),
    # ]
    # loggers.append(FileLogger(logconf, 'aidecktest.csv', logvars))

    # logconf = LogConfig(name='amarkUartTest', period_in_ms=100)
    # logvars = [
    #     LogVaraible('amarkUartTest.passed', 'uint8_t'),
    # ]
    # loggers.append(FileLogger(logconf, 'amarkUartTest.csv', logvars))

    logconf = LogConfig(name='Baro', period_in_ms=100)
    logvars = [
        LogVaraible('baro.pressure', 'float'),
        LogVaraible('baro.asl', 'float'),
        LogVaraible('baro.temp', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'baro.csv', logvars))

    logconf = LogConfig(name='Controller1', period_in_ms=100)
    logvars = [
        LogVaraible('controller.accelz', 'float'),
        LogVaraible('controller.actuatorThrust', 'float'),
        LogVaraible('controller.cmd_pitch', 'float'),
        LogVaraible('controller.cmd_roll', 'float'),
        LogVaraible('controller.cmd_thrust', 'float'),
        LogVaraible('controller.cmd_yaw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'controller1.csv', logvars))

    logconf = LogConfig(name='Controller2', period_in_ms=100)
    logvars = [
        LogVaraible('controller.ctr_yaw', 'int16_t'),
        LogVaraible('controller.pitch', 'float'),
        LogVaraible('controller.pitchRate', 'float'),
        LogVaraible('controller.r_pitch', 'float'),
        LogVaraible('controller.r_roll', 'float'),
        LogVaraible('controller.r_yaw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'controller2.csv', logvars))

    logconf = LogConfig(name='Controller3', period_in_ms=100)
    logvars = [
        LogVaraible('controller.roll', 'int16_t'),
        LogVaraible('controller.rollRate', 'float'),
        LogVaraible('controller.yaw', 'float'),
        LogVaraible('controller.yawRate', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'controller3.csv', logvars))

    logconf = LogConfig(name='crtp', period_in_ms=100)
    logvars = [
        LogVaraible('crtp.rxRate', 'uint16_t'),
        LogVaraible('crtp.txRate', 'uint16_t'),
    ]
    loggers.append(FileLogger(logconf, 'crtp.csv', logvars))

    # logconf = LogConfig(name='ctrlINDI1', period_in_ms=100)
    # logvars = [
    #     LogVaraible('ctrlINDI.cmd_thrust', 'float'),
    #     LogVaraible('ctrlINDI.cmd_roll', 'float'),
    #     LogVaraible('ctrlINDI.cmd_pitch', 'float'),
    #     LogVaraible('ctrlINDI.cmd_yaw', 'float'),
    #     LogVaraible('ctrlINDI.r_roll', 'float'),
    #     LogVaraible('ctrlINDI.r_pitch', 'float'),
    # ]
    # loggers.append(FileLogger(logconf, 'ctrlINDI1.csv', logvars))

    # logconf = LogConfig(name='ctrlINDI2', period_in_ms=100)
    # logvars = [
    #     LogVaraible('ctrlINDI.r_yaw', 'float'),
    #     LogVaraible('ctrlINDI.u_act_dyn_p', 'float'),
    #     LogVaraible('ctrlINDI.u_act_dyn_q', 'float'),
    #     LogVaraible('ctrlINDI.u_act_dyn_r', 'float'),
    #     LogVaraible('ctrlINDI.du_p', 'float'),
    #     LogVaraible('ctrlINDI.du_q', 'float'),
    # ]
    # loggers.append(FileLogger(logconf, 'ctrlINDI2.csv', logvars))

    # logconf = LogConfig(name='ctrlINDI3', period_in_ms=100)
    # logvars = [
    #     LogVaraible('ctrlINDI.du_r', 'float'),
    #     LogVaraible('ctrlINDI.ang_accel_ref_p', 'float'),
    #     LogVaraible('ctrlINDI.ang_accel_ref_q', 'float'),
    #     LogVaraible('ctrlINDI.ang_accel_ref_r', 'float'),
    #     LogVaraible('ctrlINDI.rate_d', 'float'),
    #     LogVaraible('ctrlINDI.uf_p', 'float'),
    # ]
    # loggers.append(FileLogger(logconf, 'ctrlINDI3.csv', logvars))

    # logconf = LogConfig(name='ctrlINDI4', period_in_ms=100)
    # logvars = [
    #     LogVaraible('ctrlINDI.uf_q', 'float'),
    #     LogVaraible('ctrlINDI.uf_r', 'float'),
    #     LogVaraible('ctrlINDI.Omega_f_p', 'float'),
    #     LogVaraible('ctrlINDI.Omega_f_q', 'float'),
    #     LogVaraible('ctrlINDI.Omega_f_r', 'float'),
    #     LogVaraible('ctrlINDI.n_p', 'float'),
    # ]
    # loggers.append(FileLogger(logconf, 'ctrlINDI4.csv', logvars))

    # logconf = LogConfig(name='ctrlINDI5', period_in_ms=100)
    # logvars = [
    #     LogVaraible('ctrlINDI.n_q', 'float'),
    #     LogVaraible('ctrlINDI.n_r', 'float'),
    # ]
    # loggers.append(FileLogger(logconf, 'ctrlINDI5.csv', logvars))

    logconf = LogConfig(name='ctrlMel1', period_in_ms=100)
    logvars = [
        LogVaraible('ctrlMel.cmd_thrust', 'float'),
        LogVaraible('ctrlMel.cmd_roll', 'float'),
        LogVaraible('ctrlMel.cmd_pitch', 'float'),
        LogVaraible('ctrlMel.cmd_yaw', 'float'),
        LogVaraible('ctrlMel.r_roll', 'float'),
        LogVaraible('ctrlMel.r_pitch', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ctrlMel1.csv', logvars))

    logconf = LogConfig(name='ctrlMel2', period_in_ms=100)
    logvars = [
        LogVaraible('ctrlMel.r_yaw', 'float'),
        LogVaraible('ctrlMel.accelz', 'float'),
        LogVaraible('ctrlMel.zdx', 'float'),
        LogVaraible('ctrlMel.zdy', 'float'),
        LogVaraible('ctrlMel.zdz', 'float'),
        LogVaraible('ctrlMel.i_err_x', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ctrlMel2.csv', logvars))

    logconf = LogConfig(name='ctrlMel3', period_in_ms=100)
    logvars = [
        LogVaraible('ctrlMel.i_err_y', 'float'),
        LogVaraible('ctrlMel.i_err_z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ctrlMel3.csv', logvars))

    logconf = LogConfig(name='ctrltarget1', period_in_ms=100)
    logvars = [
        LogVaraible('ctrltarget.x', 'float'),
        LogVaraible('ctrltarget.y', 'float'),
        LogVaraible('ctrltarget.z', 'float'),
        LogVaraible('ctrltarget.vx', 'float'),
        LogVaraible('ctrltarget.vx', 'float'),
        LogVaraible('ctrltarget.vx', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ctrltarget1.csv', logvars))

    logconf = LogConfig(name='ctrltarget2', period_in_ms=100)
    logvars = [
        LogVaraible('ctrltarget.ax', 'float'),
        LogVaraible('ctrltarget.ay', 'float'),
        LogVaraible('ctrltarget.az', 'float'),
        LogVaraible('ctrltarget.roll', 'float'),
        LogVaraible('ctrltarget.pitch', 'float'),
        LogVaraible('ctrltarget.yaw', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ctrltarget2.csv', logvars))

    logconf = LogConfig(name='ctrltargetZ1', period_in_ms=100)
    logvars = [
        LogVaraible('ctrltargetZ.x', 'int16_t'),
        LogVaraible('ctrltargetZ.y', 'int16_t'),
        LogVaraible('ctrltargetZ.z', 'int16_t'),
        LogVaraible('ctrltargetZ.vx', 'int16_t'),
        LogVaraible('ctrltargetZ.vy', 'int16_t'),
        LogVaraible('ctrltargetZ.vz', 'int16_t'),
    ]
    loggers.append(FileLogger(logconf, 'ctrltargetZ1.csv', logvars))

    logconf = LogConfig(name='ctrltargetZ2', period_in_ms=100)
    logvars = [
        LogVaraible('ctrltargetZ.ax', 'int16_t'),
        LogVaraible('ctrltargetZ.ay', 'int16_t'),
        LogVaraible('ctrltargetZ.az', 'int16_t'),
    ]
    loggers.append(FileLogger(logconf, 'ctrltargetZ2.csv', logvars))

    logconf = LogConfig(name='estimator', period_in_ms=100)
    logvars = [
        LogVaraible('estimator.rtApnd', 'float'),
        LogVaraible('estimator.rtRej', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'estimator.csv', logvars))

    logconf = LogConfig(name='ext_pos', period_in_ms=100)
    logvars = [
        LogVaraible('ext_pos.X', 'float'),
        LogVaraible('ext_pos.Y', 'float'),
        LogVaraible('ext_pos.Z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'ext_pos.csv', logvars))

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

    logconf = LogConfig(name='gps', period_in_ms=100)
    logvars = [
        LogVaraible('gps.lat', 'int32_t'),
        LogVaraible('gps.lon', 'int32_t'),
        LogVaraible('gps.hMSL', 'float'),
        LogVaraible('gps.hAcc', 'float'),
        LogVaraible('gps.nsat', 'int32_t'),
        LogVaraible('gps.fix', 'int32_t'),
    ]
    loggers.append(FileLogger(logconf, 'gps.csv', logvars))


    logconf = LogConfig(name='gyro', period_in_ms=100)
    logvars = [
        LogVaraible('gyro.xRaw', 'int16_t'),
        LogVaraible('gyro.yRaw', 'int16_t'),
        LogVaraible('gyro.zRaw', 'int16_t'),
        LogVaraible('gyro.xVariance', 'float'),
        LogVaraible('gyro.yVariance', 'float'),
        LogVaraible('gyro.zVariance', 'float'),
        LogVaraible('gyro.x', 'float'),
        LogVaraible('gyro.y', 'float'),
        LogVaraible('gyro.z', 'float'),
    ]
    loggers.append(FileLogger(logconf, 'gyro.csv', logvars))

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

    logconf = LogConfig(name='lhFlasher', period_in_ms=100)
    logvars = [
        LogVaraible('lhFlasher.done', 'uint8_t'),
        LogVaraible('lhFlasher.code', 'uint32_t'),
    ]
    loggers.append(FileLogger(logconf, 'lhFlasher.csv', logvars))

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
    