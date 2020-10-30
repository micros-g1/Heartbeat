# Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
# Copyright 2016 NXP
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

#
# Generated by erpcgen 1.7.4 on Thu Apr 16 10:59:26 2020.
#
# AUTOGENERATED - DO NOT EDIT
#

import erpc

# Structures data types declarations
#Which peripherals are available on used board
class BoardConfig(object):
    def __init__(self, DAC=None, MAG=None, RGB=None):
        self.DAC = DAC # bool
        self.MAG = MAG # bool
        self.RGB = RGB # bool[3]


    def _read(self, codec):
        self.DAC = codec.read_bool()
        self.MAG = codec.read_bool()
        self.RGB = []
        for _i0 in range(3):
            _v0 = codec.read_bool()
            self.RGB.append(_v0)

        return self

    def _write(self, codec):
        if self.DAC is None:
            raise ValueError("DAC is None")
        codec.write_bool(self.DAC)
        if self.MAG is None:
            raise ValueError("MAG is None")
        codec.write_bool(self.MAG)
        if self.RGB is None:
            raise ValueError("RGB is None")
        for _i0 in self.RGB:
            codec.write_bool(_i0)


    def __str__(self):
        return "<%s@%x DAC=%s MAG=%s RGB=%s>" % (self.__class__.__name__, id(self), self.DAC, self.MAG, self.RGB)

    def __repr__(self):
        return self.__str__()
        
#ADC peripheral configuration
class AdcConfig(object):
    def __init__(self, vref=None, atomicSteps=None):
        self.vref = vref # float
        self.atomicSteps = atomicSteps # float

    def _read(self, codec):
        self.vref = codec.read_float()
        self.atomicSteps = codec.read_float()
        return self

    def _write(self, codec):
        if self.vref is None:
            raise ValueError("vref is None")
        codec.write_float(self.vref)
        if self.atomicSteps is None:
            raise ValueError("atomicSteps is None")
        codec.write_float(self.atomicSteps)

    def __str__(self):
        return "<%s@%x vref=%s atomicSteps=%s>" % (self.__class__.__name__, id(self), self.vref, self.atomicSteps)

    def __repr__(self):
        return self.__str__()
        
#Struct for accelerometer and magnetometer peripherals
class Vector(object):
    def __init__(self, A_x=None, A_y=None, A_z=None, M_x=None, M_y=None, M_z=None):
        self.A_x = A_x # int16
        self.A_y = A_y # int16
        self.A_z = A_z # int16
        self.M_x = M_x # int16
        self.M_y = M_y # int16
        self.M_z = M_z # int16

    def _read(self, codec):
        self.A_x = codec.read_int16()
        self.A_y = codec.read_int16()
        self.A_z = codec.read_int16()
        self.M_x = codec.read_int16()
        self.M_y = codec.read_int16()
        self.M_z = codec.read_int16()
        return self

    def _write(self, codec):
        if self.A_x is None:
            raise ValueError("A_x is None")
        codec.write_int16(self.A_x)
        if self.A_y is None:
            raise ValueError("A_y is None")
        codec.write_int16(self.A_y)
        if self.A_z is None:
            raise ValueError("A_z is None")
        codec.write_int16(self.A_z)
        if self.M_x is None:
            raise ValueError("M_x is None")
        codec.write_int16(self.M_x)
        if self.M_y is None:
            raise ValueError("M_y is None")
        codec.write_int16(self.M_y)
        if self.M_z is None:
            raise ValueError("M_z is None")
        codec.write_int16(self.M_z)

    def __str__(self):
        return "<%s@%x A_x=%s A_y=%s A_z=%s M_x=%s M_y=%s M_z=%s>" % (self.__class__.__name__, id(self), self.A_x, self.A_y, self.A_z, self.M_x, self.M_y, self.M_z)

    def __repr__(self):
        return self.__str__()
        
