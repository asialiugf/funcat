# -*- coding: utf-8 -*-

from .api import (
    OPEN, HIGH, LOW, CLOSE, VOLUME, VOL,
    ABS, MAX, HHV, LLV,
    REF, IF, SUM, STD,
    MA, EMA, SMA,
)


def KDJ(N=9, M1=3, M2=3):
    """
    KDJ 随机指标
    """
    RSV = (CLOSE - LLV(LOW, N)) / (HHV(HIGH, N) - LLV(LOW, N)) * 100
    K = EMA(RSV, (M1 * 2 - 1))
    D = EMA(K, (M2 * 2 - 1))
    J = K * 3 - D * 2

    return K, D, J


"""
#added by charmi 20171112 begin
"""
def KD(N=9, M1=3, M2=3):
    """
    KD 随机指标
    """
    RSV = (CLOSE - LLV(LOW, N)) / (HHV(HIGH, N) - LLV(LOW, N)) * 100
    K = EMA(RSV, (M1 * 2 - 1))
    D = EMA(K, (M2 * 2 - 1))

    return K, D

def KE(N=9, M1=3, M2=5):
    RSV = (CLOSE - LLV(LOW, N)) / (HHV(HIGH, N) - LLV(LOW, N)) * 100
    K = EMA(RSV, (M1 * 2 - 1))
    E = EMA(K, M2)
    return K, E

def KE_ML(N=9, M1=3, M2=5, L=4):
    """
    KE 随机平均指标
    """
    L1 = L
    L2 = L*L
    L3 = L*L*L
    L4 = L*L*L*L

    RSV = (CLOSE - LLV(LOW, N)) / (HHV(HIGH, N) - LLV(LOW, N)) * 100
    K = EMA(RSV, (M1 * 2 - 1))
    E = EMA(K, 2)

    RSV1 = (CLOSE - LLV(LOW, N*L1)) / (HHV(HIGH, N*L1) - LLV(LOW, N*L1)) * 100
    K1 = EMA(RSV1, (M1*L1 * 2 - 1))
    E1 = EMA(K1, M2)

    RSV2 = (CLOSE - LLV(LOW, N*L2)) / (HHV(HIGH, N*L2) - LLV(LOW, N*L2)) * 100
    K2 = EMA(RSV2, (M1*L2 * 2 - 1))
    E2 = EMA(K2, M2*L1)

    RSV3 = (CLOSE - LLV(LOW, N*L3)) / (HHV(HIGH, N*L3) - LLV(LOW, N*L3)) * 100
    K3 = EMA(RSV3, (M1*L3 * 2 - 1))
    E3 = EMA(K3, M2*L2)

    """
    RSV4 = (CLOSE - LLV(LOW, N*L4)) / (HHV(HIGH, N*L4) - LLV(LOW, N*L4)) * 100
    K4 = EMA(RSV4, (M1*L4 * 2 - 1))
    E4 = EMA(K4, M2*L3)

    return K, E, K1, E1,  K2, E2, K3, E3, K4, E4
    """

    return K, E, K1, E1,  K2, E2, K3, E3


"""
#added by charmi 20171112 end
"""

def DMI(M1=14, M2=6):
    """
    DMI 趋向指标
    """
    TR = SUM(MAX(MAX(HIGH - LOW, ABS(HIGH - REF(CLOSE, 1))), ABS(LOW - REF(CLOSE, 1))), M1)
    HD = HIGH - REF(HIGH, 1)
    LD = REF(LOW, 1) - LOW

    DMP = SUM(IF((HD > 0) & (HD > LD), HD, 0), M1)
    DMM = SUM(IF((LD > 0) & (LD > HD), LD, 0), M1)
    DI1 = DMP * 100 / TR
    DI2 = DMM * 100 / TR
    ADX = MA(ABS(DI2 - DI1) / (DI1 + DI2) * 100, M2)
    ADXR = (ADX + REF(ADX, M2)) / 2

    return DI1, DI2, ADX, ADXR


def MACD(SHORT=12, LONG=26, M=9):
    """
    MACD 指数平滑移动平均线
    """
    DIFF = EMA(CLOSE, SHORT) - EMA(CLOSE, LONG)
    DEA = EMA(DIFF, M)
    MACD = (DIFF - DEA) * 2

    """
    Modified by charmi 20171112
    """
    return DIFF, DEA, MACD

"""
#added by charmi 20171112 begin
"""
def MACD_ML(SHORT=12, LONG=26, M=9, L=4):
    """
    MACD 指数平滑移动平均线
    """

    L1 = L
    L2 = L*L
    L3 = L*L*L
    L4 = L*L*L*L

    DIFF1 = EMA(CLOSE, SHORT) - EMA(CLOSE, LONG)
    DEA1 = EMA(DIFF1, M)
    MACD1 = (DIFF1 - DEA1) * 2

    DIFF2 = EMA(CLOSE, SHORT*L1) - EMA(CLOSE, LONG*L1)
    DEA2 = EMA(DIFF2, M*L1)
    MACD2 = (DIFF2 - DEA2) * 2

    DIFF3 = EMA(CLOSE, SHORT*L2) - EMA(CLOSE, LONG*L2)
    DEA3 = EMA(DIFF3, M*L2)
    MACD3 = (DIFF3 - DEA3) * 2

    """
    DIFF4 = EMA(CLOSE, SHORT*L3) - EMA(CLOSE, LONG*L3)
    DEA4 = EMA(DIFF4, M*L3)
    MACD4 = (DIFF4 - DEA4) * 2

    return DIFF1, DEA1, MACD1, DIFF2, DEA2, MACD2, DIFF3, DEA3, MACD3, DIFF4, DEA4, MACD4
    """
    return DIFF1, DEA1, MACD1, DIFF2, DEA2, MACD2, DIFF3, DEA3, MACD3
"""
#added by charmi 20171112 end
"""

def RSI(N1=6, N2=12, N3=24):
    """
    RSI 相对强弱指标
    """
    LC = REF(CLOSE, 1)
    RSI1 = SMA(MAX(CLOSE - LC, 0), N1, 1) / SMA(ABS(CLOSE - LC), N1, 1) * 100
    RSI2 = SMA(MAX(CLOSE - LC, 0), N2, 1) / SMA(ABS(CLOSE - LC), N2, 1) * 100
    RSI3 = SMA(MAX(CLOSE - LC, 0), N3, 1) / SMA(ABS(CLOSE - LC), N3, 1) * 100

    return RSI1, RSI2, RSI3


def BOLL(N=20, P=2):
    """
    BOLL 布林带
    """
    MID = MA(CLOSE, N)
    UPPER = MID + STD(CLOSE, N) * P
    LOWER = MID - STD(CLOSE, N) * P

    return UPPER, MID, LOWER


def WR(N=10, N1=6):
    """
    W&R 威廉指标
    """
    WR1 = (HHV(HIGH, N) - CLOSE) / (HHV(HIGH, N) - LLV(LOW, N)) * 100
    WR2 = (HHV(HIGH, N1) - CLOSE) / (HHV(HIGH, N1) - LLV(LOW, N1)) * 100

    return WR1, WR2


def BIAS(L1=5, L4=3, L5=10):
    """
    BIAS 乖离率
    """
    BIAS = (CLOSE - MA(CLOSE, L1)) / MA(CLOSE, L1) * 100
    BIAS2 = (CLOSE - MA(CLOSE, L4)) / MA(CLOSE, L4) * 100
    BIAS3 = (CLOSE - MA(CLOSE, L5)) / MA(CLOSE, L5) * 100

    return BIAS, BIAS2, BIAS3


def ASI(M1=26, M2=10):
    """
    ASI 震动升降指标
    """
    LC = REF(CLOSE, 1)
    AA = ABS(HIGH - LC)
    BB = ABS(LOW - LC)
    CC = ABS(HIGH - REF(LOW, 1))
    DD = ABS(LC - REF(OPEN, 1))
    R = IF((AA > BB) & (AA > CC), AA + BB / 2 + DD / 4, IF((BB > CC) & (BB > AA), BB + AA / 2 + DD / 4, CC + DD / 4))
    X = (CLOSE - LC + (CLOSE - OPEN) / 2 + LC - REF(OPEN, 1))
    SI = X * 16 / R * MAX(AA, BB)
    ASI = SUM(SI, M1)
    ASIT = MA(ASI, M2)

    return ASI, ASIT


def VR(M1=26):
    """
    VR容量比率
    """
    LC = REF(CLOSE, 1)
    VR = SUM(IF(CLOSE > LC, VOL, 0), M1) / SUM(IF(CLOSE <= LC, VOL, 0), M1) * 100

    return VR


def ARBR(M1=26):
    """
    ARBR人气意愿指标
    """
    AR = SUM(HIGH - OPEN, M1) / SUM(OPEN - LOW, M1) * 100
    BR = SUM(MAX(0, HIGH - REF(CLOSE, 1)), M1) / SUM(MAX(0, REF(CLOSE, 1) - LOW), M1) * 100

    return AR, BR


def DPO(M1=20, M2=10, M3=6):
    DPO = CLOSE - REF(MA(CLOSE, M1), M2)
    MADPO = MA(DPO, M3)

    return DPO, MADPO


def TRIX(M1=12, M2=20):
    TR = EMA(EMA(EMA(CLOSE, M1), M1), M1)
    TRIX = (TR - REF(TR, 1)) / REF(TR, 1) * 100
    TRMA = MA(TRIX, M2)

    return TRIX, TRMA
