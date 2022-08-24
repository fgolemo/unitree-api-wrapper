from dataclasses import dataclass

# typedef struct
# 	{
# 		uint8_t version_h;
# 		uint8_t version_l;
# 		uint8_t bms_status;
# 		uint8_t SOC;                       // SOC 0-100%
# 		int32_t current;                   // mA
# 		uint16_t cycle;
# 		std::array<int8_t, 2> BQ_NTC;                  // x1 degrees centigrade
# 		std::array<int8_t, 2> MCU_NTC;                 // x1 degrees centigrade
# 		std::array<uint16_t, 10>  cell_vol;            // cell voltage mV
# 	} BmsState;
#
from typing import Tuple, List


@dataclass()
class BmsState:
    version_h: int
    version_l: int
    bms_status: int
    SOC: int
    current: int
    cycle: int
    BQ_NTC: Tuple[int, int]
    MCU_NTC: Tuple[int, int]
    cell_vol: Tuple[int, int, int, int, int, int, int, int, int, int]


# 	typedef struct
# 	{
# 		float x;
# 		float y;
# 		float z;
# 	} Cartesian;


@dataclass()
class Cartesian:
    x: float
    y: float
    z: float


# 	typedef struct
# 	{
# 		std::array<float, 4> quaternion;               // quaternion, normalized, (w,x,y,z)
# 		std::array<float, 3> gyroscope;                // angular velocity （unit: rad/s)    (raw data)
# 		std::array<float, 3> accelerometer;            // m/(s2)                             (raw data)
# 		std::array<float, 3> rpy;                      // euler angle（unit: rad)
# 		int8_t temperature;
# 	} IMU;                                 // when under accelerated motion, the attitude of the robot calculated by IMU will drift.


@dataclass()
class IMU:
    quaternion: Tuple[float, float, float, float]
    gyroscope: Tuple[float, float, float]
    accelerometer: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    temperature: int


# 	typedef struct
# 	{
# 		uint8_t mode;                      // motor working mode
# 		float q;                           // current angle (unit: radian)
# 		float dq;                          // current velocity (unit: radian/second)
# 		float ddq;                         // current acc (unit: radian/second*second)
# 		float tauEst;                      // current estimated output torque (unit: N.m)
# 		float q_raw;                       // current angle (unit: radian)
# 		float dq_raw;                      // current velocity (unit: radian/second)
# 		float ddq_raw;
# 		int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
# 		std::array<uint32_t, 2> reserve;
# 	} MotorState;                          // motor feedback


@dataclass()
class MotorState:
    mode: int
    q: float
    dq: float
    ddq: float
    tauEst: float
    q_raw: float
    dq_raw: float
    ddq_raw: float
    temperature: int
    reserve: Tuple[int, int]


# 	typedef struct
# 	{
# 		std::array<uint8_t, 2> head;
# 		uint8_t levelFlag;
# 		uint8_t frameReserve;
#
# 		std::array<uint32_t, 2> SN;
# 		std::array<uint32_t, 2> version;
# 		uint16_t bandWidth;
# 		IMU imu;
# 		std::array<MotorState, 20> motorState;
# 		BmsState bms;
# 		std::array<int16_t, 4> footForce;           // force sensors
# 		std::array<int16_t, 4> footForceEst;        // force sensors
# 		uint32_t tick;                              // reference real-time from motion controller (unit: us)
# 		std::array<uint8_t, 40> wirelessRemote;     // wireless commands
# 		uint32_t reserve;
#
# 		uint32_t crc;
# 	} LowState;


@dataclass()
class LowState:
    head: Tuple[int, int]
    levelFlag: int
    frameReserve: int
    SN: Tuple[int, int]
    version: Tuple[int, int]
    bandWidth: int
    imu: IMU
    motorState: Tuple[
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
        MotorState,
    ]
    bms: BmsState
    footForce: Tuple[int, int, int, int]
    footForceEst: Tuple[int, int, int, int]
    tick: int
    wirelessRemote: List[int]
    reserve: int
    crc: int


# 	typedef struct
# 	{
# 		uint8_t off;                       // off 0xA5
# 		std::array<uint8_t, 3> reserve;
# 	} BmsCmd;


@dataclass()
class BmsCmd:
    off: int
    reserve: Tuple[int, int, int]


# 	typedef struct
# 	{
# 		uint8_t mode;                      // desired working mode
# 		float q;                           // desired angle (unit: radian)
# 		float dq;                          // desired velocity (unit: radian/second)
# 		float tau;                         // desired output torque (unit: N.m)
# 		float Kp;                          // desired position stiffness (unit: N.m/rad )
# 		float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )
# 		std::array<uint32_t, 3> reserve;
# 	} MotorCmd;


@dataclass()
class MotorCmd:
    mode: int  # desired working mode
    q: float  # desired angle (unit: radian)
    dq: float  # desired velocity (unit: radian/second)
    tau: float  # desired output torque (unit: N.m)
    Kp: float  # desired position stiffness (unit: N.m/rad )
    Kd: float  # desired velocity stiffness (unit: N.m/(rad/s) )
    reserve: Tuple[int, int]


# 	typedef struct
# 	{
# 		std::array<uint8_t, 2> head;
# 		uint8_t levelFlag;
# 		uint8_t frameReserve;
#
# 		std::array<uint32_t, 2> SN;
# 		std::array<uint32_t, 2> version;
# 		uint16_t bandWidth;
# 		std::array<MotorCmd, 20> motorCmd;
# 		BmsCmd bms;
# 		std::array<uint8_t, 40> wirelessRemote;
# 		uint32_t reserve;
#
# 		uint32_t crc;
# 	} LowCmd;


@dataclass()
class LowCmd:
    head: Tuple[int, int]
    levelFlag: int
    frameReserve: int
    SN: Tuple[int, int]
    version: Tuple[int, int]
    bandWidth: int
    motorCmd: List[MotorCmd]
    bms: BmsCmd
    wirelessRemote: List[int]
    reserve: int
    crc: int
