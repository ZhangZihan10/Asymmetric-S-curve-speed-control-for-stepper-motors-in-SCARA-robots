#include <AccelStepper.h>
#include <Arduino.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MOVE_SPEED 500

// ===============================
// 引脚定义
// ===============================
const int enablePin = 8;

const int xdirPin  = 5;
const int xstepPin = 2;

const int ydirPin  = 6;
const int ystepPin = 3;

const int zdirPin  = 7;
const int zstepPin = 4;

const int xEndstopPin = 11;
const int yEndstopPin = 10;
const int zEndstopPin = 9;

// ===============================
// 执行对象
// ===============================
AccelStepper stepperX(AccelStepper::DRIVER, xstepPin, xdirPin);
AccelStepper stepperY(AccelStepper::DRIVER, ystepPin, ydirPin);
AccelStepper stepperZ(AccelStepper::DRIVER, zstepPin, zdirPin);

// ===============================
// 参数配置
// ===============================
const int MAX_SEGMENTS = 32;
const int RX_BUF_SIZE  = 520;

// 速度下限，避免极小速度时长时间不出步
const float MIN_STEP_SPEED = 1.0f;

// 段内速度更新时间片（微秒）
const unsigned long SPEED_UPDATE_US = 3000;

// 段执行超时冗余（秒）
const float SEG_TIMEOUT_MARGIN = 2.50f;

// 起点移动参数
const float START_Z_MAX_SPEED = 5000.0f;
const float START_Z_ACCEL     = 1500.0f;
const float START_Y_MAX_SPEED = 1000.0f;
const float START_Y_ACCEL     = 800.0f;
const float START_X_MAX_SPEED = 1000.0f;
const float START_X_ACCEL     = 800.0f;

// ===============================
// 状态机
// ===============================
enum ReceiveMode {
  MODE_NONE,
  MODE_START_POINT,
  MODE_SEGMENTS_AXIS
};

enum SystemState {
  STATE_IDLE,
  STATE_RECEIVING,
  STATE_READY_START,
  STATE_READY_SEGMENTS,
  STATE_RUNNING
};

SystemState currentState = STATE_IDLE;
ReceiveMode receiveMode  = MODE_NONE;

// ===============================
// 数据结构
// ===============================
struct StartPointData {
  long z;
  long y;
  long x;
  bool valid;
};

struct AxisProfile {
  float vs;
  float ve;
  float vp;
  float aa;
  float ad;
  float Tja;
  float Tjd;
  float Ta;
  float Tc;
  float Td;
  float Ttot;
};

struct SegmentAxisSCurve {
  long z0, y0, x0;
  long z1, y1, x1;

  AxisProfile zAxis;
  AxisProfile yAxis;
  AxisProfile xAxis;
};

StartPointData startPoint;
SegmentAxisSCurve segments[MAX_SEGMENTS];
int totalSegments = 0;
int receivedCount = 0;

// ===============================
// 串口接收缓存
// ===============================
char rxBuf[RX_BUF_SIZE];

// ===============================
// 函数声明
// ===============================
void resetTaskData();
bool readLineNonBlocking(char* buffer, size_t len);
void processLine(const char* line);

bool parseBeginStart(const char* line, int& count);
bool parseBeginSegAx(const char* line, int& count);
bool parseStartPoint(const char* line, StartPointData& p);
bool parseAxisSegmentLine(const char* line, SegmentAxisSCurve& seg);

void runStartMove();
void executeAxisSCurvePath();
bool executeAxisSCurveSegment(const SegmentAxisSCurve& seg, int segIndex);

float evalAxisVelocity(float t, const AxisProfile& ap);
void applyAxisSpeed(long p0, long p1, float vAbs, AccelStepper& stepper);

bool allAxesArrived();
void setAxesCurrentPosition(long z, long y, long x);

void manipZeroSetup(AccelStepper& stepperX, AccelStepper& stepperY, AccelStepper& stepperZ,
                    int xEndstopPin, int yEndstopPin, int zEndstopPin, int movementSpeed);
void setMotorToZeroPoint(AccelStepper& stepper, int endstopPin, int movementSpeed);

// ===============================
// setup
// ===============================
void setup() {
  pinMode(xstepPin, OUTPUT);
  pinMode(xdirPin, OUTPUT);
  pinMode(ystepPin, OUTPUT);
  pinMode(ydirPin, OUTPUT);
  pinMode(zstepPin, OUTPUT);
  pinMode(zdirPin, OUTPUT);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  pinMode(xEndstopPin, INPUT_PULLUP);
  pinMode(yEndstopPin, INPUT_PULLUP);
  pinMode(zEndstopPin, INPUT_PULLUP);

  stepperX.setMaxSpeed(6000.0);
  stepperY.setMaxSpeed(3000.0);
  stepperZ.setMaxSpeed(6000.0);

  stepperX.setAcceleration(1000.0);
  stepperY.setAcceleration(1000.0);
  stepperZ.setAcceleration(1000.0);

  Serial.begin(115200);

  startPoint.valid = false;
  resetTaskData();

  manipZeroSetup(stepperX, stepperY, stepperZ,
                 xEndstopPin, yEndstopPin, zEndstopPin, MOVE_SPEED);

  Serial.println("ARDUINO_AXIS_S_CURVE_READY");
}

// ===============================
// loop
// ===============================
void loop() {
  if (currentState != STATE_RUNNING) {
    if (readLineNonBlocking(rxBuf, sizeof(rxBuf))) {
      processLine(rxBuf);
    }
  }

  if (currentState == STATE_READY_START) {
    currentState = STATE_RUNNING;
    Serial.println("ACK_START");

    runStartMove();

    currentState = STATE_IDLE;
    receiveMode = MODE_NONE;
    Serial.println("FINISHED_ALL");
  }

  if (currentState == STATE_READY_SEGMENTS) {
    currentState = STATE_RUNNING;
    Serial.println("ACK_START");

    executeAxisSCurvePath();

    currentState = STATE_IDLE;
    receiveMode = MODE_NONE;
    Serial.println("FINISHED_ALL");
  }
}

// ===============================
// 重置任务数据
// ===============================
void resetTaskData() {
  totalSegments = 0;
  receivedCount = 0;
  startPoint.valid = false;

  for (int i = 0; i < MAX_SEGMENTS; i++) {
    segments[i].z0 = segments[i].y0 = segments[i].x0 = 0;
    segments[i].z1 = segments[i].y1 = segments[i].x1 = 0;

    memset(&segments[i].zAxis, 0, sizeof(AxisProfile));
    memset(&segments[i].yAxis, 0, sizeof(AxisProfile));
    memset(&segments[i].xAxis, 0, sizeof(AxisProfile));
  }
}

// ===============================
// 非阻塞读一行
// ===============================
bool readLineNonBlocking(char* buffer, size_t len) {
  static size_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buffer[idx] = '\0';
      idx = 0;
      return true;
    }

    if (idx < len - 1) {
      buffer[idx++] = c;
    } else {
      idx = 0;
      buffer[0] = '\0';
      Serial.println("ERR_RX_OVERFLOW");
      return false;
    }
  }
  return false;
}

// ===============================
// 处理协议行
// ===============================
void processLine(const char* line) {
  if (line == nullptr || strlen(line) == 0) return;

  int count = 0;

  if (parseBeginStart(line, count)) {
    resetTaskData();

    if (count != 1) {
      Serial.println("ERR_BEGIN_START_COUNT");
      currentState = STATE_IDLE;
      receiveMode = MODE_NONE;
      return;
    }

    receiveMode = MODE_START_POINT;
    currentState = STATE_RECEIVING;
    receivedCount = 0;

    Serial.println("ACK_READY");
    return;
  }

  if (parseBeginSegAx(line, count)) {
    resetTaskData();

    if (count <= 0 || count > MAX_SEGMENTS) {
      Serial.println("ERR_SEG_COUNT");
      currentState = STATE_IDLE;
      receiveMode = MODE_NONE;
      return;
    }

    totalSegments = count;
    receiveMode = MODE_SEGMENTS_AXIS;
    currentState = STATE_RECEIVING;
    receivedCount = 0;

    Serial.println("ACK_READY");
    return;
  }

  if (strncmp(line, "P,", 2) == 0) {
    if (currentState != STATE_RECEIVING || receiveMode != MODE_START_POINT) {
      Serial.println("ERR_P_STATE");
      return;
    }

    StartPointData p;
    if (!parseStartPoint(line, p)) {
      Serial.println("ERR_P_PARSE");
      return;
    }

    startPoint = p;
    startPoint.valid = true;
    receivedCount = 1;
    Serial.println("ACK_POINT");
    return;
  }

  if (strncmp(line, "A,", 2) == 0) {
    if (currentState != STATE_RECEIVING || receiveMode != MODE_SEGMENTS_AXIS) {
      Serial.println("ERR_A_STATE");
      return;
    }

    if (receivedCount >= totalSegments) {
      Serial.println("ERR_SEG_OVERFLOW");
      return;
    }

    SegmentAxisSCurve seg;
    if (!parseAxisSegmentLine(line, seg)) {
      Serial.println("ERR_A_PARSE");
      return;
    }

    segments[receivedCount] = seg;
    receivedCount++;
    Serial.println("ACK_SEG");
    return;
  }

  if (strcmp(line, "RUN_START") == 0) {
    if (currentState != STATE_RECEIVING || receiveMode != MODE_START_POINT) {
      Serial.println("ERR_RUN_START_STATE");
      return;
    }

    if (!startPoint.valid || receivedCount != 1) {
      Serial.println("ERR_RUN_START_DATA");
      return;
    }

    currentState = STATE_READY_START;
    return;
  }

  if (strcmp(line, "RUN_SEGAX") == 0) {
    if (currentState != STATE_RECEIVING || receiveMode != MODE_SEGMENTS_AXIS) {
      Serial.println("ERR_RUN_SEGAX_STATE");
      return;
    }

    if (receivedCount != totalSegments) {
      Serial.println("ERR_RUN_SEGAX_COUNT");
      return;
    }

    currentState = STATE_READY_SEGMENTS;
    return;
  }

  Serial.println("ERR_UNKNOWN_CMD");
}

// ===============================
// 解析 BEGIN_START
// ===============================
bool parseBeginStart(const char* line, int& count) {
  return sscanf(line, "BEGIN_START,%d", &count) == 1;
}

// ===============================
// 解析 BEGIN_SEGAX
// ===============================
bool parseBeginSegAx(const char* line, int& count) {
  return sscanf(line, "BEGIN_SEGAX,%d", &count) == 1;
}

// ===============================
// 解析起点
// 格式: P,z,y,x
// ===============================
bool parseStartPoint(const char* line, StartPointData& p) {
  long z, y, x;

  int matched = sscanf(line, "P,%ld,%ld,%ld", &z, &y, &x);
  if (matched != 3) return false;

  p.z = z;
  p.y = y;
  p.x = x;
  p.valid = true;
  return true;
}

// ===============================
// 解析逐轴独立段
// 格式:
// A,z0,y0,x0,z1,y1,x1,
// J1(vs,ve,vp,aa,ad,Tja,Tjd,Ta,Tc,Td,Ttot),
// J2(...),
// J3(...)
// 共有 39 个整数
// ===============================
bool parseAxisSegmentLine(const char* line, SegmentAxisSCurve& seg) {
  long vals[39];

  int matched = sscanf(
    line,
    "A,%ld,%ld,%ld,%ld,%ld,%ld,"
    "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
    "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
    "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
    &vals[0],  &vals[1],  &vals[2],  &vals[3],  &vals[4],  &vals[5],
    &vals[6],  &vals[7],  &vals[8],  &vals[9],  &vals[10], &vals[11], &vals[12], &vals[13], &vals[14], &vals[15], &vals[16],
    &vals[17], &vals[18], &vals[19], &vals[20], &vals[21], &vals[22], &vals[23], &vals[24], &vals[25], &vals[26], &vals[27],
    &vals[28], &vals[29], &vals[30], &vals[31], &vals[32], &vals[33], &vals[34], &vals[35], &vals[36], &vals[37], &vals[38]
  );

  if (matched != 39) return false;

  seg.z0 = vals[0];
  seg.y0 = vals[1];
  seg.x0 = vals[2];
  seg.z1 = vals[3];
  seg.y1 = vals[4];
  seg.x1 = vals[5];

  AxisProfile* aps[3] = { &seg.zAxis, &seg.yAxis, &seg.xAxis };

  int idx = 6;
  for (int j = 0; j < 3; j++) {
    aps[j]->vs   = ((float)vals[idx++]) / 100.0f;
    aps[j]->ve   = ((float)vals[idx++]) / 100.0f;
    aps[j]->vp   = ((float)vals[idx++]) / 100.0f;
    aps[j]->aa   = ((float)vals[idx++]) / 100.0f;
    aps[j]->ad   = ((float)vals[idx++]) / 100.0f;
    aps[j]->Tja  = ((float)vals[idx++]) / 1000000.0f;
    aps[j]->Tjd  = ((float)vals[idx++]) / 1000000.0f;
    aps[j]->Ta   = ((float)vals[idx++]) / 1000000.0f;
    aps[j]->Tc   = ((float)vals[idx++]) / 1000000.0f;
    aps[j]->Td   = ((float)vals[idx++]) / 1000000.0f;
    aps[j]->Ttot = ((float)vals[idx++]) / 1000000.0f;

    if (aps[j]->Ttot < 0.0001f) aps[j]->Ttot = 0.0001f;
    if (aps[j]->Ta   < 0.0f)    aps[j]->Ta   = 0.0f;
    if (aps[j]->Tc   < 0.0f)    aps[j]->Tc   = 0.0f;
    if (aps[j]->Td   < 0.0f)    aps[j]->Td   = 0.0f;
    if (aps[j]->Tja  < 0.0f)    aps[j]->Tja  = 0.0f;
    if (aps[j]->Tjd  < 0.0f)    aps[j]->Tjd  = 0.0f;
  }

  return true;
}

// ===============================
// 起点平滑移动
// ===============================
void runStartMove() {
  stepperZ.setMaxSpeed(START_Z_MAX_SPEED);
  stepperZ.setAcceleration(START_Z_ACCEL);

  stepperY.setMaxSpeed(START_Y_MAX_SPEED);
  stepperY.setAcceleration(START_Y_ACCEL);

  stepperX.setMaxSpeed(START_X_MAX_SPEED);
  stepperX.setAcceleration(START_X_ACCEL);

  stepperZ.moveTo(startPoint.z);
  stepperY.moveTo(startPoint.y);
  stepperX.moveTo(startPoint.x);

  while (!allAxesArrived()) {
    stepperZ.run();
    stepperY.run();
    stepperX.run();
  }

  setAxesCurrentPosition(startPoint.z, startPoint.y, startPoint.x);
}

// ===============================
// 执行整条逐轴 S 曲线路径
// ===============================
void executeAxisSCurvePath() {
  for (int i = 0; i < totalSegments; i++) {
    bool ok = executeAxisSCurveSegment(segments[i], i);
    if (!ok) {
      Serial.print("ERR_SEG_EXEC,");
      Serial.println(i);
      return;
    }
  }
}

// ===============================
// 执行单段逐轴 S 曲线
// ===============================
bool executeAxisSCurveSegment(const SegmentAxisSCurve& seg, int segIndex) {
  setAxesCurrentPosition(seg.z0, seg.y0, seg.x0);

  stepperZ.moveTo(seg.z1);
  stepperY.moveTo(seg.y1);
  stepperX.moveTo(seg.x1);

  // MATLAB 端已同步，这里再取最大值保险
  float Tseg = seg.zAxis.Ttot;
  if (seg.yAxis.Ttot > Tseg) Tseg = seg.yAxis.Ttot;
  if (seg.xAxis.Ttot > Tseg) Tseg = seg.xAxis.Ttot;
  if (Tseg < 0.0001f) Tseg = 0.0001f;

  unsigned long startUs = micros();
  unsigned long lastUpdateUs = 0;

  while (!allAxesArrived()) {
    unsigned long nowUs = micros();
    float t = (nowUs - startUs) * 1e-6f;

    if ((nowUs - lastUpdateUs) >= SPEED_UPDATE_US) {
  float vz, vy, vx;

  if (t <= Tseg) {
    vz = fabs(evalAxisVelocity(t, seg.zAxis));
    vy = fabs(evalAxisVelocity(t, seg.yAxis));
    vx = fabs(evalAxisVelocity(t, seg.xAxis));
  } else {
    // ---------- 理论段时间结束后，进入追赶模式 ----------
    float remainTime = (Tseg + SEG_TIMEOUT_MARGIN) - t;
    if (remainTime < 0.02f) remainTime = 0.02f;   // 防止除零或速度过大

    long dz_rem = labs(seg.z1 - stepperZ.currentPosition());
    long dy_rem = labs(seg.y1 - stepperY.currentPosition());
    long dx_rem = labs(seg.x1 - stepperX.currentPosition());

    // 用“剩余步数 / 剩余时间”估算追赶速度
    vz = dz_rem / remainTime;
    vy = dy_rem / remainTime;
    vx = dx_rem / remainTime;

    // 设置追赶速度上下限
    const float MIN_CATCHUP_SPEED = 20.0f;
    const float MAX_CATCHUP_Z = 2000.0f;
    const float MAX_CATCHUP_Y = 600.0f;
    const float MAX_CATCHUP_X = 600.0f;

    if (dz_rem > 0) vz = constrain(vz, MIN_CATCHUP_SPEED, MAX_CATCHUP_Z); else vz = 0.0f;
    if (dy_rem > 0) vy = constrain(vy, MIN_CATCHUP_SPEED, MAX_CATCHUP_Y); else vy = 0.0f;
    if (dx_rem > 0) vx = constrain(vx, MIN_CATCHUP_SPEED, MAX_CATCHUP_X); else vx = 0.0f;
  }

  applyAxisSpeed(seg.z0, seg.z1, vz, stepperZ);
  applyAxisSpeed(seg.y0, seg.y1, vy, stepperY);
  applyAxisSpeed(seg.x0, seg.x1, vx, stepperX);

  lastUpdateUs = nowUs;
}

    if (stepperZ.distanceToGo() != 0) stepperZ.runSpeed();
    if (stepperY.distanceToGo() != 0) stepperY.runSpeed();
    if (stepperX.distanceToGo() != 0) stepperX.runSpeed();

    if (t > (Tseg + SEG_TIMEOUT_MARGIN)) {
      long zcur = stepperZ.currentPosition();
      long ycur = stepperY.currentPosition();
      long xcur = stepperX.currentPosition();

      long dz = labs(seg.z1 - zcur);
      long dy = labs(seg.y1 - ycur);
      long dx = labs(seg.x1 - xcur);

      Serial.print("SEG_DEBUG,");
      Serial.print(segIndex);
      Serial.print(",Tseg=");
      Serial.print(Tseg, 6);
      Serial.print(",t=");
      Serial.print(t, 6);
      Serial.print(",z0=");
      Serial.print(seg.z0);
      Serial.print(",z1=");
      Serial.print(seg.z1);
      Serial.print(",zcur=");
      Serial.print(zcur);
      Serial.print(",dz=");
      Serial.print(dz);

      Serial.print(",y0=");
      Serial.print(seg.y0);
      Serial.print(",y1=");
      Serial.print(seg.y1);
      Serial.print(",ycur=");
      Serial.print(ycur);
      Serial.print(",dy=");
      Serial.print(dy);

      Serial.print(",x0=");
      Serial.print(seg.x0);
      Serial.print(",x1=");
      Serial.print(seg.x1);
      Serial.print(",xcur=");
      Serial.print(xcur);
      Serial.print(",dx=");
      Serial.println(dx);

      if (dz <= 15 && dy <= 15 && dx <= 15) {
        break;
      } else {
        return false;
      }
    }
  }

  setAxesCurrentPosition(seg.z1, seg.y1, seg.x1);

  Serial.print("DONE_SEG,");
  Serial.println(segIndex);
  

  return true;
}

// ===============================
// 单轴 S 曲线速度求值
// ===============================
float evalAxisVelocity(float t, const AxisProfile& ap) {
  if (t <= 0.0f) return ap.vs;
  if (t >= ap.Ttot) return ap.ve;

  if (t < ap.Tja) {
    float j = ap.aa / max(ap.Tja, 1e-6f);
    return ap.vs + 0.5f * j * t * t;
  }
  else if (t < 2.0f * ap.Tja) {
    float dt = t - ap.Tja;
    return ap.vs + 0.5f * ap.aa * ap.Tja + ap.aa * dt;
  }
  else if (t < ap.Ta) {
    float d = ap.Ta - t;
    float j = -ap.aa / max(ap.Tja, 1e-6f);
    return ap.vp - 0.5f * fabs(j) * d * d;
  }
  else if (t < (ap.Ta + ap.Tc)) {
    return ap.vp;
  }
  else if (t < (ap.Ta + ap.Tc + ap.Tjd)) {
    float dt = t - (ap.Ta + ap.Tc);
    float j = -ap.ad / max(ap.Tjd, 1e-6f);
    return ap.vp + 0.5f * j * dt * dt;
  }
  else if (t < (ap.Ta + ap.Tc + 2.0f * ap.Tjd)) {
    float dt = t - (ap.Ta + ap.Tc + ap.Tjd);
    return (ap.vp - 0.5f * ap.ad * ap.Tjd) - ap.ad * dt;
  }
  else {
    float d = ap.Ttot - t;
    if (d < 0.0f) d = 0.0f;
    float j = ap.ad / max(ap.Tjd, 1e-6f);
    return ap.ve + 0.5f * j * d * d;
  }
}

// ===============================
// 根据段方向给某轴设置速度
// ===============================
void applyAxisSpeed(long p0, long p1, float vAbs, AccelStepper& stepper) {
  if (stepper.distanceToGo() == 0) {
    stepper.setSpeed(0.0f);
    return;
  }

  if (vAbs < MIN_STEP_SPEED) {
    vAbs = MIN_STEP_SPEED;
  }

  if (p1 < p0) {
    stepper.setSpeed(-vAbs);
  } else if (p1 > p0) {
    stepper.setSpeed(vAbs);
  } else {
    stepper.setSpeed(0.0f);
  }
}

// ===============================
// 是否全部到达
// ===============================
bool allAxesArrived() {
  return (stepperZ.distanceToGo() == 0 &&
          stepperY.distanceToGo() == 0 &&
          stepperX.distanceToGo() == 0);
}

// ===============================
// 强制设置当前位置
// ===============================
void setAxesCurrentPosition(long z, long y, long x) {
  stepperZ.setCurrentPosition(z);
  stepperY.setCurrentPosition(y);
  stepperX.setCurrentPosition(x);
}

// ===============================
// 归零逻辑
// ===============================
void manipZeroSetup(AccelStepper& stepperX, AccelStepper& stepperY, AccelStepper& stepperZ,
                    int xEndstopPin, int yEndstopPin, int zEndstopPin, int movementSpeed) {
  while (digitalRead(xEndstopPin) == LOW ||
         digitalRead(yEndstopPin) == LOW ||
         digitalRead(zEndstopPin) == LOW) {
    setMotorToZeroPoint(stepperX, xEndstopPin, -movementSpeed);
    setMotorToZeroPoint(stepperY, yEndstopPin,  movementSpeed);
    setMotorToZeroPoint(stepperZ, zEndstopPin, -movementSpeed);
  }
}

void setMotorToZeroPoint(AccelStepper& stepper, int endstopPin, int movementSpeed) {
  if (digitalRead(endstopPin) == HIGH) {
    stepper.setCurrentPosition(0);
    stepper.stop();
  } else {
    stepper.setMaxSpeed(1000);
    stepper.setSpeed(movementSpeed);
    stepper.runSpeed();
  }
}