#include <Servo.h>
#include <math.h>

static const double RHYTHM_EPSILON = 0.08;
#define AVERAGE_NUM 16
#define AVERAGE_NUM_SLOW (AVERAGE_NUM * 4)
#define RHYTHM_BUFFER_LENGTH 256
static const int RHYTHM_HYSTERISYS = 5;
static const int RHYTHM_MAX_COUNT = 10;
static const int RHYTHM_THRESHOLD = 35;

namespace meowsic {
  typedef struct {
    long sum;
    int head;
    int array[AVERAGE_NUM];
  } average_t;

  typedef struct {
    long sum;
    int array[AVERAGE_NUM_SLOW];
    int head;
  } average_slow_t;

  typedef struct {
    int prev_val;
  } differential_t;

  typedef struct {
    char counts[RHYTHM_BUFFER_LENGTH];
    int head;
    int count;
    char current_state;
  } rhythm_t;

  typedef struct {
    unsigned long prev_t;
    double prev_phy;
    double prev_rhythm;
    double offset;
  } theta_t;
}

int init_average(meowsic::average_t *ave) {
  ave->sum = 0;
  ave->head = 0;
  for (int i = 0; i < AVERAGE_NUM; i++) {
    ave->array[i] = 0;
  }
  return 0;
}

int init_rhythm(meowsic::rhythm_t *rhythm) {
  for (int i = 0; i < RHYTHM_BUFFER_LENGTH; i++) {
    rhythm->counts[i] = 0;
  }
    rhythm->head = 0;
    rhythm->count = 0;
    rhythm->current_state = 0;
    
    return 0;
}

int init_theta(meowsic::theta_t *theta) {
  theta->prev_phy = 0;
  theta->prev_rhythm = 0;
  theta->offset = 0;
  return 0;
}

int average(meowsic::average_t *ave, int audio) {
  /* static long sum = 0; */
  /* static int array[AVERAGE_NUM] = {}; */
  /* static int head = 0; */
  ave->sum -= ave->array[ave->head];
  ave->array[ave->head] = abs(audio);
  ave->sum += ave->array[ave->head];
  ave->head = (ave->head + 1) % AVERAGE_NUM;
  int average = ave->sum / AVERAGE_NUM;
  return average;
}

int init_average_slow(meowsic::average_slow_t *ave) {
  ave->sum = 0;
  ave->head = 0;
  for (int i = 0; i < AVERAGE_NUM_SLOW; i++) {
    ave->array[i] = 0;
  }
  return 0;
}

int average_slow(meowsic::average_slow_t *ave, int audio) {
  ave->sum -= ave->array[ave->head];
  ave->array[ave->head] = abs(audio);
  ave->sum += ave->array[ave->head];
  ave->head = (ave->head + 1) % AVERAGE_NUM_SLOW;
  int average = ave->sum / AVERAGE_NUM_SLOW;
  return average;
}

int init_differential(meowsic::differential_t *diff) {
  diff->prev_val = 0;
  return 0;
}

int differential(meowsic::differential_t *diff, int audio) {
  int result = audio - diff->prev_val;
  diff->prev_val = audio;
  return result;
}

double calculateRhythmFromAudio(meowsic::rhythm_t *rhythm, int diff) {
  /* 1 if audio > audio_slow, else 0  */
  /* static char counts[RHYTHM_BUFFER_LENGTH] = {}; */
  /* static int head = 0; */
  /* static int count = 0; */
  /* static char current_state = 0; */

  rhythm->count -= rhythm->counts[rhythm->head];
  if (rhythm->current_state) {
    if (diff < RHYTHM_THRESHOLD - RHYTHM_HYSTERISYS)
      rhythm->current_state = 0;
    rhythm->counts[rhythm->head] = 0;
  } else {
    if (diff > RHYTHM_THRESHOLD + RHYTHM_HYSTERISYS) {
      rhythm->current_state = 1;
      rhythm->counts[rhythm->head] = 1;
    } else {
      rhythm->counts[rhythm->head] = 0;
    }
  }
  rhythm->count += rhythm->counts[rhythm->head];
  rhythm->head = (rhythm->head + 1) % RHYTHM_BUFFER_LENGTH;
  
  return (double)rhythm->count / RHYTHM_MAX_COUNT;
}

double calculateVolumeFromAudio(int audio) {
  static const int audio_max = 128;
  double volume = abs((double)audio) / audio_max;
  if (volume > 1)
    volume = 1.0;
  if (volume < 0)
    volume = 0.0;
  return volume;

  /* static double prev_volume = 1; */
  /* static int cnt = 0; */
  /* if (cnt % 16 == 0) { */
  /*   double volume = abs((double)audio * 5 / audio_max) + 1; */
  /*   if(volume > 6) */
  /*     volume = 6; */
  /* } */

  /* prev_volume = volume + (prev_volume - volume) * cnt / 16; */
  /* cnt++; */
  /* return prev_volume; */
}
  /* if (rhythm * rhythm < RHYTHM_EPSILON * RHYTHM_EPSILON) */
  /*   rhythm = 0.0; */

int calculateTheta(meowsic::theta_t *theta, unsigned long t_us, double rhythm, double volume) {
  static const double gain = 0.5;

  double phy = ((double)t_us * 1.0E-6) * (0.2  + rhythm) * 2 * M_PI;
  if (abs(rhythm - theta->prev_rhythm) > RHYTHM_EPSILON) {
    double prev_phy = ((double)theta->prev_t * 1.0E-6) * (0.2  + rhythm) * 2 * M_PI;
    theta->offset = theta->prev_phy - prev_phy;
  }
  phy += theta->offset;
 
  int degree = volume * 60 * sin(phy) + 90;
  if (degree > 180)
    degree = 180;
  if (degree < 0)
    degree = 0;

  theta->prev_t = t_us;
  theta->prev_phy = phy;
  theta->prev_rhythm = rhythm;
  return degree;
}

Servo servo1;
Servo servo2;

void moveServo(int degree, int rl) {
  if(rl == 1)
    servo1.write(degree);
  else
    servo2.write(degree);
}

meowsic::average_t left_average;
meowsic::average_t right_average;
meowsic::average_slow_t left_average_slow;
meowsic::average_slow_t right_average_slow;
meowsic::rhythm_t left_rhythm;
meowsic::rhythm_t right_rhythm;
meowsic::differential_t left_diff;
meowsic::differential_t right_diff;
meowsic::theta_t left_theta;
meowsic::theta_t right_theta;

void setup() {
  // put your setup code here, to run once:
  servo1.attach(3, 750, 1850);
  servo2.attach(9, 750, 1850);
  Serial.begin(115200);
  init_average(&left_average);
  init_average(&right_average);
  init_average_slow(&left_average_slow);
  init_average_slow(&right_average_slow);
  init_rhythm(&left_rhythm);
  init_rhythm(&right_rhythm);
  init_differential(&left_diff);
  init_differential(&right_diff);
  init_theta(&left_theta);
  init_theta(&right_theta);
}


typedef enum {
  LEFT = 1,
  RIGHT
} rl_t;

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long prev_t = 0;
  static int count = 0;
  unsigned long t = prev_t;

  rl_t rl = RIGHT;
  int audio = (rl == RIGHT) ? (int)analogRead(0) - 512 : (int)analogRead(1) - 512;
  meowsic::average_t *average_obj = (rl == RIGHT) ? &right_average : &left_average;
  meowsic::average_slow_t *average_slow_obj = (rl == RIGHT) ? &right_average_slow : &left_average_slow;
  meowsic::differential_t *diff_obj = (rl == RIGHT) ? &right_diff : &left_diff;
  meowsic::rhythm_t *rhythm_obj = (rl == RIGHT) ? &right_rhythm : &left_rhythm;
  meowsic::theta_t *theta_obj = (rl == RIGHT) ? &right_theta : &left_theta;
  
  int ave = average(average_obj, audio);
  int ave_slow = average_slow(average_slow_obj, audio);
  int diff = differential(diff_obj, audio);
  double rhythm = calculateRhythmFromAudio(rhythm_obj, diff);
  double volume = calculateVolumeFromAudio(ave);
  int theta = calculateTheta(theta_obj, t, rhythm, volume);
  
  moveServo(theta, RIGHT);
  moveServo(theta, LEFT);
  if (rl == RIGHT) {
    Serial.print(t/100);
    Serial.print("\t");
    Serial.print(audio);
    Serial.print("\t");
    Serial.print(ave);
    Serial.print("\t");
    Serial.print(volume);
    Serial.print("\t");
    Serial.print(theta);
    Serial.print("\t");
    Serial.print(diff);
    Serial.print("\t");
    Serial.print(ave_slow);
    Serial.print("\t");
    Serial.print(rhythm_obj->count);
    Serial.print("\t");
    Serial.print(theta_obj->offset);
    Serial.println("");
  }
  while ((t = micros()) - prev_t < 10000);
  prev_t = t;
  count++;
}
