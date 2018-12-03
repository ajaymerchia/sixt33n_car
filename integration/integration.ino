/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_3
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3
//char *WORDS[4] = {"Matrix", "Modify", "EECS", "Rhodes"};
char *MODES[4] = {"DRIVE_FAR", "DRIVE_LEFT", "DRIVE_CLOSE", "DRIVE_RIGHT"};



#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.283;
float theta_right = 0.2885;
float beta_left = -32.09;
float beta_right = -31.56;
float v_star = 81.8;

// PWM inputs to jolt the car straight
int left_jolt = 240;
int right_jolt = 230;

// Control gains
float k_left = 0.5;
float k_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left_OL(float v_star) {
  return (v_star + beta_left) / theta_left;
}

float driveStraight_right_OL(float v_star) {
  return (v_star + beta_right) / theta_right;
}

float driveStraight_left(float delta) {
  return driveStraight_left_OL(v_star) - (k_left / theta_left) * delta;
}

float driveStraight_right(float delta) {
  return driveStraight_right_OL(v_star) + (k_right / theta_right) * delta;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
//#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
 #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {3500, 2000, 1500, 1500};

float delta_reference(int k) {
  // YOUR CODE HERE
  float car_delta = CAR_WIDTH * (v_star / 5) * k / TURN_RADIUS;
  if (drive_mode == DRIVE_RIGHT) {
    return car_delta;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -car_delta;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define KMEANS_THRESHOLD            0.041
#define LOUDNESS_THRESHOLD          500

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/
char *WORDS[4] = {"Matrix", "Modify", "EECS", "Rhodes"};

float pca_vec1[SNIPPET_SIZE] = {-0.00376866, -0.0189012, -0.03516227, -0.05661686, -0.07957052, -0.16367538, -0.16697057, -0.19867985, -0.19598158, -0.20335335, -0.21290054, -0.21741, -0.22396845, -0.23817383, -0.2430003, -0.21232477, -0.14828045, -0.09743033, 0.00170277, 0.05552989, 0.09560981, 0.09041017, 0.11476945, 0.13641752, 0.15187874, 0.14089254, 0.10872868, 0.06601898, 0.02715708, 0.00963226, 0.00533542, 0.0046061, -0.00577104, -0.00902716, -0.02531295, -0.09343613, -0.15046963, -0.17061931, -0.15684876, -0.12722816, -0.0868667, -0.02434577, 0.0444341, 0.08483636, 0.11691294, 0.13160803, 0.13240606, 0.14408423, 0.13930163, 0.14573859, 0.14387171, 0.13165882, 0.13413329, 0.13102033, 0.11988351, 0.09707641, 0.09125536, 0.0767251, 0.07715464, 0.06293596, 0.05549859, 0.05554385, 0.03320784, 0.02610693, 0.02876296, 0.02956124, 0.02576383, 0.02555701, 0.02177625, 0.02338174, 0.02623668, 0.02989526, 0.02570069, 0.0271193, 0.02242742, 0.0208822, 0.01817166, 0.01651639, 0.02040192, 0.01985631};
float pca_vec2[SNIPPET_SIZE] = {0.00456921, 0.00520503, 0.00842498, 0.00822722, -0.01099322, -0.06990705, -0.06855829, -0.06978631, -0.08339379, -0.09612039, -0.11262855, -0.10920118, -0.10149134, -0.0811602, -0.09418292, -0.09045398, -0.09354196, -0.0832073, -0.04146102, 0.02145135, 0.08166037, 0.13758398, 0.14115486, 0.13331641, 0.12537355, 0.12684868, 0.17409241, 0.18396688, 0.2133388, 0.23352616, 0.23149893, 0.2271428, 0.23426203, 0.23071837, 0.21989566, 0.2006303, 0.17022927, 0.14814633, 0.12986354, 0.09056051, 0.03752583, -0.02945168, -0.07162491, -0.10044903, -0.11670835, -0.12375705, -0.12186395, -0.12817324, -0.13060638, -0.13278976, -0.13203462, -0.12113719, -0.12377134, -0.12429104, -0.11872753, -0.11360166, -0.11110601, -0.09726462, -0.09033617, -0.08289973, -0.07321414, -0.05913734, -0.04696506, -0.04464656, -0.03472131, -0.02773692, -0.02049456, -0.01142175, -0.00232078, 0.00553358, 0.00927925, 0.01503708, 0.0120978, 0.0105578, 0.00676795, 0.00299651, 0.00176984, 0.00188907, 0.00630477, 0.00589305};
float mean_vec[SNIPPET_SIZE] = {0.00399398, 0.00502615, 0.00661467, 0.00966489, 0.01482133,0.02524292, 0.0270099 , 0.02984182, 0.03086195, 0.03088859,0.03164445, 0.03192202, 0.03344005, 0.03464468, 0.03505199,0.03408412, 0.03086367, 0.02829015, 0.02410972, 0.02002936,0.01653111, 0.01340649, 0.012511 , 0.01234259, 0.01223567,0.01146354, 0.01072923, 0.00970238, 0.010152 , 0.01051716,0.01038071, 0.00992957, 0.01064611, 0.01041216, 0.01063197,0.01306282, 0.01518269, 0.01533172, 0.01492973, 0.01498699,0.01600839, 0.01598685, 0.01390398, 0.01237792, 0.0113902 ,0.01030926, 0.00937847, 0.00917093, 0.00856194, 0.00855091,0.00841066, 0.00780958, 0.00780803, 0.00775789, 0.00737416,0.00747628, 0.00754936, 0.0070588 , 0.00677002, 0.00632827,0.00599724, 0.00543301, 0.00476897, 0.00484417, 0.00447144,0.00436476, 0.00400027, 0.00377739, 0.00351611, 0.00361046,0.00357337, 0.00367347, 0.00344692, 0.00338573, 0.00322196,0.00313505, 0.00299099, 0.00291908, 0.00289898, 0.0028566};
float centroid1[2] = {-0.041340215326619349, 0.0049270033595849132};
float centroid2[2] = {0.058804178230909367, -0.035219504668458419};
float centroid3[2] = {-0.035170221675216252, -0.027934645753971261};
float centroid4[2] = {0.017706258770926235, 0.058227147062844757};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

/*---------------------------*/
/*      Helper functions     */
/*---------------------------*/
void subtract(float *x, float *y, float *result, int numElem) {
    for (int i = 0 ; i < numElem ; i++)
        result[i] = y[i] - x[i];
}

float dot_product(float *v1, float *v2, int numElem) {
    float result = 0;
    for (int i = 0; i < numElem; i++){
      result = result + v1[i] * v2[i];
    }
    return result;
}

int index_of_smallest(float *arr, int numElem) {
    int smallest = 0;
    for(int i = 1; i < numElem; i++) {
            if(arr[i] < arr[smallest])
                    smallest = i;
    }

    return smallest;
}

void printArr(float *arr, int numElem) {
    Serial.print("[");
    for (int i = 0; i < numElem; i++) {
      Serial.print(arr[i]);
      Serial.print(", ");
    }
    Serial.println("]");
}




void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i <= 4; i++) {
    sample_lens[i] = run_times[i]/SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){

    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' on the principal components
      // YOUR CODE HERE
      subtract(mean_vec, result, result, SNIPPET_SIZE);
      proj1 = dot_product(result, pca_vec1, SNIPPET_SIZE);
      proj2 = dot_product(result, pca_vec2, SNIPPET_SIZE);


      // Classification
      // Use the function l2_norm defined above
      // ith centroid: centroids[i]
      // YOUR CODE HERE
      float distances[4] = {0};
      for (int i = 0; i < 4; i++) {
          distances[i] = l2_norm(proj1, proj2, centroids[i]);
      }

      int smallest = index_of_smallest(distances, 4);
      Serial.println("Smallest Distance");
      Serial.println(distances[smallest]);

      Serial.print("Predicted Word: ");
      Serial.println(WORDS[smallest]);
      Serial.println("-------------------");
      

      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (distances[smallest] < KMEANS_THRESHOLD) {
        Serial.print("Word Identified: ");
          Serial.println(WORDS[smallest]);
        drive_mode = smallest; // from 0-3, inclusive
        start_drive_mode();
      } else {
                  Serial.println("Above KMEANS_THRESHOLD.");

      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    } else {
        Serial.println("Below LOUDNESS_THRESHOLD.");
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);

      Serial.print("Drive Mode: ");
      Serial.println(MODES[drive_mode]);
      Serial.println("Writing Power");
      Serial.print("(");
      Serial.print(left_cur_pwm);
      Serial.print(",");
      Serial.print(right_cur_pwm);
      Serial.println(")");
      
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
