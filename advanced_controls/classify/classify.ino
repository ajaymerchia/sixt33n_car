/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define KMEANS_THRESHOLD            0.025
#define LOUDNESS_THRESHOLD          700

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

char *WORDS[4] = {"Matrix", "Slothman", "Longo", "Rhodes"};

float pca_vec1[SNIPPET_SIZE] = {0.07303513, 0.08927463, 0.09172529, 0.09779544, 0.08882192, 0.13023121, 0.12648877, 0.14069465, 0.16536234, 0.16803317, 0.21969414, 0.24057043, 0.23715149, 0.20591958, 0.1734393, 0.08417365, 0.02336402, -0.03910211, -0.09858951, -0.13957565, -0.18712479, -0.2194567, -0.24081492, -0.24627835, -0.23744425, -0.22066802, -0.19628548, -0.18064473, -0.17000051, -0.17500119, -0.16705568, -0.15415757, -0.13782068, -0.10969682, -0.04712873, 0.00065975, 0.04951016, 0.07931035, 0.10479378, 0.09572702, 0.06761212, 0.08042669, 0.07649149, 0.06535201, 0.05288225, 0.02217049, 0.00713183, -0.00220252, -0.00668116, -0.00774377, -0.00145841, -0.00401459, 0.00165748, 0.01311646, 0.00990038, 0.01286626, 0.02068232, 0.01848974, 0.01080403, 0.0152828, 0.00324602, -0.00558437, -0.00872378, -0.00874819, -0.01156671, -0.01027059, -0.00886234, -0.01430264, -0.00883179, -0.00891563, -0.00943644, -0.0067163, -0.00797853, -0.00947391, -0.00600518, -0.00923144, -0.00894197, -0.01018935, -0.01129258, -0.00987072};
float pca_vec2[SNIPPET_SIZE] = {0.00087719, -0.00781619, -0.01471974, -0.04443223, -0.04889156, -0.04620033, 0.01163616, -0.00452001, -0.01471285, -0.02062393, -0.06201363, -0.09579485, -0.10541994, -0.07595999, 0.03182724, 0.11328852, 0.16033165, 0.20155354, 0.19450315, 0.1721077, 0.13081943, 0.06666703, 0.01483963, -0.03024058, -0.05914867, -0.07787885, -0.09428742, -0.10145868, -0.11649529, -0.13717454, -0.1392961, -0.14567643, -0.19008704, -0.212356, -0.23678561, -0.27333353, -0.27789686, -0.24322782, -0.23125014, -0.16420673, -0.07258106, 0.00327503, 0.05875088, 0.11301998, 0.13293775, 0.15716798, 0.16010008, 0.1590751, 0.16527616, 0.15468047, 0.14351292, 0.12701871, 0.11564501, 0.09333807, 0.0940357, 0.07250219, 0.07662128, 0.06471379, 0.06053781, 0.05437933, 0.04501428, 0.04040769, 0.0227305, 0.02493387, 0.02074035, 0.01759965, 0.01141756, 0.00632051, 0.00746151, 0.00446162, 0.00516734, 0.00218364, 0.00167789, 0.00297479, 0.00328384, 0.00536338, 0.00579649, 0.0051229, 0.00294458, 0.00384474};
float mean_vec[SNIPPET_SIZE] = {0.00747644, 0.00899511, 0.01112506, 0.01370221, 0.0161115 ,0.02376586, 0.02607111, 0.02716335, 0.02874066, 0.02953445,0.0312359 , 0.03315923, 0.03441238, 0.03462737, 0.03273681,0.02928764, 0.02690872, 0.0245775 , 0.02291758, 0.02059618,0.01896802, 0.01607425, 0.01436676, 0.01295074, 0.01228586,0.01149051, 0.01145793, 0.0113405 , 0.01172604, 0.0122617 ,0.01240871, 0.01231487, 0.01295564, 0.01311831, 0.01338612,0.01459963, 0.01524599, 0.01474631, 0.01541615, 0.01471177,0.01416948, 0.01403809, 0.01347965, 0.01300742, 0.0119424 ,0.01056868, 0.01003968, 0.00955494, 0.00916519, 0.0086207 ,0.00832993, 0.00770929, 0.00761152, 0.00731549, 0.00678713,0.00703384, 0.00700866, 0.00652733, 0.00643934, 0.0056395 ,0.00510942, 0.00467148, 0.00393331, 0.0035268 , 0.00353311,0.00329054, 0.00305687, 0.00287466, 0.00275811, 0.00283395,0.0028356 , 0.00271637, 0.00252241, 0.00244703, 0.00235142,0.00232682, 0.00238101, 0.00233596, 0.00236807, 0.00216792};
float centroid1[2] = {0.042005485743222173, -0.034091230827706896};
float centroid2[2] = {0.024351817731659075, 0.027384539581265955};
float centroid3[2] = {-0.017262835502896576, 0.031137979864624875};
float centroid4[2] = {-0.049094467971984676, -0.024431288618183961};
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

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      Serial.println("-------------------");

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' on the principal components
      // YOUR CODE HERE
      printArr(result, 80);
      // subtract mean vec from result
      subtract(mean_vec, result, result, SNIPPET_SIZE);
      // project ^ onto PC1, PC2
      printArr(result, 80);


      float projection[2] = {dot_product(result, pca_vec1, SNIPPET_SIZE), dot_product(result, pca_vec2, SNIPPET_SIZE)};

      printArr(projection, 2);

      // Classification
      // Use the function l2_norm defined above
      // ith centroids: centroids[i]
      // YOUR CODE HERE
      float distances[4] = {0,0,0,0};
      for (int i = 0; i < 4; i++) {
          distances[i] = l2_norm(projection[0], projection[1], centroids[i]);
      }

      int smallest = index_of_smallest(distances, 4);
      Serial.println("Smallest Distance");
      Serial.println(distances[smallest]);

      Serial.print("Predicted Word: ");
      Serial.println(WORDS[smallest]);
      Serial.println("-------------------");

      // Check against KMEANS_THRESHOLD and print result over serial
      if (distances[smallest] < KMEANS_THRESHOLD) {
          Serial.print("Word Identified: ");
          Serial.println(WORDS[smallest]);
      } else {
          Serial.println("Above KMEANS_THRESHOLD.");
      }


      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }


    delay(2000);
    re_pointer = 0;
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

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
