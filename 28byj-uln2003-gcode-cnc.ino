// version 3

unsigned long step_delay = 2000; // 14=2092, 15=1953, 16=1831 rpm=microseconds
int axisX[5] = {2,3,5,4, -1}; // in1, in2, in3, in4, pin 5 is for step sequence -1
int axisY[5] = {6,8,9,7, -1}; // in1, in2, in3, in4, pin 5 is for step sequence -1
int axisZ[5] = {10,12,13,11, -1}; // in1, in2, in3, in4, pin 5 is for step sequence -1

// Motor steps to go 1 millimeter.
// Use test sketch to go 100 steps. Measure the length of line.
// Calculate steps per mm. Enter here.
float StepsPerMillimeterX = 50.0;
float StepsPerMillimeterY = 50.0;
float StepsPerMillimeterZ = 50.0;

// Drawing robot limits, in mm
// OK to start with. Could go up to 85 mm if calibrated well.
float Xmin = 0;
float Xmax = 80;
float Ymin = 0;
float Ymax = 80;
float Zmin = 0; // max penetration depth
float Zmax = 50;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelay = 0;
int LineDelay = 0;
int penDelay = 10;

#define LINE_BUFFER_LENGTH 512

#define MOTION_MODE_SEEK 0 // G0 (Default: Must be zero)
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_NONE 8 // G80

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value. 
#define Y_AXIS 1
#define Z_AXIS 2

float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmin;

// Current position of plothead
float currentPos[N_AXIS];
boolean xyzSet[N_AXIS];

// Set to true to get debug output.
boolean verbose = false;

/**********************
   void setup() - Initialisations
 ***********************/
void setup() {
  //  Setup

  Serial.begin( 9600 );
  delay(100);

  enableAxis(axisX);
  enableAxis(axisY);
  enableAxis(axisZ);
  
  //  Set & move to initial default position
  // TBD
  currentPos[X_AXIS] = Xmin;
  currentPos[Y_AXIS] = Ymin;
  currentPos[Z_AXIS] = Zmin;

  //  Notifications!!!
  Serial.println("28BYJ-48 Stepper Mini CNC Plotter");
  Serial.print("X range is from ");
  Serial.print(Xmin);
  Serial.print(" to ");
  Serial.print(Xmax);
  Serial.println(" mm.");
  Serial.print("Y range is from ");
  Serial.print(Ymin);
  Serial.print(" to ");
  Serial.print(Ymax);
  Serial.println(" mm.");
  Serial.print("Z range is from ");
  Serial.print(Zmin);
  Serial.print(" to ");
  Serial.print(Zmax);
  Serial.println(" mm.");
}

void enableAxis(int axis[]) {
  for (int pin=0; pin<4; pin++){
    pinMode(axis[pin], OUTPUT);
  }
}

/**********************
   void loop() - Main loop
 ***********************/
void loop()
{


  delay(100);
  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (1) {

    // Serial reception - Mostly from Grbl, added semicolon support
    while ( Serial.available() > 0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {             // End of line reached
        if ( lineIndex > 0 ) {                        // Line is complete. Then execute!
          line[ lineIndex ] = '\0';                   // Terminate string
          if (verbose) {
            Serial.print( "Received : ");
            Serial.println( line );
          }
          processIncomingLine( line );
          lineIndex = 0;
        }
        else {
          // Empty or comment line. Skip block.
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");
      }
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   // Throw away all comment characters
          if ( c == ')' )  lineIsComment = false;     // End of comment. Resume line.
        }
        else {
          if ( c <= ' ' ) {                           // Throw away whitepace and control characters
          }
          else if ( c == '/' ) {                    // Block delete not supported. Ignore character.
          }
          else if ( c == '%' ) {                    // Block delete not supported. Ignore character.
          }
          else if ( c == '(' ) {                    // Enable comments flag and ignore all characters until ')' or EOL.
            lineIsComment = true;
          }
          else if ( c == ';' ) {
            lineSemiColon = true;
          }
          else if ( lineIndex >= LINE_BUFFER_LENGTH - 1 ) {
            Serial.println( "ERROR - lineBuffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          }
          else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase
            line[ lineIndex++ ] = c - 'a' + 'A';
          }
          else {
            line[ lineIndex++ ] = c;
          }
        }
      }
    }
  }
}

void processIncomingLine( char* line ) {

  float xyzPos[N_AXIS];
  xyzPos[X_AXIS] = 0.0;
  xyzPos[Y_AXIS] = 0.0;
  xyzPos[Z_AXIS] = 0.0;

  xyzSet[X_AXIS] = false;
  xyzSet[Y_AXIS] = false;
  xyzSet[Z_AXIS] = false;

  float ijkPos[N_AXIS];
  ijkPos[X_AXIS] = 0.0;
  ijkPos[Y_AXIS] = 0.0;
  ijkPos[Z_AXIS] = 0.0;

  float rPos = 0.0;
  float feedRate = 0.0;

  uint8_t char_counter = 0;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint8_t motion = MOTION_MODE_NONE;

  while ( line[char_counter] != 0 ) {

    letter = line[char_counter];
    char_counter++;
    if (!read_float(line, &char_counter, &value)) {
      Serial.println( "BAD_NUMBER_FORMAT" );
    }
    int_value = trunc(value);

    switch ( letter ) {                              // Select command, if any
      case 'G':
        switch ( int_value ) {                         // Select G command
          case 0: motion = MOTION_MODE_SEEK; break;    // G0
          case 1: motion = MOTION_MODE_LINEAR; break;  // G1
          case 2: motion = MOTION_MODE_CW_ARC; break;  // G2
          case 3: motion = MOTION_MODE_CCW_ARC; break; // G3
        }
        break;
      case 'M':
        switch ( int_value ) {
          case 3:                                // using M3 from gcode tools e.g. (spindle on)
            //Serial.println( "Drives ON " );
            //Serial.println( "Home Position " );  // TO DO!!!! assume we are already home!!!!!!!!!!!!!
            break;
          case 5:                                 // using M5 from gcode tools e.g. (spindle off)
            //Serial.println( "Drives OFF " );
            //disableSteppers();
            break;
          case 2:                                 // using M2 program end
            Serial.println( "Finished! " );
            disableSteppers();
            break;
          default:
            Serial.print( "[Unsupported M command] M");
            Serial.println( int_value );
        }
      default:
        switch (letter) {
          case 'F': feedRate = value; break;
          case 'I': ijkPos[X_AXIS] = value; break;
          case 'J': ijkPos[Y_AXIS] = value; break;
          case 'K': ijkPos[Z_AXIS] = value; break;
          case 'R': rPos = value; break;
          case 'X': xyzPos[X_AXIS] = value; xyzSet[X_AXIS] = true; break;
          case 'Y': xyzPos[Y_AXIS] = value; xyzSet[Y_AXIS] = true; break;
          case 'Z': xyzPos[Z_AXIS] = value; xyzSet[Z_AXIS] = true; break;
          case 'M': break;
          default:
            Serial.print( "UNSUPPORTED_GCODE_COMMAND ");
            Serial.print( letter );
            Serial.println( int_value );
        }
    }
  }

  switch (motion) {
    case MOTION_MODE_NONE:
      break;
    case MOTION_MODE_SEEK:
      drawLine(xyzPos);
      break;
    case MOTION_MODE_LINEAR:
      drawLine(xyzPos);
      break;
    case MOTION_MODE_CW_ARC:
      mc_arc(currentPos, xyzPos, ijkPos, rPos, feedRate, 0, X_AXIS, Y_AXIS, Z_AXIS, true);
      break;
    case MOTION_MODE_CCW_ARC:
      mc_arc(currentPos, xyzPos, ijkPos, rPos, feedRate, 0, X_AXIS, Y_AXIS, Z_AXIS, false);
      break;
  }
}

/*********************************
   Draw a line from (x0;y0) to (x1;y1).
   int (x1;y1) : Starting coordinates
   int (x2;y2) : Ending coordinates
 **********************************/
void drawLine(float *target) {

  float x1, y1, z1;

  // Check for any changes
  if (xyzSet[X_AXIS]) {
    x1 = target[X_AXIS];
  }
  else {
    x1 = currentPos[X_AXIS];
  }
  if (xyzSet[Y_AXIS]) {
    y1 = target[Y_AXIS];
  }
  else {
    y1 = currentPos[Y_AXIS];
  }
  if (xyzSet[Z_AXIS]) {
    z1 = target[Z_AXIS];
  }
  else {
    z1 = currentPos[Z_AXIS];
  }

  if (verbose)
  {
    Serial.print("current position XYZ: ");
    Serial.print(currentPos[X_AXIS]);
    Serial.print(", ");
    Serial.print(currentPos[Y_AXIS]);
    Serial.print(", ");
    Serial.print(currentPos[Z_AXIS]);
    Serial.println("");
  }

  //  Bring instructions within limits
  if (x1 >= Xmax) {
    x1 = Xmax;
  }
  if (x1 <= Xmin) {
    x1 = Xmin;
  }
  if (y1 >= Ymax) {
    y1 = Ymax;
  }
  if (y1 <= Ymin) {
    y1 = Ymin;
  }
  if (z1 >= Zmax) {
    z1 = Zmax;
  }
  if (z1 <= Zmin) {
    z1 = Zmin;
  }

  if (verbose)
  {
    Serial.print("Target X, Y, Z: ");
    Serial.print(x1);
    Serial.print(", ");
    Serial.print(y1);
    Serial.print(", ");
    Serial.print(z1);
    Serial.println("");
  }

  //  Convert coordinates to steps
  float x2 = (int)(x1 * StepsPerMillimeterX);
  float y2 = (int)(y1 * StepsPerMillimeterY);
  float z2 = (int)(z1 * StepsPerMillimeterZ);

  //  Let's find out the change for the coordinates
  long dx = abs(x2 - Xpos);
  long dy = abs(y2 - Ypos);
  long dz = abs(z2 - Zpos);
  int sx = Xpos < x2 ? StepInc : -StepInc;
  int sy = Ypos < y2 ? StepInc : -StepInc;
  int sz = Zpos < z2 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dz != 0) {
    for (i = 0; i < dz; ++i) {
      step(axisZ, sz);
      delay(StepDelay);
    }
    if (verbose) {
      Serial.println("Move Z");
    }
    delay(penDelay);
  }

  if (dx != 0 || dy != 0) {
    if (dx > dy) {
      for (i = 0; i < dx; ++i) {
        step(axisX, sx);
        over += dy;
        if (over >= dx) {
          over -= dx;
          step(axisY, sy);
        }
        delay(StepDelay);
      }
      if (verbose) {
        Serial.println("Move XY");
      }
    }
    else {
      for (i = 0; i < dy; ++i) {
        step(axisY, sy);
        over += dx;
        if (over >= dy) {
          over -= dy;
          step(axisX, sx);
        }
        delay(StepDelay);
      }
      if (verbose) {
        Serial.println("Move YX");
      }
    }
  }

  if (verbose)
  {
    Serial.print("distance dx, dy, dz: ");
    Serial.print(dx);
    Serial.print(", ");
    Serial.print(dy);
    Serial.print(", ");
    Serial.print(dz);
    Serial.println("");
  }
  if (verbose)
  {
    Serial.print("current Xpos, Ypos, Zpos: ");
    Serial.print(Xpos);
    Serial.print(", ");
    Serial.print(Ypos);
    Serial.print(", ");
    Serial.print(Zpos);
    Serial.println("");
  }
  if (verbose)
  {
    Serial.print("target Xpos, Ypos, Zpos (");
    Serial.print(x2);
    Serial.print(", ");
    Serial.print(y2);
    Serial.print(", ");
    Serial.print(z2);
    Serial.println(")");
    Serial.println("");
  }

  //  Update the positions
  currentPos[X_AXIS] = x1;
  currentPos[Y_AXIS] = y1;
  currentPos[Z_AXIS] = z1;

  Xpos = x2;
  Ypos = y2;
  Zpos = z2;

  //  Delay before any next lines are submitted
  delay(LineDelay);
}

void disableSteppers() {
  // assume we are using default output pins
  int i;
  for (i = 2; i < 14; i++) {
    digitalWrite(i, LOW);
  }
}

void setSpeed(long rpm)
{
  step_delay = 60L * 1000L * 1000L / 2048 / rpm;
}

void step(int *axis, int forward){
  
  if (forward > 0){
    axis[4]++;
    if (axis[4] > 3) axis[4] = 0;
    seq(axis, axis[4]);
  }
  else {
    axis[4]--;
    if (axis[4] < 0) axis[4] = 3;
    seq(axis, axis[4]);
  }
}

void seq (int *axis, int seqNum){

  int pattern[4];
  
  switch(seqNum){
    case 0:
    {
      pattern[0] = 1;
      pattern[1] = 1;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
    }
    case 1:
    {
      pattern[0] = 0;
      pattern[1] = 1;
      pattern[2] = 1;
      pattern[3] = 0;
      break;
    }
    case 2:
    {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 1;
      pattern[3] = 1;
      break;
    }
    case 3:
    {
      pattern[0] = 1;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 1;
      break;
    }
    default:
    {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
    }
  }

  // write pattern to pins
  for (int p=0; p<4; p++){
    digitalWrite(axis[p], pattern[p]);
  }
  delayMicroseconds(step_delay);
}

/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)
#define DEFAULT_ARC_TOLERANCE 0.002 // mm
#define N_ARC_CORRECTION 12 // Integer (1-255)

// Execute an arc in offset mode format. position == current xyz, target == target xyz,
// offset == offset from current xyz, axis_X defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
// The arc is approximated by generating a huge number of tiny, linear segments. The chordal tolerance
// of each segment is configured in settings.arc_tolerance, which is defined to be the maximum normal
// distance from segment to the circle when the end points both lie on the circle.
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
            uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
  if (is_clockwise_arc) { // Correct atan2 output per direction
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) {
      angular_travel -= 2 * M_PI;
    }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) {
      angular_travel += 2 * M_PI;
    }
  }

  // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
  // (2x) settings.arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
  // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
  // For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases.
  uint16_t segments = floor(fabs(0.5 * angular_travel * radius) /
                            sqrt(DEFAULT_ARC_TOLERANCE * (2 * radius - DEFAULT_ARC_TOLERANCE)) );

  if (segments) {
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
    // all segments.
    if (invert_feed_rate) {
      feed_rate *= segments;
    }

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear]) / segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. Single precision values can accumulate error greater than tool precision in rare
       cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
       expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.

       Small angle approximation may be used to reduce computation overhead further. A third-order approximation
       (second order sin() has too much error) holds for most, if not, all CNC applications. Note that this
       approximation will begin to accumulate a numerical drift error when theta_per_segment is greater than
       ~0.25 rad(14 deg) AND the approximation is successively used without correction several dozen times. This
       scenario is extremely unlikely, since segment lengths and theta_per_segment are automatically generated
       and scaled by the arc tolerance setting. Only a very large arc tolerance setting, unrealistic for CNC
       applications, would cause this numerical drift error. However, it is best to set N_ARC_CORRECTION from a
       low of ~4 to a high of ~20 or so to avoid trig operations while keeping arc generation accurate.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    float cos_T = 2.0 - theta_per_segment * theta_per_segment;
    float sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;

    for (i = 1; i < segments; i++) { // Increment (segments-1).

      if (count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix. ~40 usec
        r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
        r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        cos_Ti = cos(i * theta_per_segment);
        sin_Ti = sin(i * theta_per_segment);
        r_axis0 = -offset[axis_0] * cos_Ti + offset[axis_1] * sin_Ti;
        r_axis1 = -offset[axis_0] * sin_Ti - offset[axis_1] * cos_Ti;
        count = 0;
      }

      // Update arc_target location
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;

      //mc_line(position, feed_rate, invert_feed_rate);
      drawLine(position);

    }
  }
  // Ensure last segment arrives at target location.
  //mc_line(target, feed_rate, invert_feed_rate);
  drawLine(target);
}

/*
  nuts_bolts.c - Shared functions
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float)


// Extracts a floating point value from a string. The following code is based loosely on
// the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
// available conversion method examples, but has been highly optimized for Grbl. For known
// CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
// Scientific notation is officially not supported by g-code, and the 'E' character may
// be a g-code word on some CNC systems. So, 'E' notation will not be recognized.
// NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)
{
  char *ptr = line + *char_counter;
  unsigned char c;

  // Grab first character and increment pointer. No spaces assumed in line.
  c = *ptr++;

  // Capture initial positive/minus character
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }

  // Extract number into fast integer. Track decimal in terms of exponent value.
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while (1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) {
          exp--;
        }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) {
          exp++;  // Drop overflow digits
        }
      }
    } else if (c == (('.' - '0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }

  // Return if no digits have been read.
  if (!ndigit) {
    return (false);
  };

  // Convert integer into floating point.
  float fval;
  fval = (float)intval;

  // Apply decimal. Should perform no more than two floating point multiplications for the
  // expected range of E0 to E-4.
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01;
      exp += 2;
    }
    if (exp < 0) {
      fval *= 0.1;
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    }
  }

  // Assign floating point value with correct sign.
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }

  *char_counter = ptr - line - 1; // Set char_counter to next statement

  return (true);
}

// Simple hypotenuse computation function.
float hypot_f(float x, float y) {
  return (sqrt(x * x + y * y));
}
