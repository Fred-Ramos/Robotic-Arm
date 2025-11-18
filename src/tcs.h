#ifndef TCS_H
#define TCS_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

//COLOR SENSOR VARIABLES
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
double r_perc, g_perc, b_perc;

//COLOR SENSOR FUNCTIONS
void setup_TCS34725(){                     
  while (!tcs.begin(TCS34725_ADDRESS, &Wire1)) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }

  Serial.println("Found sensor");
}


String tcs_getcolor(){ 
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  r_perc = (double)r;
  g_perc = (double)g;
  b_perc = (double)b;
  double scale_factor = 1/(max(r_perc, max(g_perc, b_perc)));
  r_perc = r*scale_factor;
  g_perc = g*scale_factor;
  b_perc = b*scale_factor;
  double this_color[3] = {r_perc, g_perc, b_perc};
  String colorNames[4] = {"r", "g", "b", "y"};
  double defined_color[4][3] = {
    {1, 0, 0}, // Red
    {0, 1, 0}, // Green
    {0, 0, 1}, // Blue
    {1, 1, 0} // Yellow
  };

  double dif[4] = {0, 0, 0, 0};
  for (int k = 0; k < 4; k++){ //difference to each defined color
    double r1 = pow(this_color[0] - defined_color[k][0], 2);
    double g1 = pow(this_color[1] - defined_color[k][1], 2);
    double b1 = pow(this_color[2] - defined_color[k][2], 2);
    dif[k] = sqrt(r1 + g1 + b1); //sqrt (diference(r)^2 + diference(g)^2 + diference(b)^2)
  }

  double min_dif = dif[0];
  int color_index = 0;
 for (int i = 0; i < 4; i++) {
    if (dif[i] < min_dif) {
      min_dif = dif[i];
      color_index = i;
    }
 }

 return colorNames[color_index];
}

#endif