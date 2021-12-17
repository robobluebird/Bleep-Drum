// Created by Dr. Bleep
// Modified by Zach Schroeder

// Added tempo fine tuning + oled
// Removed noise mode and reverse mode, for size reasons

#include <ssd1306.h> // https://github.com/lexus2k/ssd1306
#include <avr/pgmspace.h>
#include "samples.h"
#include <SPI.h>

// Replace Bounce2 library with simple debounce code, for size and dependency reduction
int lastRedState = HIGH;
int lastBlueState = HIGH;
int lastGreenState = HIGH;
int lastYellowState = HIGH;
int lastTapState = HIGH;

unsigned long lastDebounceRed = 0;
unsigned long lastDebounceBlue = 0;
unsigned long lastDebounceGreen = 0;
unsigned long lastDebounceYellow = 0;
unsigned long lastDebounceTap = 0;

unsigned long debounceDelay = 2;

// Reroute two button pins below because OLED now used analog pins 4 and 5 (also called pins 18 and 19) for
// oled data and clock, respectively

// This code does not use an OLED reset pin because it is working with Adafruit oled which has internal reset circuitry

#define red_pin 17
#define blue_pin 1 // BLUE button now goes to digital pin 1
#define green_pin 2
#define yellow_pin 0 // YELLOW pin now routes to digital pin 0

#define play_pin 3
#define rec_pin 4
#define tap_pin 7
#define shift_pin 8

uint32_t cm, pm, erase_led;
const char noise_table[] PROGMEM = {};
const unsigned long dds_tune = 4294967296 / 9800; // 2^32/measured dds freq but this takes too long
byte shift, bankpg, bankpr, bout, rout, gout, prevpot2, erase_latch, potTempo;
byte banko = 0;
byte n1, n2;
int eee, tapHoldDuration;
byte bankpb = 4;
byte beat;
uint16_t pot1, pot2;
long pot3 = 200, pot4 = 200;
int kick_sample, snare_sample, sample, hat_sample, noise_sample, bass_sample, B2_freq_sample, B1_freq_sample;
uint16_t increment, increment2, increment3, increment4, increment5, increment2v, increment4v;
uint32_t accumulator, accumulator2, accumulator3, accumulator4, accumulator5, accu_freq_1, accu_freq_2;
int rando;
byte B2_sequence[129] = {};
byte B3_sequence[129] = {};
byte B1_sequence[129] = {};
byte B4_sequence[129] = {};
int B2_freq_sequence[129] = {};
int B1_freq_sequence[129] = {};
int sample_sum;
int j, k, freq3, cc;
int kf, pf, holdkf, kfe;
int shiftcount = 0;
int t1, c1, count1, dd;
byte noise_type;
uint16_t index, index2, index3, index4, index5, index4b, index_freq_1, index_freq_2, index4bv;
uint16_t indexr, index2r, index3r, index4r, index4br, index2vr, index4vr;
int osc, oscc;
byte ledstep;
unsigned long freq, freq2;
int wavepot, lfopot, arppot;
byte loopstep = 0;
byte loopstepf = 0;
byte recordbutton, prevrecordbutton, record, looptrigger, prevloopstep, preva, prevb;
int looprate;
long prev, prev2, prev3;
byte playmode = 1;
byte play = 0;
byte playbutton, pplaybutton;
byte pkbutton, kbutton, B4_trigger, B4_latch, cbutton, pcbutton, B4_loop_trigger, B1_trigger, kick, B1_latch, clap, B1_loop_trigger, B4_seq_trigger, B3_seq_trigger;
byte ptbutton, tbutton, ttriger, B1_seq_trigger, B3_latch, B2_trigger, bc, B2_loop_trigger, B3_loop_trigger;
byte B2_latch, B3_trigger, B2_seq_trigger, pbutton, ppbutton;
byte kicktriggerv, B2_seq_latch, kickseqtriggerv,  B1_seq_latch, pewseqtriggerv, precordbutton;
byte recordmode = 1;
unsigned long ratepot;
byte r, g, b, e, erase;
byte button1, button2, button3, button4, tapb;
byte pbutton1, pbutton2, pbutton3, pbutton4, ptapb;
byte bf1, bf2, bf3, bf4, bft;
uint16_t midicc3 = 128;
uint16_t midicc4 = 157;
uint16_t midicc1, midicc2, midicc5, midicc6, midicc7, midicc8;
byte midi_note_check;
byte prevshift, shift_latch;
byte tick;
byte t;
long tapbank[4];
byte  mnote, mvelocity, miditap, pmiditap, miditap2, midistep, pmidistep, miditempo, midinoise;
unsigned long recordoffsettimer, offsetamount, taptempof;
int click_pitch;
byte click_amp;
int click_wait;
int sample_out_temp;
byte sample_out;
uint32_t dds_time;
byte bft_latch;
unsigned long raw1, raw2;
unsigned long log1, log2;
byte click_play, click_en;
int shift_time;
int shift_time_latch;
byte printer = 0;
float trueTempo = 120.0; // "true" as in "the tempo in bpm"

// 60.0 / tempo = seconds per beat.
// seconds per beat / 8 = seconds per 32nd note
// seconds per 32nd note * sample rate = samples per 32nd note
uint16_t slice = (60.0 / trueTempo / 8.0) * 9813;

unsigned long taptempo = (unsigned long)slice; // cast "slice" to long because taptempo was originally a long (might not matter?)

void setup() {
  cli();

  pinMode (12, OUTPUT);
  pinMode (13, OUTPUT);
  pinMode (11, OUTPUT);
  pinMode (10, OUTPUT);
  pinMode (9, OUTPUT);
  pinMode (5, OUTPUT);
  pinMode (6, OUTPUT);
  pinMode (16, OUTPUT);

  pinMode (3, INPUT);     digitalWrite(3, HIGH);  //play
  pinMode (4, INPUT);     digitalWrite (4, HIGH); //rec
  pinMode (8, INPUT);     digitalWrite (8, HIGH); //tap
  pinMode (7, INPUT);     digitalWrite(7, HIGH);   //shift
  pinMode (12, OUTPUT);

  pinMode (green_pin, INPUT_PULLUP);   //low left clap green
  pinMode (yellow_pin, INPUT_PULLUP);   // low right kick yellow
  pinMode (blue_pin, INPUT_PULLUP);    //Up Right tom Blue
  pinMode (red_pin, INPUT_PULLUP);   // Up right pew red

  delay(100);

  //pinMode (16, INPUT); digitalWrite (16, HIGH);
  digitalWrite(16, HIGH); //
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  /* Enable interrupt on timer2 == 127, with clk/8 prescaler. At 16MHz,
     this gives a timer interrupt at 15625Hz. */

  TIMSK2 = (1 << OCIE2A);
  OCR2A = 50; // sets the compare. measured at 9813Hz

  TCCR2A = 1 << WGM21 | 0 << WGM20; /* CTC mode, reset on match */
  TCCR2B = 0 << CS22 | 1 << CS21 | 1 << CS20; /* clk, /8 prescaler */
  
  TCCR0B = B0000001;
  TCCR1B = B0000001;

  sei();

  // A lot of delays here. Not sure if they are totally needed but I couldn't get the oled
  // to display properly when on battery power without them *shrug*
  delay(5000);
  ssd1306_128x64_i2c_init();
  delay(5000);
  ssd1306_setFixedFont(ssd1306xled_font8x16);
  ssd1306_clearScreen();
  delay(5000);
}

int sine_sample;
byte index_sine;
uint32_t acc_sine;
uint32_t trig_out_time, trig_out_latch;
ISR(TIMER2_COMPA_vect) {
  dds_time++;

  OCR2A = 50;

  prevloopstep = loopstep;
  if (recordmode == 1 && miditempo == 0) {
    if (dds_time - prev > (taptempof) ) {
      prev = dds_time;
      loopstep++;
      if (loopstep > 31) {
        loopstep = 0;
      }
    }
  }

  if (shift == 1) {
    if (bft == 1) {
      bft = 0;
      bft_latch = 0;
      trueTempo += 0.05;
      slice = (60.0 / trueTempo / 8.0) * 9813;
      taptempo = (unsigned long)slice;
    }
  } else {
    if (potTempo == 0) { // if we've held shift + tap for long enough...
      if (bft == 1) {
        bft = 0;
        bft_latch = 0;
        trueTempo -= 0.05;
        slice = (60.0 / trueTempo / 8.0) * 9813;
        taptempo = (unsigned long)slice;
      }
    }
  }

  taptempof = taptempo;

  recordoffsettimer = dds_time - prev ;
  offsetamount = taptempof - (taptempof >> 2 );

  if ((recordoffsettimer) > (offsetamount)) {
    loopstepf = loopstep + 1;
    if (loopstepf > 31) {
      loopstepf = 0;
    }
  }

  if (loopstep % 4 == 0) {
    click_pitch = 220;
  }
  
  if (loopstep % 8 == 0) {
    click_pitch = 293;
  }

  if (loopstep == 0) {
    click_pitch = 440;
  }

  if (play == 1) {
    click_play = 1;
    B4_loop_trigger = B4_sequence[loopstep + banko];
    B1_loop_trigger = B1_sequence[loopstep + banko];
    B2_loop_trigger = B2_sequence[loopstep + banko];
    B3_loop_trigger = B3_sequence[loopstep + banko];
  }

  if (play == 0) {
    loopstep = 31;
    prev = 0;
    click_play = 0;
    B4_loop_trigger = 0;
    B1_loop_trigger = 0;
    B2_loop_trigger = 0;
    B3_loop_trigger = 0;
  }

  if (loopstep != prevloopstep) {
    digitalWrite(12, 1);
    trig_out_time = dds_time;
    trig_out_latch = 1;
  }

  if (dds_time - trig_out_time > 80 && trig_out_latch == 1) {
    trig_out_latch = 0;
    digitalWrite(12, 0);
  }

  if (loopstep != prevloopstep && B3_loop_trigger == 1) {
    B3_seq_trigger = 1;
  } else {
    B3_seq_trigger = 0;
  }

  if (loopstep != prevloopstep && B2_loop_trigger == 1) {
    B2_seq_trigger = 1;
  } else {
    B2_seq_trigger = 0;
  }

  if (loopstep != prevloopstep && B4_loop_trigger == 1) {
    B4_seq_trigger = 1;
  } else {
    B4_seq_trigger = 0;
  }

  if (loopstep != prevloopstep && B1_loop_trigger == 1) {
    B1_seq_trigger = 1;
  } else {
    B1_seq_trigger = 0;
  }

  if (B3_trigger == 1 || B3_seq_trigger == 1) {
    index3 = 0;
    accumulator3 = 0;
    B3_latch = 1;
  }

  if (B4_trigger == 1 || B4_seq_trigger == 1) {
    index4 = 0;
    accumulator4 = 0;
    B4_latch = 1;
  }

  if (B1_trigger == 1) {
    index = 0;
    accumulator = 0;
    B1_latch = 1;
  }

  if (B1_seq_trigger == 1) {
    index_freq_1 = 0;
    accu_freq_1 = 0;
    B1_seq_latch = 1;
  }

  if (B2_seq_trigger == 1) {
    index_freq_2 = 0;
    accu_freq_2 = 0;
    B2_seq_latch = 1;
  }

  if (B2_trigger == 1) {
    index2 = 0;
    accumulator2 = 0;
    B2_latch = 1;
  }

  if (loopstep % 4 == 0 && prevloopstep % 4 != 0) {
    click_amp = 64;
  }

  sine_sample = (((pgm_read_byte(&sine_table[index_sine]) - 127) * click_amp) >> 8) * click_play * click_en;

  if (playmode == 1) {
    snare_sample = (pgm_read_byte(&snare_table[(index3)])) - 127;
    kick_sample = (pgm_read_byte(&kick_table[(index4)])) - 127;
    hat_sample = (pgm_read_byte(&tick_table[(index)])) - 127;
    bass_sample = (((pgm_read_byte(&bass_table[(index2)])))) - 127;
    B1_freq_sample = pgm_read_byte(&tick_table[(index_freq_1)]) - 127;
    B2_freq_sample = (pgm_read_byte(&bass_table[(index_freq_2)])) - 127;
    noise_sample = (((pgm_read_byte(&sine_table[(index5)])))) - 127;
  }

  sample_out_temp = ((snare_sample + kick_sample + hat_sample + bass_sample + B1_freq_sample + B2_freq_sample + sine_sample) >> 1) + 127;

  if (sample_out_temp > 255) {
    sample_out_temp -= (sample_out_temp - 255) << 1; //fold don't clip!
  }

  if (sample_out_temp < 0) {
    sample_out_temp += sample_out_temp * -2;
  }

  sample_out = sample_out_temp;

  uint16_t dac_out = (0 << 15) | (1 << 14) | (1 << 13) | (1 << 12) | (sample_out << 4);
  digitalWrite(10, LOW);
  SPI.transfer(dac_out >> 8);
  SPI.transfer(dac_out & 255);
  digitalWrite(10, HIGH);

  acc_sine += click_pitch << 2 ;
  index_sine = (dds_tune * acc_sine) >> (32 - 8);
  click_wait++;

  if (click_wait > 4) {
    click_wait = 0;

    if (click_amp >= 4) {
      click_amp -= 1;
    }

    if (click_amp < 4) {
      click_amp = 0;
    }
  }

  if (B1_latch == 1) {
    if (midicc1 > 4) {
      accumulator += midicc1;
    } else {
      accumulator += pot1;
    }

    index = (accumulator >> (6));

    if (index > tick_length) {
      index = 0;
      accumulator = 0;
      B1_latch = 0;
    }
  }

  if (B2_latch == 1) {
    if (midicc2 > 4) {
      accumulator2 += midicc2;
    } else {
      accumulator2 += pot2;
    }

    index2 = (accumulator2 >> (6));

    if (index2 > bass_length) {
      index2 = 0;
      accumulator2 = 0;
      B2_latch = 0;
    }
  }

  if (B3_latch == 1) {
    accumulator3 += (midicc3);
    index3 = (accumulator3 >> 6);

    if (index3 > snare_length) {
      index3 = 0;
      accumulator3 = 0;
      B3_latch = 0;
    }
  }

  if (B4_latch == 1) {
    accumulator4 += (midicc4);
    index4 = (accumulator4 >> (6));

    if (index4 > kick_length) {
      index4 = 0;
      accumulator4 = 0;
      B4_latch = 0;
    }
  }

  accu_freq_1 += kf;
  index_freq_1 = (accu_freq_1 >> (6));
  if (B1_seq_trigger == 1) {
    kf = B1_freq_sequence[loopstep + banko];
    kfe = kf;
  }

  if (index_freq_1 > tick_length) {
    kf = 0;
    index_freq_1 = 0;
    accu_freq_1 = 0;
    B1_seq_latch = 0;
  }

  accu_freq_2 += pf;
  index_freq_2 = (accu_freq_2 >> (6));

  if (B2_seq_trigger == 1) {
    pf = B2_freq_sequence[loopstepf + banko];
  }
  
  if (index_freq_2 > bass_length) {
    pf = 0;
    index_freq_2 = 0;
    accu_freq_2 = 0;
    B2_seq_latch = 0;
  }
}

void loop() {
  cm = millis();

  midi_note_check = 0;

  pbutton1 = button1;
  pbutton2 = button2;
  pbutton3 = button3;
  pbutton4 = button4;

  int red = digitalRead(red_pin);
  int blue = digitalRead(blue_pin);
  int green = digitalRead(green_pin);
  int yellow = digitalRead(yellow_pin);
  int tappy = digitalRead(tap_pin);

  if (red != lastRedState) lastDebounceRed = 0;
  if (blue != lastBlueState) lastDebounceBlue = 0;
  if (green != lastGreenState) lastDebounceGreen = 0;
  if (yellow != lastYellowState) lastDebounceYellow = 0;
  if (tapb != lastTapState) lastDebounceTap = 0;

  if (cm - lastDebounceRed > debounceDelay) button1 = red;
  if (cm - lastDebounceBlue > debounceDelay) button2 = blue;
  if (cm - lastDebounceGreen > debounceDelay) button3 = green;
  if (cm - lastDebounceYellow > debounceDelay) button4 = yellow;
  if (cm - lastDebounceTap > debounceDelay) tapb = tappy;

  if (button1 == 0 && pbutton1 == 1) {
    bf1 = 1;
  } else {
    bf1 = 0;
  }

  if (button2 == 0 && pbutton2 == 1) {
    bf2 = 1;
  } else {
    bf2 = 0;
  }

  if (button3 == 0 && pbutton3 == 1) {
    bf3 = 1;
  } else {
    bf3 = 0;
  }

  if (button4 == 0 && pbutton4 == 1) {
    bf4 = 1;
  } else {
    bf4 = 0;
  }

  if (tapb == 0 && ptapb == 1 && bft_latch == 0) {
    bft = 1;
    bft_latch = 1;
  } else {
    bft = 0;
    bft_latch = 0;
  }

  if (midi_note_check == 67) {
    play++;
    play %= 2;
  }

  if (midi_note_check == 69) {
    playmode++;
    playmode %= 2;
  }

  if (midi_note_check == 70) {
    midinoise = 1;
    shift_latch = 1;
  }

  if (midi_note_check == 72) {
    banko = 0; //blue
  }
  if (midi_note_check == 74) {
    banko = 31; // yellow
  }
  if (midi_note_check == 76) {
    banko = 63; //red
  }
  if (midi_note_check == 77) {
    banko = 95; //green
  }

  pmiditap = miditap;
  pmidistep = midistep;

  LEDS();
  BUTTONS();
  RECORD();

  ptapb = tapb;

  raw1 = analogRead(0);
  log1 = raw1 * raw1;
  raw2 = analogRead(1);
  log2 = raw2 * raw2;

  pot1 = (log1 >> 11) + 2; //simple way of getting an exonential range from the linear pot
  pot2 = (log2 >> 11) + 42;

  char buf[8];
  dtostrf(trueTempo, 6, 2, buf); // arduino doesn't play nice with %f for sprintf, use this function instead
  ssd1306_printFixed(0,  0, buf, STYLE_NORMAL); // print the tempo to the oled

  lastRedState = red;
  lastBlueState = blue;
  lastGreenState = green;
  lastYellowState = yellow;
  lastTapState = tappy;
}

void RECORD() {
  pplaybutton = playbutton;
  playbutton = digitalRead(play_pin);

  if (pplaybutton == 1 && playbutton == 0 && shift == 1 && recordbutton == 1 ) {
    play = !play;
  }

  precordbutton = recordbutton;
  recordbutton = digitalRead(rec_pin);
  if (recordbutton == 0 && precordbutton == 1) {
    record = !record;
    play = 1;
    erase_latch = 1;
    eee = 0;
  }
  
  if (recordbutton == 1 && precordbutton == 0) {
    erase_latch = 0;
  }

  if (play == 0) {
    record = 0;
  }

  if (recordbutton == 0) {
    if (playbutton == 0 &&  erase_latch == 1) {
      eee++;
      if (eee >= 200) {
        eee = 0;
        erase_latch = 0;
        erase = 1;
        erase_led = millis();
        play = 1;
        record = 0;
        for (byte j; j < 32; j++) {
          B1_sequence[j + banko] = 0;
          B2_sequence[j + banko] = 0;
          B4_sequence[j + banko] = 0;
          B3_sequence[j + banko] = 0;
        }
      }
    }
  }

  if (millis() - erase_led > 5000) {
    erase = 0;
  }

  if (record == 1) {
    if (B1_trigger == 1) {
      B1_sequence[loopstepf + banko] = 1;
      B1_freq_sequence[loopstepf + banko] = pot1;
    }

    if (B2_trigger == 1) {
      B2_sequence[loopstepf + banko] = 1;
      B2_freq_sequence[loopstepf + banko] = (pot2);
    }

    if (B4_trigger == 1) {
      B4_sequence[loopstepf + banko] = 1;
    }

    if (B3_trigger == 1) {
      B3_sequence[loopstepf + banko] = 1;
    }
  }
}

void LEDS() {
  int binv = 255 - bout;
  int ginv = 255 - gout;
  int rinv = 255 - rout;
  
  analogWrite(9, binv);
  analogWrite(6, ginv);
  analogWrite(5, rinv);

  if (record == 0 && play == 0) {
    rout = 16;
    gout = 16;
    bout = 16;
  }

  if (play == 1 && record == 0) {
    bout = b * !erase;
    rout = r * !erase;
    gout = g * !erase;

    if (loopstep == 0) {
      r = 12;
      g = 15;
      b = 12;
    } else if (loopstep % 4 == 0) {
      r = 8;
      g = 10;
      b = 10;
    } else {
      b = bankpb;
      r = bankpr;
      g = bankpg;
    }
  }

  if (play == 1 && record == 1) {
    bout = b;
    rout = r;
    gout = g;

    if ( loopstep == 0 ) {
      r = 30;
      g = 6;
      b = 6;
    } else if ( loopstep % 4 == 0) {
      r = 20;
      g = 2;
      b = 2;
    } else {
      b = bankpb;
      r = bankpr;
      g = bankpg;
    }
  }
}

void BUTTONS() {
  prevshift = shift;

  shift = digitalRead(shift_pin);

  if (shift == 0 && prevshift == 1) {
    shift_latch++;
    shift_latch %= 2;
    shift_time = 0;
    shift_time_latch = 1;
  }

  if (shift == 0 && tapb == 1 && shift_time_latch == 1) {
    shift_time++;
    if (shift_time > 200) {
      click_en = !click_en;
      shift_time = 0;
      shift_time_latch = 0;
    }
  }

  if (shift == 0 && recordbutton == 1) {
    prevpot2 = pot2;

    if (button1 == 0 ) { // red
      banko = 63;
    }

    if (button4 == 0) { // yellow
      banko = 31;
    }

    if (button2 == 0 || banko == 0) { // blue
      banko = 0;
    }

    if (button3 == 0) { // green
      banko = 95;
    }

    if (tapb == 0) {

      // use a counter to delay going into "pot controlled tempo" mode, so that shift + tap can also
      // be used for "tempo -= 0.5" functionality
      tapHoldDuration++;

      if (tapHoldDuration > 100) {
        potTempo = 1;
        ratepot = analogRead(1);

        // analogRead gets a value between 0 and 1023. Map it to 0-800 range because we want pot control
        // to move by 0.20 increments. E.G. tempo range = 40-200 = 160 whole steps * 5 steps (0.20) per whole step
        trueTempo = 40.0 + (0.2 * map(ratepot, 0, 1023, 0, 800));
        slice = (60.0 / trueTempo / 8.0) * 9813; // this calculation is explained around line 107
        taptempo = (unsigned long)slice;
      }
    } else {
      tapHoldDuration = 0;
      potTempo = 0;
    }
  } else {
    tapHoldDuration = 0;
    potTempo = 0;
  }

  if (banko == 63) { //red
    bankpr = 5;
    bankpg = 0;
    bankpb = 0;
  }

  if (banko == 31) { //green
    bankpr = 0;
    bankpg = 8;
    bankpb = 0;
  }

  if (banko == 0) { //blue
    bankpr = 0;
    bankpg = 0;
    bankpb = 9;
  }

  if ( banko == 95) { // yellow
    bankpr = 6;
    bankpg = 8;
    bankpb = 0;
  }

  if (shift == 1) {
    if (bf1 == 1 || midi_note_check == 60) {
      B1_trigger = 1;
    } else {
      B1_trigger = 0;
    }
    
    if (bf4 == 1 || midi_note_check == 65) {
      B4_trigger = 1;
    } else {
      B4_trigger = 0;
    }
    
    if (bf2 == 1 || midi_note_check == 62) {
      B2_trigger = 1;
    } else {
      B2_trigger = 0;
    }
    
    if (bf3 == 1 || midi_note_check == 64) {
      B3_trigger = 1;
    } else {
      B3_trigger = 0;
    }

  }
}
