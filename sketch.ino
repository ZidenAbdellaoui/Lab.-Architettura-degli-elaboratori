#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define DEBOUNCE_TIME_MS         50
#define MEASUREMENT_INTERVAL_MS  200
#define ECHO_TIMEOUT_MS          25
#define TWO_MINUTE_TIMEOUT_MS    120000UL
#define MAX_DISTANCE_CM          400.0f
#define BAR_GRAPH_MAX_DISPLAY_CM 200.0f
#define DANGER_DISTANCE_CM       5.0f
#define FLASH_INTERVAL_MS        300
#define LCD_CONTRAST             0xBF

#define DANGER_MODE_SOLID_FLASH   0
#define DANGER_MODE_CHECKERBOARD  1
#define DANGER_MODE_INVERSE       2
#define DANGER_MODE_ANIMATION     3
#define NUM_DANGER_MODES          4

// LCD: RST=PD4, CE=PB2, DC=PD7, DIN=PB3 (MOSI), CLK=PB5 (SCK)
#define LCD_RST_PORT   PORTD
#define LCD_RST_DDR    DDRD
#define LCD_RST_PIN    PD4

#define LCD_CE_PORT    PORTB
#define LCD_CE_DDR     DDRB
#define LCD_CE_PIN     PB2

#define LCD_DC_PORT    PORTD
#define LCD_DC_DDR     DDRD
#define LCD_DC_PIN     PD7

#define LCD_DIN_PORT   PORTB
#define LCD_DIN_DDR    DDRB
#define LCD_DIN_PIN    PB3

#define LCD_CLK_PORT   PORTB
#define LCD_CLK_DDR    DDRB
#define LCD_CLK_PIN    PB5

#define TRIG_DDR       DDRB
#define TRIG_PORT      PORTB
#define TRIG_PIN       PB1

#define ECHO_DDR       DDRB
#define ECHO_PORT      PORTB
#define ECHO_PIN_REG   PINB
#define ECHO_PIN       PB0

// Button: PD2 (INT0)
#define BUTTON_DDR     DDRD
#define BUTTON_PORT    PORTD
#define BUTTON_PIN     PD2

// --- Global Variables ---
volatile bool wakeUpFlag = false;
volatile uint32_t g_millis = 0;
volatile unsigned long last_button_press_time = 0;

// Distance measurement
volatile uint16_t echo_start_time_ticks = 0;
volatile uint16_t echo_pulse_duration_ticks = 0;
volatile uint8_t echo_state = 0;
volatile uint32_t measurement_start_time_ms = 0;

// System state
uint32_t last_measurement_trigger_time = 0;
uint32_t time_continuously_over_2m_started_at = 0;
bool active_session_initialized = false;

bool triangle_flashing_active = false;
bool triangle_visible = false;
uint32_t last_flash_time = 0;
uint8_t danger_display_mode = DANGER_MODE_SOLID_FLASH;
uint32_t last_mode_change_time = 0;
uint8_t animation_frame = 0;

// Font and graphics
const unsigned char PROGMEM font[][5] = {
 {0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20 ' ' (space)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 0x21 '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 0x22 '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 0x23 '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 0x24 '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 0x25 '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 0x26 '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 0x27 '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 0x28 '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 0x29 ')'
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // 0x2A '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 0x2B '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 0x2C ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 0x2D '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 0x2E '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 0x2F '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0x30 '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 0x31 '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 0x32 '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 0x33 '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 0x34 '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 0x35 '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 0x36 '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 0x37 '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 0x38 '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 0x39 '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 0x3A ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 0x3B ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 0x3C '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 0x3D '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 0x3E '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 0x3F '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 0x40 '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 0x41 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 0x42 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 0x43 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 0x44 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 0x45 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 0x46 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 0x47 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 0x48 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 0x49 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 0x4A 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 0x4B 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 0x4C 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 0x4D 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 0x4E 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 0x4F 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 0x50 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 0x51 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 0x52 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 0x53 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 0x54 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 0x55 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 0x56 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 0x57 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 0x58 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 0x59 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 0x5A 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 0x5B '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 0x5C '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 0x5D ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 0x5E '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 0x5F '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 0x60 '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 0x61 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 0x62 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 0x63 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 0x64 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 0x65 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 0x66 'f'
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 0x67 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 0x68 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 0x69 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 0x6A 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 0x6B 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 0x6C 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 0x6D 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 0x6E 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 0x6F 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 0x70 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 0x71 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 0x72 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 0x73 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 0x74 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 0x75 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 0x76 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 0x77 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 0x78 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 0x79 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 0x7A 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 0x7B '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 0x7C '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 0x7D '}'
    {0x10, 0x08, 0x08, 0x10, 0x08}, // 0x7E '~'
};

const uint8_t PROGMEM checkerboard_pattern[8] = {
    0b10101010, 0b01010101, 0b10101010, 0b01010101,
    0b10101010, 0b01010101, 0b10101010, 0b01010101
};

const uint8_t PROGMEM animation_frames[4][8] = {
    {0b00011000, 0b00111100, 0b01111110, 0b11111111,
     0b11111111, 0b01111110, 0b00111100, 0b00011000}, // Expanding circle
    {0b10000001, 0b01000010, 0b00100100, 0b00011000,
     0b00011000, 0b00100100, 0b01000010, 0b10000001}, // X pattern
    {0b11111111, 0b10000001, 0b10000001, 0b10000001,
     0b10000001, 0b10000001, 0b10000001, 0b11111111}, // Square frame
    {0b11000011, 0b11100111, 0b01111110, 0b00111100,
     0b00111100, 0b01111110, 0b11100111, 0b11000011}  // Diamond
};

// --- Timer0 for millis() ---
void TIMER0_Init() {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    g_millis++;
}

uint32_t millis() {
    uint32_t m;
    cli();
    m = g_millis;
    sei();
    return m;
}

// --- SPI and LCD Functions ---
void SPI_Enable() {
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);
}

void SPI_Disable() {
    SPCR = 0;
}

void LCD_Send(uint8_t data, bool isCommand) {
    LCD_CE_PORT &= ~(1 << LCD_CE_PIN);
    if (isCommand) {
        LCD_DC_PORT &= ~(1 << LCD_DC_PIN);
    } else {
        LCD_DC_PORT |= (1 << LCD_DC_PIN);
    }
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    LCD_CE_PORT |= (1 << LCD_CE_PIN);
}

void LCD_Init() {
    LCD_RST_DDR |= (1 << LCD_RST_PIN);
    LCD_DC_DDR |= (1 << LCD_DC_PIN);
    LCD_CE_DDR |= (1 << LCD_CE_PIN);
    
    LCD_RST_PORT |= (1 << LCD_RST_PIN);
    _delay_ms(10);
    LCD_RST_PORT &= ~(1 << LCD_RST_PIN);
    _delay_ms(100);
    LCD_RST_PORT |= (1 << LCD_RST_PIN);
    _delay_ms(100);

    SPI_Enable();

    LCD_Send(0x21, true);
    LCD_Send(LCD_CONTRAST, true);
    LCD_Send(0x04, true);
    LCD_Send(0x14, true);
    LCD_Send(0x20, true);
    LCD_Send(0x0C, true);
    LCD_Clear();
}

void LCD_PowerDown() {
    LCD_Send(0x20, true);
    LCD_Send(0x08, true);
    _delay_ms(10);
    SPI_Disable();
    LCD_DC_DDR &= ~(1 << LCD_DC_PIN);
    LCD_CE_DDR &= ~(1 << LCD_CE_PIN);
    LCD_RST_DDR &= ~(1 << LCD_RST_PIN);
}

void LCD_WakeUp() {
    LCD_DC_DDR |= (1 << LCD_DC_PIN);
    LCD_CE_DDR |= (1 << LCD_CE_PIN);
    LCD_RST_DDR |= (1 << LCD_RST_PIN);
    
    LCD_RST_PORT |= (1 << LCD_RST_PIN);
    _delay_ms(1);
    LCD_RST_PORT &= ~(1 << LCD_RST_PIN);
    _delay_ms(10);
    LCD_RST_PORT |= (1 << LCD_RST_PIN);
    _delay_ms(100);
    
    SPI_Enable();
    LCD_Init();
}

void LCD_SetPosition(uint8_t x, uint8_t y_page) {
    x = (x > 83) ? 83 : x;
    y_page = (y_page > 5) ? 5 : y_page;
    LCD_Send(0x80 | x, true);
    LCD_Send(0x40 | y_page, true);
}

void LCD_Clear() {
    LCD_SetPosition(0, 0);
    for (uint16_t i = 0; i < (84 * 6); i++) {
        LCD_Send(0x00, false);
    }
}

void LCD_PrintString(const char* str, uint8_t x, uint8_t y_page) {
    LCD_SetPosition(x, y_page);
    while (*str) {
        char c = *str++;
        if (c < 0x20 || c > 0x7E) c = 0x20;
        uint8_t char_index = c - 0x20;
        for (uint8_t i = 0; i < 5; i++) {
            LCD_Send(pgm_read_byte(&font[char_index][i]), false);
        }
        LCD_Send(0x00, false);
        x += 6;
        if (x > 78) { x = 0; y_page++; LCD_SetPosition(x, y_page); }
    }
}

void LCD_DrawBarGraph(float current_distance_cm, uint8_t x_start, uint8_t y_page,
                     uint8_t graph_width_pixels, uint8_t bar_height_mask,
                     float max_graph_distance_cm) {
    uint8_t filled_length = (current_distance_cm <= 0) ? 0 :
                           (current_distance_cm >= max_graph_distance_cm) ? graph_width_pixels :
                           (uint8_t)((current_distance_cm / max_graph_distance_cm) * graph_width_pixels);

    LCD_SetPosition(x_start, y_page);
    for (uint8_t i = 0; i < graph_width_pixels; i++) {
        LCD_Send((i < filled_length) ? bar_height_mask : 0x00, false);
    }
}

void display_danger_pattern() {
  
    LCD_SetPosition(0, 0);
            for (uint8_t page = 0; page < 6; page++) {
                LCD_SetPosition(0, page);
                for (uint8_t col = 0; col < 84; col++) {
                    uint8_t pattern = pgm_read_byte(&animation_frames[animation_frame][col % 8]);
                    LCD_Send(pattern, false);
                }
            }
           _delay_ms(500);
            LCD_Init();

}

void update_danger_display(float current_distance) {
    if (millis() - last_mode_change_time > 3000) {
        last_mode_change_time = millis();
        danger_display_mode = (danger_display_mode + 1) % NUM_DANGER_MODES;
        animation_frame = 0;
    }
    
    if (millis() - last_flash_time > FLASH_INTERVAL_MS) {
        last_flash_time = millis();
        triangle_visible = !triangle_visible;
        
        if (triangle_visible) {
            display_danger_pattern();
        } else {
            LCD_Clear();
            LCD_Init();
            char dist_str[12];
            dtostrf(current_distance, 4, 1, dist_str);
            LCD_PrintString("DANGER!", 0, 0);
            LCD_PrintString(dist_str, 0, 1);
            LCD_PrintString("cm", 42, 1);
            LCD_DrawBarGraph(current_distance, 0, 3, 83, 0xFF, BAR_GRAPH_MAX_DISPLAY_CM);
        }
    }
}

// --- Distance Measurement ---
void TIMER1_Init_For_Echo() {
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TCNT1 = 0;
}

ISR(PCINT0_vect) {
    if (echo_state == 1 && (ECHO_PIN_REG & (1 << ECHO_PIN))) {
        echo_start_time_ticks = TCNT1;
        echo_state = 2;
    } 
    else if (echo_state == 2 && !(ECHO_PIN_REG & (1 << ECHO_PIN))) {
        uint16_t end_time = TCNT1;
        if (end_time >= echo_start_time_ticks) {
            echo_pulse_duration_ticks = end_time - echo_start_time_ticks;
        } else {
            echo_pulse_duration_ticks = (0xFFFF - echo_start_time_ticks) + end_time;
        }
        echo_state = 3;
        PCICR &= ~(1 << PCIE0);
    }
}

void trigger_distance_measurement() {
    if (echo_state != 0 && echo_state != 3 && echo_state != 4) return;

    echo_state = 1;
    echo_pulse_duration_ticks = 0;
    measurement_start_time_ms = millis();

    TIMER1_Init_For_Echo();
    PCMSK0 |= (1 << PCINT0);
    PCICR |= (1 << PCIE0);

    TRIG_PORT &= ~(1 << TRIG_PIN);
    _delay_us(2);
    TRIG_PORT |= (1 << TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);
}

float get_distance_cm() {
    if (echo_state == 3) {
        echo_state = 0;
        return (echo_pulse_duration_ticks * 0.5f * 0.0343f) / 2.0f;
    }
    else if (echo_state == 1 || echo_state == 2) {
        if (millis() - measurement_start_time_ms > ECHO_TIMEOUT_MS) {
            echo_state = 4;
            PCICR &= ~(1 << PCIE0);
        }
        return -2.0f;
    }
    else if (echo_state == 4) {
        echo_state = 0;
        return -1.0f;
    }
    return -3.0f;
}

ISR(INT0_vect) {
    unsigned long current_time = millis();
    if (current_time - last_button_press_time > DEBOUNCE_TIME_MS) {
        wakeUpFlag = true;
        active_session_initialized = false;
        last_button_press_time = current_time;
    }
    EIFR |= (1 << INTF0);
}
void LCD_ClearArea(uint8_t x, uint8_t y_page, uint8_t width, uint8_t height_pages) {
    for (uint8_t p = 0; p < height_pages; p++) {
        LCD_SetPosition(x, y_page + p);
        for (uint8_t i = 0; i < width; i++) {
            LCD_Send(0x00, false);
        }
    }
}
void goToSleep() {
    LCD_Clear();
    LCD_PrintString("Sleeping...", 0, 0);
    _delay_ms(500);
    LCD_PowerDown();

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
    
    LCD_WakeUp();
}

//Main
int main(void) {
    BUTTON_DDR &= ~(1 << BUTTON_PIN);
    BUTTON_PORT |= (1 << BUTTON_PIN);
    
    TRIG_DDR |= (1 << TRIG_PIN);
    TRIG_PORT &= ~(1 << TRIG_PIN);
    
    ECHO_DDR &= ~(1 << ECHO_PIN);
    ECHO_PORT &= ~(1 << ECHO_PIN);

    TIMER0_Init();
    LCD_Init();
    
    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0);
    
    sei();

    LCD_Clear();
    LCD_PrintString("Press Button", 0, 0);
    LCD_PrintString("To Start", 0, 1);
    
    float current_distance = -3.0f;

    while (1) {
        if (wakeUpFlag) {
            if (!active_session_initialized) {
                LCD_WakeUp();
                LCD_Clear();
                time_continuously_over_2m_started_at = 0;
                last_measurement_trigger_time = millis();
                trigger_distance_measurement();
                active_session_initialized = true;
                triangle_flashing_active = false;
                danger_display_mode = DANGER_MODE_SOLID_FLASH;
            }

            if (echo_state == 0 && (millis() - last_measurement_trigger_time > MEASUREMENT_INTERVAL_MS)) {
                last_measurement_trigger_time = millis();
                trigger_distance_measurement();
            }
            current_distance = get_distance_cm();

            if (current_distance >= -1.0f) {
                LCD_ClearArea(0, 0, 84, 2);
                
                if (current_distance == -1.0f) {
                    LCD_PrintString("No Echo", 0, 0);
                    triangle_flashing_active = false;
                } 
                else if (current_distance < DANGER_DISTANCE_CM) {
                    triangle_flashing_active = true;
                    update_danger_display(current_distance);
                } 
                else {
                    LCD_Init();
                    triangle_flashing_active = false;
                    char dist_str[12];
                    dtostrf(current_distance, 4, 1, dist_str);
                    LCD_PrintString("Dist: ", 0, 0);
                    LCD_PrintString(dist_str, 36, 0);
                    LCD_PrintString("cm", 66, 0);
                    if(current_distance>=100){
                    LCD_PrintString(" ", 0, 0);
                    }
                    else{
                    LCD_PrintString("  ", 0, 0);
                    }
                    LCD_DrawBarGraph(current_distance, 0, 3, 83, 0xFF, BAR_GRAPH_MAX_DISPLAY_CM);
                _delay_ms(500);
                LCD_Init();
                }
            }

            if (!triangle_flashing_active && triangle_visible) {
                LCD_ClearArea(0, 0, 84, 6);
                triangle_visible = false;
            }
            if (current_distance >= 0 || current_distance == -1.0f) {
                if (current_distance > 200.0f || current_distance == -1.0f) {
                    if (time_continuously_over_2m_started_at == 0) {
                        time_continuously_over_2m_started_at = millis();
                    } 
                    else if (millis() - time_continuously_over_2m_started_at > TWO_MINUTE_TIMEOUT_MS) {
                        wakeUpFlag = false;
                    }
                } 
                else {
                    time_continuously_over_2m_started_at = 0;
                }
            }
        } 
        else {
            goToSleep();
        }
    }
    return 0;
}