#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include "display_input.pio.h"

// ----------------------------------------------------------------------------

// Forward declarations
static void on_pio_irq(void);
static void dio1_irq_cb(uint gpio, uint32_t events);

// SPI Defines
#define SPI_PORT spi0
#define PIN_CHIP_SELECT   17    // Chip select
#define PIN_CLOCK         18    // Serial clock
#define PIN_MASTER_OUT    19    // Master out, slave in (MOSI)
#define PIN_COMMAND       20    // Data/command select
#define PIN_RESET         21    // Reset

// TI-86 display bus input pins
#define DISPLAY_INPUT_PIN_D1      2    // Digital input bit 1 of 4
#define DISPLAY_INPUT_PIN_D2      3    // Digital input bit 2 of 4
#define DISPLAY_INPUT_PIN_D3      4    // Digital input bit 3 of 4
#define DISPLAY_INPUT_PIN_D4      5    // Digital input bit 4 of 4
#define DISPLAY_INPUT_PIN_SCP     6    // Shift clock pulse - load four more bits on LOW
#define DISPLAY_INPUT_PIN_LP      7    // Latch pulse - signals a new line by going HIGH
#define DISPLAY_INPUT_PIN_DIO1    8    // DIO1 - signals a new frame by going HIGH

// Display defines
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define WORDS_PER_LINE (DISPLAY_WIDTH / 32)

// SSD1309 chip commands
#define SSD1309_CMD_LOWER_COLUMN_START_ADDRESS              0x00        /**< command lower column start address */
#define SSD1309_CMD_HIGHER_COLUMN_START_ADDRESS             0x10        /**< command higher column start address */
#define SSD1309_CMD_MEMORY_ADDRESSING_MODE                  0x20        /**< command memory addressing mode */
#define SSD1309_CMD_SET_COLUMN_ADDRESS                      0x21        /**< command set column address */
#define SSD1309_CMD_SET_PAGE_ADDRESS                        0x22        /**< command set page address */
#define SSD1309_CMD_RIGHT_HORIZONTAL_SCROLL                 0x26        /**< command right horizontal scroll */
#define SSD1309_CMD_LEFT_HORIZONTAL_SCROLL                  0x27        /**< command left horizontal scroll */
#define SSD1309_CMD_VERTICAL_RIGHT_HORIZONTAL_SCROLL        0x29        /**< command vertical right horizontal scroll */
#define SSD1309_CMD_VERTICAL_LEFT_HORIZONTAL_SCROLL         0x2A        /**< command vertical left horizontal scroll */
#define SSD1309_CMD_RIGHT_HORIZONTAL_SCROLL_ONE_COL         0x2C        /**< command right horizontal scroll by one column */
#define SSD1309_CMD_LEFT_HORIZONTAL_SCROLL_ONE_COL          0x2D        /**< command left horizontal scroll by one column */
#define SSD1309_CMD_DEACTIVATE_SCROLL                       0x2E        /**< command deactivate scroll */
#define SSD1309_CMD_ACTIVATE_SCROLL                         0x2F        /**< command activate scroll */
#define SSD1309_CMD_DISPLAY_START_LINE                      0x40        /**< command display start line */
#define SSD1309_CMD_CONTRAST_CONTROL                        0x81        /**< command contrast control */
#define SSD1309_CMD_COLUMN_0_MAPPED_TO_SEG0                 0xA0        /**< command column 0 mapped to seg 0 */
#define SSD1309_CMD_COLUMN_127_MAPPED_TO_SEG0               0xA1        /**< command column 127 mapped to seg 0 */
#define SSD1309_CMD_VERTICAL_SCROLL_AREA                    0xA3        /**< command vertical scroll area */
#define SSD1309_CMD_ENTIRE_DISPLAY_OFF                      0xA4        /**< command entire display off */ 
#define SSD1309_CMD_ENTIRE_DISPLAY_ON                       0xA5        /**< command entire display on */ 
#define SSD1309_CMD_NORMAL_DISPLAY                          0xA6        /**< command normal display */ 
#define SSD1309_CMD_INVERSE_DISPLAY                         0xA7        /**< command inverse display */ 
#define SSD1309_CMD_MULTIPLEX_RATIO                         0xA8        /**< command multiplex ratio */ 
#define SSD1309_CMD_DISPLAY_OFF                             0xAE        /**< command display off */ 
#define SSD1309_CMD_DISPLAY_ON                              0xAF        /**< command display on */ 
#define SSD1309_CMD_PAGE_ADDR                               0xB0        /**< command page address */ 
#define SSD1309_CMD_SCAN_DIRECTION_COM0_START               0xC0        /**< command scan direction com 0 start */ 
#define SSD1309_CMD_SCAN_DIRECTION_COMN_1_START             0xC8        /**< command scan direction com n-1 start */ 
#define SSD1309_CMD_DISPLAY_OFFSET                          0xD3        /**< command display offset */ 
#define SSD1309_CMD_DISPLAY_CLOCK_DIVIDE                    0xD5        /**< command display clock divide */ 
#define SSD1309_CMD_PRE_CHARGE_PERIOD                       0xD9        /**< command pre charge period */ 
#define SSD1309_CMD_COM_PINS_CONF                           0xDA        /**< command com pins conf */ 
#define SSD1309_CMD_COMH_DESLECT_LEVEL                      0xDB        /**< command comh deslect level */ 
#define SSD1309_CMD_GPIO                                    0xDC        /**< command set gpio */ 
#define SSD1309_CMD_LOCK                                    0xFD        /**< command set lock */

// ----------------------------------------------------------------------------

// Display data input - PIO setup
// Also setup the DMA pieces here for the PIO to read out the data
static PIO pio = pio0;
static uint sm;
static int dma_ch;

// Variables to hold and manage the incoming screen data
static uint32_t frame_bits[DISPLAY_HEIGHT][WORDS_PER_LINE];
static volatile uint current_line = 0;

// Output buffer where we store data that will be sent to the display
static bool output_buffer[DISPLAY_HEIGHT][DISPLAY_WIDTH] = { false };

// ----------------------------------------------------------------------------
// Hardware Initialization

// Configure the pins that we use to read digital signals from the TI-86
void initialize_input_pins ()
{
    // Initialize the digital input bit pins
    gpio_init(DISPLAY_INPUT_PIN_D1);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_D1);
    gpio_set_dir(DISPLAY_INPUT_PIN_D1, GPIO_IN);

    gpio_init(DISPLAY_INPUT_PIN_D2);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_D2);
    gpio_set_dir(DISPLAY_INPUT_PIN_D2, GPIO_IN);   

    gpio_init(DISPLAY_INPUT_PIN_D3);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_D3);
    gpio_set_dir(DISPLAY_INPUT_PIN_D3, GPIO_IN);

    gpio_init(DISPLAY_INPUT_PIN_D4);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_D4);
    gpio_set_dir(DISPLAY_INPUT_PIN_D4, GPIO_IN);

    // Initialize the bus signaling pins
    gpio_init(DISPLAY_INPUT_PIN_SCP);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_SCP);
    gpio_set_dir(DISPLAY_INPUT_PIN_SCP, GPIO_IN);

    gpio_init(DISPLAY_INPUT_PIN_LP);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_LP);
    gpio_set_dir(DISPLAY_INPUT_PIN_LP, GPIO_IN);   

    gpio_init(DISPLAY_INPUT_PIN_DIO1);
    gpio_disable_pulls(DISPLAY_INPUT_PIN_DIO1);
    gpio_set_dir(DISPLAY_INPUT_PIN_DIO1, GPIO_IN);
}

// Initialize the SPI bus so we can send data out to the new display
// Different SPI implementations may require modifications to a different screen
void initialize_spi ()
{
    // Reset pin is pulled low to reset, so we'll initialise it to a driven-high state
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 1);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CHIP_SELECT);
    gpio_set_dir(PIN_CHIP_SELECT, GPIO_OUT);
    gpio_put(PIN_CHIP_SELECT, 1);
    
    // Data/Command select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_COMMAND);
    gpio_set_dir(PIN_COMMAND, GPIO_OUT);
    gpio_put(PIN_COMMAND, 1);

    // Initialize the clock pin
    gpio_init(PIN_CLOCK);
    gpio_set_dir(PIN_CLOCK, GPIO_OUT);
    gpio_put(PIN_CLOCK, 1);

    // Initialize the data out pin
    gpio_init(PIN_MASTER_OUT);
    gpio_set_dir(PIN_MASTER_OUT, GPIO_OUT);
    gpio_put(PIN_MASTER_OUT, 0);

    // Initialize the SPI bus
    // Don't include the Chip Select pin, since we will toggle that ourselves
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_CLOCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MASTER_OUT, GPIO_FUNC_SPI);

    // Configure SPI format
    spi_set_format(
        spi0,
        8,      // Bits per transfer
        SPI_CPOL_0,
        SPI_CPHA_0,
        SPI_MSB_FIRST
    );
}

// ----------------------------------------------------------------------------
// Display Functions

void send_display_command(uint8_t c) 
{
    // SSD1309 needs CS and CMD to be low for commands
    gpio_put(PIN_CHIP_SELECT, 1);
    gpio_put(PIN_COMMAND, 0);
    sleep_ms(1);
    gpio_put(PIN_CHIP_SELECT, 0);

    size_t data_len = sizeof(c);
    spi_write_blocking(SPI_PORT, &c, data_len);

    gpio_put(PIN_COMMAND, 1);
    gpio_put(PIN_CHIP_SELECT, 1);

}


void send_display_data(uint8_t c) 
{
    // SSD1309 needs CS to be low and CMD to be high for data
    gpio_put(PIN_CHIP_SELECT, 1);
    gpio_put(PIN_COMMAND, 1);
    sleep_ms(1);
    gpio_put(PIN_CHIP_SELECT, 0);

    size_t data_len = sizeof(c);
    spi_write_blocking(SPI_PORT, &c, data_len);

    gpio_put(PIN_CHIP_SELECT, 1);

}

void initialize_display ()
{
    // Reset the display
    gpio_put(PIN_RESET, 1);
    sleep_ms(10);
    gpio_put(PIN_RESET, 0);
    sleep_ms(10);
    gpio_put(PIN_RESET, 1);

    // Configure the display how we want it set up
    send_display_command(SSD1309_CMD_NORMAL_DISPLAY);
    send_display_command(SSD1309_CMD_ENTIRE_DISPLAY_OFF);
    send_display_command(SSD1309_CMD_MEMORY_ADDRESSING_MODE);
    send_display_command(0x00);
    send_display_command(SSD1309_CMD_SCAN_DIRECTION_COMN_1_START);
    send_display_command(SSD1309_CMD_DISPLAY_ON);
}

// Send zeros to the display to clear the contents
void display_clear ()
{
    gpio_put(PIN_CHIP_SELECT, 1);
    gpio_put(PIN_COMMAND, 1);
    sleep_ms(1);
    gpio_put(PIN_CHIP_SELECT, 0);

    uint8_t zeros = 0;

    for (uint32_t i = 0; i < ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8); i++) {
        spi_write_blocking(SPI_PORT, &zeros, sizeof(zeros));
    }

    gpio_put(PIN_CHIP_SELECT, 1);
}

// Write the display buffer out to the OLED display using SPI
void display_buffer ()
{
    // Reset state and wait
    gpio_put(PIN_CHIP_SELECT, 1);
    gpio_put(PIN_COMMAND, 1);
    sleep_ms(1);
    gpio_put(PIN_CHIP_SELECT, 0);

    for (uint16_t y = 0; y < DISPLAY_HEIGHT; y=y+8) {
        for (uint16_t x = 0; x < DISPLAY_WIDTH; x++) {
            uint8_t pixel_col = 0;

            pixel_col |= (output_buffer[y+0][x] << 0); // Shift b0 to the 0th position (LSB)
            pixel_col |= (output_buffer[y+1][x] << 1); // Shift b1 to the 1st position
            pixel_col |= (output_buffer[y+2][x] << 2); // Shift b2 to the 2nd position
            pixel_col |= (output_buffer[y+3][x] << 3); // Shift b3 to the 3rd position
            pixel_col |= (output_buffer[y+4][x] << 4); // Shift b4 to the 4th position
            pixel_col |= (output_buffer[y+5][x] << 5); // Shift b5 to the 5th position
            pixel_col |= (output_buffer[y+6][x] << 6); // Shift b6 to the 6th position
            pixel_col |= (output_buffer[y+7][x] << 7); // Shift b7 to the 7th position (MSB)

            spi_write_blocking(SPI_PORT, &pixel_col, 1);
        }
    }

    gpio_put(PIN_CHIP_SELECT, 1);

}

// Map bit positions in each 4-bit sample to pixel order.
// Bits within a sample are (D1..D4) -> bit positions (0..3).
// If you want pixels to be D1,D2,D3,D4 in that order, leave as {0,1,2,3}.
// If your wiring means the first pixel should be D4, then D3, D2, D1, use {3,2,1,0}.
static const uint8_t BIT_TO_PIXEL[4] = { 0, 1, 2, 3 };

// Get the data out of the input buffer (supplied by the PIO) and then write the data out to the new display
void copy_input_to_output_buffer ()
{
    for (uint16_t yy = DISPLAY_HEIGHT; yy > 0; yy--) {
        uint16_t y = yy -1;
        int x_base = 0;
        int NIBBLE_MSB_FIRST = 0;

        for (uint16_t ww = WORDS_PER_LINE; ww > 0; ww--) {
            uint16_t w = ww-1;
            uint32_t word = frame_bits[y][w];
            
            for (int s = 0; s < 8; ++s) {
                // Pick which 4-bit nibble to read
                int nib_idx = NIBBLE_MSB_FIRST ? (7 - s) : s; // 7..0 or 0..7
                uint8_t sample4 = (word >> (nib_idx * 4)) & 0xF;

                // Write 4 pixels for this sample
                // Pixel order along X for this sample uses BIT_TO_PIXEL mapping
                int x = x_base + s * 4;
                output_buffer[y][x] = ((sample4 >> BIT_TO_PIXEL[0]) & 1); // pixel #0
                output_buffer[y][x + 1] = ((sample4 >> BIT_TO_PIXEL[1]) & 1); // pixel #0
                output_buffer[y][x + 2] = ((sample4 >> BIT_TO_PIXEL[2]) & 1); // pixel #0
                output_buffer[y][x + 3] = ((sample4 >> BIT_TO_PIXEL[3]) & 1); // pixel #0
            }

            x_base += 8 * 4; // 8 samples * 4 pixels per sample = 32 pixels per word
        }
    }
}

// Send the input buffer to the console using text glyphs to display the screen
// Can be useful to know what is actually in the input buffer when there are display issues
void debug_input_buffer_to_console ()
{
        // Display text of the buffer
        for (uint16_t y = 0; y < DISPLAY_HEIGHT; y++) {
            int x_base = 0;
            int NIBBLE_MSB_FIRST = 1;

            for (uint16_t w = 0; w < WORDS_PER_LINE; w++) {
                uint32_t word = frame_bits[y][w];
                
                for (int s = 0; s < 8; ++s) {
                    // Pick which 4-bit nibble to read
                    int nib_idx = NIBBLE_MSB_FIRST ? (7 - s) : s; // 7..0 or 0..7
                    uint8_t sample4 = (word >> (nib_idx * 4)) & 0xF;

                    // Write 4 pixels for this sample
                    // Pixel order along X for this sample uses BIT_TO_PIXEL mapping
                    int x = x_base + s * 4;
                    if (sample4 >> BIT_TO_PIXEL[3] & 1) // pixel #3
                        printf(".");
                    else
                        printf(" ");

                    if (sample4 >> BIT_TO_PIXEL[2] & 1) // pixel #2
                        printf(".");
                    else
                        printf(" ");

                    if (sample4 >> BIT_TO_PIXEL[1] & 1) // pixel #1
                        printf(".");
                    else
                        printf(" ");

                    if (sample4 >> BIT_TO_PIXEL[0] & 1) // pixel #0
                        printf(".");
                    else
                        printf(" ");
                }

                x_base += 8 * 4; // 8 samples * 4 pixels per sample = 32 pixels per word
            }
            
            printf("\n");
        }

        printf("----\n");
}

// -- Data Input Functions ----------------------------------------------------

// PIO IRQ handler: triggered after each line (IRQ 0)
static void on_pio_irq(void) {

    pio_interrupt_clear(pio, 0);

    if (++current_line >= DISPLAY_HEIGHT) {
        current_line = 0; // Frame complete
    }

    // Re-arm DMA for next line
    dma_channel_set_write_addr(dma_ch, frame_bits[current_line], false);
    dma_channel_set_trans_count(dma_ch, WORDS_PER_LINE, false);
    dma_channel_start(dma_ch);
}

// -- Main Loop ---------------------------------------------------------------

int main()
{
    stdio_init_all();
    printf("Initial hardware configuration and init\n");

    // Initialize inputs and outputs, get the display ready
    initialize_input_pins();
    initialize_spi();
    initialize_display();

    // --- Load and configure PIO program (do NOT enable yet) ---
    uint offset = pio_add_program(pio0, &display_input_program);
    sm = pio_claim_unused_sm(pio0, true);

    pio_sm_config sm_cfg = display_input_program_get_default_config(offset);
    sm_config_set_in_pins(&sm_cfg, DISPLAY_INPUT_PIN_D1);
    sm_config_set_in_shift(&sm_cfg, false, true, 32);   // autopush@32, 32-bit words
    sm_config_set_clkdiv(&sm_cfg, 1.0f);

    // Initialize SM at the program start, but keep it disabled for now
    pio_sm_init(pio0, sm, offset, &sm_cfg);
    pio_sm_set_enabled(pio0, sm, false);

    // Ensure clean start
    pio_sm_clear_fifos(pio0, sm);
    pio_sm_restart(pio0, sm);

    // --- DMA: PIO RX FIFO → RAM buffer ---
    dma_ch = dma_claim_unused_channel(true);
    dma_channel_config dmc = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&dmc, DMA_SIZE_32);      // <-- explicit 32-bit
    channel_config_set_read_increment(&dmc, false);                 // RX FIFO addr fixed
    channel_config_set_write_increment(&dmc, true);                 // walk dest buffer
    channel_config_set_dreq(&dmc, pio_get_dreq(pio0, sm, false));    // false => RX DREQ

    // Configure (don't start yet)
    dma_channel_configure(dma_ch, &dmc,
                        frame_bits[0],          // dst
                        &pio->rxf[sm],          // src
                        WORDS_PER_LINE,         // 5 words per line
                        false);                 // don't start

    // --- PIO interrupt setup ---
    irq_set_exclusive_handler(PIO0_IRQ_0, on_pio_irq);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);

    // --- Arm DMA before enabling the SM ---
    current_line = 0;
    dma_channel_set_write_addr(dma_ch, frame_bits[current_line], false);
    dma_channel_set_trans_count(dma_ch, WORDS_PER_LINE, false);
    dma_channel_start(dma_ch);

    // Now let the SM run so it can feed the FIFO that DMA drains
    pio_sm_set_enabled(pio0, sm, true);

    // ------------------------------------------------------------------------

    printf("Starting main while loop\n");

    display_clear();

    while (true) {
        sleep_ms(100);
        copy_input_to_output_buffer();
        display_buffer();
        
        // Uncomment to send text representation of the input buffer to the console
        //debug_input_buffer_to_console();
    }
}
