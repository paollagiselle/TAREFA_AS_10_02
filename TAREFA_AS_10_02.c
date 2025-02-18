#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define VRX_PIN 26            // Pino do joystick (eixo X)
#define VRY_PIN 27            // Pino do joystick (eixo Y)
#define JOYSTICK_BUTTON 22    // Pino para o botão do joystick
#define BOTAO_A 5             // Pino para o botão A
#define LED_RED 12            // Pino para o LED vermelho
#define LED_GREEN 11          // Pino para o LED verde
#define LED_BLUE 13           // Pino para o LED azul
#define I2C_PORT i2c1         // Configuração do I2C
#define I2C_SDA 14            // Pino SDA do I2C
#define I2C_SCL 15            // Pino SCL do I2C
#define I2C_ADDR 0x3C         // Endereço I2C do display SSD1306
#define DEBOUNCE_TIME 300     // Tempo de debounce para botões

// Variáveis de controle
bool estado_LED_GREEN = false;      
bool estado_LEDS_RGB = true;       
uint32_t ultima_vez_BOTAO_JOYSTICK = 0;
uint32_t ultima_vez_BOTAO_A = 0;
ssd1306_t ssd;                       // Estrutura do display

// Inicializa o PWM em um pino específico (para controlar a intensidade dos LEDs)
uint pwm_init_gpio(uint gpio, uint wrap) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);  // Configura o pino para função PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);            // Define a resolução do PWM
    pwm_set_enabled(slice_num, true);         // Habilita o PWM
    return slice_num;
}

// Controla o brilho dos LEDs com base na leitura do ADC do joystick
void LED_PWM(uint gpio, uint channel) {
    adc_select_input(channel);    // Seleciona o canal do ADC
    uint16_t adc_value = adc_read();  // Lê o valor do joystick
    
    const uint16_t zona_morta = 50;  // Define uma zona morta para evitar pequenas variações

    uint16_t pwm_value = 0;
    if (adc_value > (2048 + zona_morta)) {
        pwm_value = (adc_value - 2048) * 2;  // Ajuste para o valor do PWM
    } else if (adc_value < (2048 - zona_morta)) {
        pwm_value = (2048 - adc_value) * 2;
    }

    pwm_set_gpio_level(gpio, estado_LEDS_RGB ? pwm_value : 0);  // Ajusta a intensidade do LED
}

// Função de interrupção para os botões com debounce
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (gpio == JOYSTICK_BUTTON && (current_time - ultima_vez_BOTAO_JOYSTICK) > DEBOUNCE_TIME) {
        estado_LED_GREEN = !estado_LED_GREEN;  // Alterna o estado do LED verde
        gpio_put(LED_GREEN, estado_LED_GREEN);
        ultima_vez_BOTAO_JOYSTICK = current_time;  // Atualiza o tempo
    }

    if (gpio == BOTAO_A && (current_time - ultima_vez_BOTAO_A) > DEBOUNCE_TIME) {
        estado_LEDS_RGB = !estado_LEDS_RGB;  // Alterna o estado geral dos LEDs RGB
        ultima_vez_BOTAO_A = current_time;
    }
}

// Atualiza o display com a posição do joystick
void atualizar_display(uint16_t x, uint16_t y) {
    // Converte o valor do joystick para a posição do display
    int pos_x = ((x - 2048) * 60) / 2048 + 64;  // Mapeia a posição X
    int pos_y = ((y - 2048) * 28) / 2048 + 32;  // Mapeia a posição Y

    // Garante que o quadrado fique dentro dos limites do display
    if (pos_x < 0) pos_x = 0;
    if (pos_x > 120) pos_x = 120;
    if (pos_y < 0) pos_y = 0;
    if (pos_y > 56) pos_y = 56;

    // Atualiza o display com a nova posição
    ssd1306_fill(&ssd, false);
    ssd1306_rect(&ssd, pos_x, pos_y, pos_x + 8, pos_y + 8, true, true);
    ssd1306_send_data(&ssd);
}

int main() {
    stdio_init_all();       // Inicializa a comunicação serial
    adc_init();             // Inicializa o ADC
    adc_gpio_init(VRX_PIN); // Inicializa o pino do eixo X do joystick
    adc_gpio_init(VRY_PIN); // Inicializa o pino do eixo Y do joystick
    
    i2c_init(I2C_PORT, 400000);  // Inicializa a comunicação I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  // Configura os pinos I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);  // Ativa o pull-up nos pinos I2C
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, I2C_ADDR, I2C_PORT);  // Inicializa o display
    ssd1306_config(&ssd);  // Configura o display
    ssd1306_fill(&ssd, false);  // Limpa o display
    ssd1306_send_data(&ssd);  // Atualiza o display

    // Inicializa os LEDs
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);   // Configura o LED vermelho como saída
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT); // Configura o LED verde
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_BLUE, GPIO_OUT);  // Configura o LED azul
    
    // Inicializa os botões
    gpio_init(JOYSTICK_BUTTON);
    gpio_set_dir(JOYSTICK_BUTTON, GPIO_IN);   // Configura o botão do joystick
    gpio_pull_up(JOYSTICK_BUTTON);            // Ativa o pull-up
    gpio_set_irq_enabled_with_callback(JOYSTICK_BUTTON, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);  // Habilita interrupção

    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);  // Configura o botão A
    gpio_pull_up(BOTAO_A);           // Ativa o pull-up
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);  // Habilita interrupção

    uint pwm_wrap = 4096;  // Define a resolução do PWM
    pwm_init_gpio(LED_RED, pwm_wrap);  // Inicializa o PWM para o LED vermelho
    pwm_init_gpio(LED_BLUE, pwm_wrap); // Inicializa o PWM para o LED azul
    
    // Loop principal
    while (true) {
        uint16_t vrx_value, vry_value;

        adc_select_input(0);         // Lê o valor do eixo X do joystick
        vrx_value = adc_read();
        adc_select_input(1);         // Lê o valor do eixo Y do joystick
        vry_value = adc_read();
        
        LED_PWM(LED_RED, 0);         // Ajusta o brilho do LED vermelho
        LED_PWM(LED_BLUE, 1);        // Ajusta o brilho do LED azul
        atualizar_display(vrx_value, vry_value);  // Atualiza a posição no display
        
        sleep_ms(50);  // Atraso de 50ms antes de repetir
    }
    return 0;
}
