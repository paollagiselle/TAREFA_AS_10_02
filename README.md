# TAREFA - AULA SÍNCRONA - 10/02
# PAOLLA GISELLE RIBEIRO
# MATRÍCULA: TIC370101811

## Descrição Geral
Este projeto utiliza um joystick analógico para controlar um quadrado em um display OLED SSD1306. Além disso, um conjunto de LEDs RGB é controlado via PWM com base nos movimentos do joystick. Um botão no joystick e O BOTÃO A permitem alternar os estados dos LEDs.

## Funcionamento
1. **Controle do Display OLED**
   - O joystick move um quadrado no display.
   - Os valores de VRX e VRY são convertidos em coordenadas X e Y no display.

2. **Controle dos LEDs RGB**
   - Os valores do joystick controlam a intensidade dos LEDs RGB via PWM.
   - O botão A alterna o estado geral dos LEDs (ligados/desligados).
   
3. **Botões**
   - O botão do joystick alterna o LED verde de estado (on/off).
   - O botão A alterna a ativação dos LEDs RGB.

## Bibliotecas Utilizadas
- `pico/stdlib.h` → Configuração do Raspberry Pi Pico
- `hardware/i2c.h` → Comunicação com o display OLED
- `hardware/adc.h` → Leitura dos valores do joystick
- `hardware/pwm.h` → Controle dos LEDs RGB

## Código-Fonte
O código está estruturado em funções:
- `pwm_init_gpio()` → Configuração do PWM nos LEDs
- `LED_PWM()` → Controla a intensidade do LED baseado no joystick
- `gpio_irq_handler()` → Gerencia as interrupções dos botões
- `atualizar_display()` → Atualiza o quadrado no display conforme o joystick
- `main()` → Loop principal que lê o joystick e atualiza os LEDs e display

LINK DO VÍDEO DE APRESENTAÇÃO DO PROJETO: 

