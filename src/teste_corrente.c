/**
 * @file main.c
 * @brief Segmentação Automática de Imagens em Tons de Cinza (Método de Otsu) utilizando o Microcontrolador STM32
 * @version 1.0
 * @date 20 de Novembro de 2025
 *
 * @authors
 * - Victor Linhares Alves
 * - Mabiu Moura Coelho
 *
 * Engenharia da Computação – IFCE
 * SEMB (Sistemas Embarcados)
 *
 * @section descricao_geral Descrição Geral
 * Este programa realiza a segmentação automática de imagens em tons de cinza
 * utilizando o método de Otsu em um microcontrolador STM32. O algoritmo determina
 * o limiar ótimo que separa duas classes de pixels (fundo e objeto) pela
 * maximização da variância entre classes. A imagem é recebida via UART, processada
 * em streaming para economia de RAM, e os resultados (limiar e hashes de validação)
 * são enviados de volta via UART.
 *
 * @section alteracoes Desenvolvimento / Alterações / Melhorias
 * - 10/11 Adaptação para leitura via UART em vez de arquivos PGM, processando byte a byte para atender aos limites de RAM (8KB) do STM32F030.
 * - 12/11 - 14/11 Adição de hashes (FNV-1a e Fletcher-32) para validação de integridade da transmissão UART.
 * - 12/11 - 14/11 Implementação de loop infinito com handshake "READY" para processar múltiplas imagens sem reset.
 *
 * @section entrada Entrada
 * - Imagem em tons de cinza (90x90 pixels, intensidade 0-255) recebida via UART como stream de bytes (8100 bytes).
 *
 * @section saida Saída
 * - Via UART: Limiar de Otsu (inteiro), soma dos pixels, hash FNV-1a 64-bit e checksum Fletcher-32.
 * - Feedback visual: LED pisca 2x após processamento.
 *
 * @section uso Como Usar
 * 1. Compilar e flashar no STM32F030 Nucleo-64 usando STM32CubeIDE ou GCC.
 *    @code
 *    make all && openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "program build/main.elf verify reset exit"
 *    @endcode
 * 2. Executar o script Python de teste (teste_otsu_placa.py) para enviar a imagem e validar resultados.
 *
 * @section plataforma Plataforma Alvo
 * - Microcontrolador: **STM32F030 Nucleo-64**
 * - Compilador: **GCC (GNU C Compiler)** ou STM32CubeIDE
 * - Periféricos: UART2 (38400 baud), GPIO para LED (LD2).
 *
 * @section licenca Licença
 * Código livre para uso acadêmico e educacional, mediante citação dos autores.
 * © 2025 Victor Linhares Alves e Mabiu Coelho. Todos os direitos reservados.
 *
 */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** Número máximo de tons de cinza (0–255). */
#define MAX_GRAY 256
/** Dimensões fixas da imagem. */
#define WIDTH 90
/** Dimensões fixas da imagem. */
#define HEIGHT 90
/** Quantidade total de pixels. */
#define SIZE (WIDTH * HEIGHT)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/** Histograma bruto — quantidade de pixels em cada nível de cinza. */
static uint32_t occurrence[MAX_GRAY];

/** Histograma normalizado — probabilidade de ocorrência de cada nível. */
static double histogram[MAX_GRAY];

/** Soma total dos valores dos pixels (usada como hash simples). */
static uint32_t sum_pixels;

/** Hash FNV-1a 64-bit para validação de integridade. */
static uint64_t hash64_fnv1a;

/** Checksum Fletcher-32 para validação de integridade. */
static uint32_t hash32_fletcher;

/** Variáveis temporárias para o checksum Fletcher-32. */
static uint32_t fletcher_s1, fletcher_s2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/** @brief Calcula o histograma, soma e hashes em streaming durante a recepção via UART */
static void calc_histogram_and_sum(void);

/** @brief Calcula o limiar */
static double otsu_threshold(double *hist);

/** @brief Envia sinal "READY" via UART para indicar prontidão para receber imagem */
static void uart_send_ready(void);

/** @brief Envia os resultados (limiar, soma e hashes) via UART */
static void uart_send_results(int thr, uint32_t sum);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * @brief Envia o sinal de handshake "READY" via UART.
 *
 * Esta função formata e transmite a string "READY\r\n" para sincronizar com o script Python.
 */
static void uart_send_ready(void)
{
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "READY\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
}

/**
 * @brief Recebe a imagem via UART byte a byte, calculando histograma, soma e hashes em streaming.
 *
 * Processa cada pixel recebido, sobrescrevendo cada byte em uma variável p poupar RAM.
 * Inclui timeout por byte para evitar travamentos e hashes para validação de integridade.
 *
 * @note Se ocorrer timeout durante a recepção, a função retorna sem processar.
 */
static void calc_histogram_and_sum(void)
{
  sum_pixels = 0;

  // Inicializa occurrence e histogram
  for (int i = 0; i < MAX_GRAY; i++) {
    occurrence[i] = 0;
    histogram[i] = 0.0;
  }

  // Inicia hashes (mantém igual, agora com dados mockados)
  hash64_fnv1a = 14695981039346656037ULL;
  fletcher_s1 = 0xFFFFu;
  fletcher_s2 = 0xFFFFu;
  uint32_t counter = 0;

  // Gera pixels mockados on-the-fly: v = i % 256
  for (uint32_t i = 0; i < SIZE; i++) {
    uint8_t v = (uint8_t)(i & 0xFF); // mesmo que i % 256, mas mais rápido

    occurrence[v]++;
    sum_pixels += v;

    // Atualiza FNV
    hash64_fnv1a ^= (uint64_t)v;
    hash64_fnv1a *= 1099511628211ULL;

    // Atualiza Fletcher
    fletcher_s1 += v;
    fletcher_s2 += fletcher_s1;
    counter++;
    if (counter >= 360) {
      fletcher_s1 = (fletcher_s1 & 0xFFFFu) + (fletcher_s1 >> 16);
      fletcher_s2 = (fletcher_s2 & 0xFFFFu) + (fletcher_s2 >> 16);
      counter = 0;
    }
  }

  // Finaliza Fletcher
  fletcher_s1 = (fletcher_s1 & 0xFFFFu) + (fletcher_s1 >> 16);
  fletcher_s2 = (fletcher_s2 & 0xFFFFu) + (fletcher_s2 >> 16);
  hash32_fletcher = (fletcher_s2 << 16) | (fletcher_s1 & 0xFFFFu);

  // Normaliza o histograma
  double total = (double)SIZE;
  for (int i = 0; i < MAX_GRAY; i++) {
    histogram[i] = (double)occurrence[i] / total;
  }
}


/**
 * @brief Calcula o limiar ótimo de Otsu com base na variância entre classes.
 *
 * @param hist Vetor com o histograma normalizado da imagem
 * @return Valor real (double) do limiar ótimo
 */
static double otsu_threshold(double *hist)
{
  double prob[MAX_GRAY];
  double mean[MAX_GRAY];
  double max_between = 0.0;
  double threshold = 0.0;

  prob[0] = hist[0];
  mean[0] = 0.0;

  for (int i = 1; i < MAX_GRAY; i++) {
    prob[i] = prob[i - 1] + hist[i];
    mean[i] = mean[i - 1] + (double)i * hist[i];
  }

  for (int i = 0; i < MAX_GRAY - 1; i++) {
    if (prob[i] > 0.0 && prob[i] < 1.0) {
      double val = mean[MAX_GRAY - 1] * prob[i] - mean[i];
      double between = (val * val) / (prob[i] * (1.0 - prob[i]));
      if (between > max_between) {
        max_between = between;
        threshold = (double)i;
      }
    }
  }

  return threshold;
}

/**
 * @brief Envia os resultados (limiar, soma e hashes) via UART para validação.
 *
 * @param thr Limiar inteiro arredondado
 * @param sum Soma dos pixels
 */
static void uart_send_results(int thr, uint32_t sum)
{
  char buf[96];
  int n = snprintf(buf, sizeof(buf), "THR:%d\r\n", thr);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
  n = snprintf(buf, sizeof(buf), "SUM:%lu\r\n", (unsigned long)sum);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  // Split FNV para evitar problema com %llX
  unsigned long fnv_high = (unsigned long)(hash64_fnv1a >> 32);
  unsigned long fnv_low = (unsigned long)(hash64_fnv1a & 0xFFFFFFFFUL);
  n = snprintf(buf, sizeof(buf), "FNV1A64:0x%08lX%08lX\r\n", fnv_high, fnv_low);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  n = snprintf(buf, sizeof(buf), "FLETCH32:0x%08lX\r\n", (unsigned long)hash32_fletcher);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
 * @brief Ponto de entrada principal do programa.
 *
 * Etapas executadas em loop infinito:
 * 1. Envio de "READY" via UART para sincronização.
 * 2. Recepção e processamento da imagem via UART (histograma, soma e hashes).
 * 3. Cálculo do limiar ótimo de Otsu.
 * 4. Arredondamento do limiar usando round().
 * 5. Envio dos resultados via UART.
 * 6. Feedback visual via LED.
 *
 * @return Não retorna (loop infinito).
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  while (1)
  {
    calc_histogram_and_sum();
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    double t_real = otsu_threshold(histogram);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    int thr = (int)round(t_real);
    HAL_Delay(100);

    // evitar warning (já que não envia por UART)
    (void)thr;
    (void)sum_pixels;
    (void)hash64_fnv1a;
    (void)hash32_fletcher;

    // Opção A: corrente máxima (recomendado pra medir consumo do processamento)
    // (sem delay)

    // Opção B: visível no olho, mas altera MUITO a corrente média:
    // HAL_Delay(100);
    // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

/* ======= Inicialização de periféricos ======= */
/**
 * @brief Configura o clock do sistema.
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief Inicializa a UART2 para comunicação.
 */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief Inicializa os GPIOs (LED e botão).
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Botão de usuário */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* LED2 */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Manipulador de erros (loop infinito).
 */
void Error_Handler(void)
{
__disable_irq();
while (1) { }
}
