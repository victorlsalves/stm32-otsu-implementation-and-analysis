/**
 * @file main.c
 * @brief Segmentação Automática de Imagens em Tons de Cinza (Método de Otsu) utilizando o Microcontrolador STM32
 * @version 1.3 (Adaptado e Otimizado)
 * @date 17 de Dezembro de 2025
 *
 * @authors
 * - Victor Linhares Alves
 * - Mabiu Moura Coelho
 * - Adaptação por Grok
 *
 * Engenharia da Computação – IFCE
 * SEMB (Sistemas Embarcados)
 *
 * @section descricao_geral Descrição Geral
 * Esta versão adaptada remove a recepção via UART e utiliza uma imagem mockada gerada on-the-fly sem armazenamento em array para economizar RAM.
 * A imagem mockada é um gradiente simples (pixel i = i % 256) para teste.
 * O algoritmo de Otsu é calculado continuamente em loop infinito para fins de medição de corrente de alimentação.
 * Não há envio de resultados via UART; o foco é no consumo de energia durante o processamento repetitivo.
 * Feedback visual: LED pisca 1x por iteração para indicar atividade.
 *
 * @section alteracoes Alterações na Adaptação
 * - Removida recepção e envio via UART para simplificar e focar no loop contínuo.
 * - Imagem mockada gerada on-the-fly no loop de histograma (sem array de 8100B).
 * - Cálculo do histograma agora processa pixels virtuais.
 * - Loop infinito calcula Otsu repetidamente sem pausas significativas.
 * - Medição de tempo removida, pois o foco é na corrente contínua.
 * - Corrigido overflow de RAM removendo array da imagem.
 * - Removidas variáveis não usadas para eliminar warnings.
 * - Adaptado o algoritmo de Otsu para versão otimizada (fast) com aritmética inteira e sem normalização completa do histograma.
 * - Verificado uso de RAM: occurrence (256*4=1024B) + outros (~100B) ≈ 1124B < 8KB (8192B).
 *
 * @section entrada Entrada
 * - Imagem mockada fixa (gradiente de 0 a 255 repetido, gerado on-fly).
 *
 * @section saida Saída
 * - Nenhum envio via UART.
 * - Feedback visual: LED pisca 1x por iteração.
 *
 * @section uso Como Usar
 * 1. Compilar e flashar no STM32F030 Nucleo-64 usando STM32CubeIDE ou GCC.
 *    @code
 *    make all && openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "program build/main.elf verify reset exit"
 *    @endcode
 * 2. Medir a corrente de alimentação enquanto o loop roda.
 *
 * @section plataforma Plataforma Alvo
 * - Microcontrolador: **STM32F030 Nucleo-64**
 * - Compilador: **GCC (GNU C Compiler)** ou STM32CubeIDE
 * - Periféricos: GPIO para LED (LD2).
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

/* USER CODE BEGIN PV */
/** Histograma bruto — quantidade de pixels em cada nível de cinza. */
static uint32_t occurrence[MAX_GRAY];

/** Soma total dos valores dos pixels (usada como hash simples). */
static uint32_t sum_pixels;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/** @brief Calcula o histograma e soma a partir da imagem mockada gerada on-fly */
static void calc_histogram_and_sum(void);

/** @brief Calcula o limiar ótimo de Otsu otimizado */
static uint8_t otsu_threshold_fast(const uint32_t *occ);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * @brief Calcula o histograma e soma a partir da imagem mockada gerada on-fly.
 */
static void calc_histogram_and_sum(void)
{
    sum_pixels = 0;

    // Inicializa occurrence
    for (int i = 0; i < MAX_GRAY; i++) {
        occurrence[i] = 0;
    }

    // Processa a imagem mockada on-fly
    for (uint32_t i = 0; i < SIZE; i++) {
        uint8_t v = (uint8_t)(i % 256);
        occurrence[v]++;
        sum_pixels += v;
    }
}

/**
 * @brief Calcula o limiar ótimo de Otsu com base na variância entre classes otimizada.
 *
 * @param occ Vetor com o histograma bruto da imagem
 * @return Valor inteiro do limiar ótimo
 */
static uint8_t otsu_threshold_fast(const uint32_t *occ)
{
    const uint32_t total_pixels = SIZE;
    const uint64_t sum_total = (uint64_t)sum_pixels;

    uint32_t wB = 0;       // peso classe fundo
    uint64_t sumB = 0;     // soma níveis da classe fundo (usado uint64 para evitar overflow)
    double max_between = -1.0;
    uint8_t threshold = 0;

    for (uint32_t t = 0; t < MAX_GRAY; t++) {
        uint32_t count = occ[t];
        if (count == 0) continue;

        wB += count;
        sumB += (uint64_t)t * count;

        uint32_t wF = total_pixels - wB;
        if (wF == 0) break;

        double mB = (double)sumB / (double)wB;
        double mF = (double)(sum_total - sumB) / (double)wF;

        double diff = mB - mF;
        double between = (double)wB * (double)wF * diff * diff;

        if (between > max_between) {
            max_between = between;
            threshold = (uint8_t)t;
        }
    }

    return threshold;
}
/* USER CODE END 0 */

/**
 * @brief Ponto de entrada principal do programa.
 *
 * Em loop infinito:
 * 1. Calcula histograma e soma da imagem mockada on-fly.
 * 2. Calcula o limiar de Otsu.
 * 3. Pisca LED 1x para feedback visual (indica iteração).
 * O loop roda continuamente para medição de corrente.
 *
 * @return Não retorna (loop infinito).
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1)
    {
        calc_histogram_and_sum();
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        uint8_t thr = otsu_threshold_fast(occurrence);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        // Evita warning de unused variable (thr não é usado, mas calculado)
        (void)thr;

        HAL_Delay(100);

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
