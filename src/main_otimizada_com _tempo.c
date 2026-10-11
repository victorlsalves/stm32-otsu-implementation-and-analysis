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

/** Soma total dos valores dos pixels (usada como hash simples). */
static uint32_t sum_pixels;

/** Hash FNV-1a 64-bit para validação de integridade. */
static uint64_t hash64_fnv1a;

/** Checksum Fletcher-32 para validação de integridade. */
static uint32_t hash32_fletcher;

/** Variáveis temporárias para o checksum Fletcher-32. */
static uint32_t fletcher_s1, fletcher_s2;

/** Tempo de execução do algoritmo de Otsu em ms (medido na própria placa). */
static uint32_t algo_time_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/** @brief Calcula o histograma, soma e hashes em streaming durante a recepção via UART */
static int calc_histogram_and_sum(void);

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
static int calc_histogram_and_sum(void)
{
    uint8_t byte;
    sum_pixels = 0;

    for (int i = 0; i < MAX_GRAY; i++) occurrence[i] = 0;

    hash64_fnv1a = 14695981039346656037ULL;
    fletcher_s1 = 0xFFFFu;
    fletcher_s2 = 0xFFFFu;
    uint32_t counter = 0;

    for (uint32_t i = 0; i < SIZE; i++) {
        if (HAL_UART_Receive(&huart2, &byte, 1, 100) != HAL_OK) {
            return 0; // FALHOU
        }

        uint8_t v = byte;
        occurrence[v]++;
        sum_pixels += v;

        hash64_fnv1a ^= (uint64_t)v;
        hash64_fnv1a *= 1099511628211ULL;

        fletcher_s1 += v;
        fletcher_s2 += fletcher_s1;
        counter++;
        if (counter >= 360) {
            fletcher_s1 = (fletcher_s1 & 0xFFFFu) + (fletcher_s1 >> 16);
            fletcher_s2 = (fletcher_s2 & 0xFFFFu) + (fletcher_s2 >> 16);
            counter = 0;
        }
    }

    fletcher_s1 = (fletcher_s1 & 0xFFFFu) + (fletcher_s1 >> 16);
    fletcher_s2 = (fletcher_s2 & 0xFFFFu) + (fletcher_s2 >> 16);
    hash32_fletcher = (fletcher_s2 << 16) | (fletcher_s1 & 0xFFFFu);

    return 1; // OK
}



#include <stddef.h>  // size_t
#include <stdio.h>
#include <stdint.h>

/* -----------------------------
   Medição de RAM do OTSU (sem interferir no resultado)
   - Apenas sizeof() (constante em tempo de compilação)
   - Não cria arrays locais
   - Não lê/escreve histogramas
-------------------------------- */

/* 1) Variáveis GLOBAIS realmente usadas pelo OTSU (threshold_fast)
      - occurrence[]: histograma bruto
      - sum_pixels: soma total (vira sum_total)
   Obs: hashes/uart/tempo NÃO entram aqui porque você pediu "variáveis do algoritmo",
   ignorando entrada/saída.
*/
#define OTSU_RAM_GLOBAL_BYTES \
    ( sizeof(occurrence) + sizeof(sum_pixels) )

/* 2) Stack do otsu_threshold_fast (estimativa conservadora)
   IMPORTANTE:
   - Compilador pode colocar muita coisa em registradores (stack real pode ser menor).
   - Isso é um "pior caso razoável" baseado nas variáveis declaradas na função.
*/
#define OTSU_RAM_STACK_EST_BYTES \
    ( sizeof(uint32_t) /* total_pixels */ \
    + sizeof(uint64_t) /* sum_total */ \
    + sizeof(uint32_t) /* wB */ \
    + sizeof(uint32_t) /* wF */ \
    + sizeof(uint64_t) /* sumB */ \
    + sizeof(double)   /* max_between */ \
    + sizeof(uint8_t)  /* threshold */ \
    + sizeof(uint32_t) /* t (loop) */ \
    + sizeof(uint32_t) /* count (loop tmp) */ \
    + sizeof(double)   /* mB */ \
    + sizeof(double)   /* mF */ \
    + sizeof(double)   /* diff */ \
    + sizeof(double)   /* between */ )

#define OTSU_RAM_TOTAL_EST_BYTES (OTSU_RAM_GLOBAL_BYTES + OTSU_RAM_STACK_EST_BYTES)

static void otsu_get_memory_usage(size_t *ram_globals, size_t *stack_min, size_t *total)
{
    size_t g = 0;
    size_t s = 0;

    g += sizeof(occurrence);
    g += sizeof(sum_pixels); // faz parte do Otsu (soma total usada no cálculo)

    // stack mínima (estimada) só das variáveis do otsu_threshold_fast
    s += sizeof(uint32_t);  // total_pixels
    s += sizeof(uint64_t);  // sum_total
    s += sizeof(uint32_t);  // wB
    s += sizeof(uint32_t);  // wF
    s += sizeof(uint64_t);  // sumB
    s += sizeof(double);    // max_between
    s += sizeof(uint8_t);   // threshold
    s += sizeof(uint32_t);  // t
    s += sizeof(uint32_t);  // count
    s += sizeof(double);    // mB
    s += sizeof(double);    // mF
    s += sizeof(double);    // diff
    s += sizeof(double);    // between

    if (ram_globals) *ram_globals = g;
    if (stack_min)   *stack_min   = s;
    if (total)       *total       = g + s;
}


/**
 * @brief Calcula o limiar ótimo de Otsu com base na variância entre classes.
 *
 * @param hist Vetor com o histograma normalizado da imagem
 * @return Valor real (double) do limiar ótimo
 */


static uint8_t otsu_threshold_fast(const uint32_t *occ)
{

    const uint32_t total_pixels = SIZE;
    const uint64_t sum_total = (uint64_t)sum_pixels;

    uint32_t wB = 0;       // peso classe fundo
    uint32_t wF = 0;       // peso classe objeto
    uint64_t sumB = 0;     // soma níveis da classe fundo

    double max_between = -1.0;
    uint8_t threshold = 0;

    for (uint32_t t = 0; t < MAX_GRAY; t++) {
        uint32_t count = occ[t];
        if (count == 0) {
            continue;
        }

        wB += count;
        wF = total_pixels - wB;
        if (wF == 0) {
            break;
        }

        sumB += (uint64_t)t * count;

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

  unsigned long fnv_high = (unsigned long)(hash64_fnv1a >> 32);
  unsigned long fnv_low  = (unsigned long)(hash64_fnv1a & 0xFFFFFFFFUL);
  n = snprintf(buf, sizeof(buf), "FNV1A64:0x%08lX%08lX\r\n", fnv_high, fnv_low);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  n = snprintf(buf, sizeof(buf), "FLETCH32:0x%08lX\r\n", (unsigned long)hash32_fletcher);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  n = snprintf(buf, sizeof(buf), "TIME_ALGO:%lu\r\n", (unsigned long)algo_time_ms);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  // NOVO: memória do Otsu
  size_t g, s, t;
  otsu_get_memory_usage(&g, &s, &t);

  n = snprintf(buf, sizeof(buf), "OTSURAM_GLOBAL:%lu\r\n", (unsigned long)g);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  n = snprintf(buf, sizeof(buf), "OTSURAM_STACK_MIN:%lu\r\n", (unsigned long)s);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);

  n = snprintf(buf, sizeof(buf), "OTSURAM_TOTAL:%lu\r\n", (unsigned long)t);
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
    uart_send_ready();

    int ok = calc_histogram_and_sum();
    if (!ok) {
        // opcional: avisar erro
        // HAL_UART_Transmit(&huart2,(uint8_t*)"ERR:RX_TIMEOUT\r\n",16,HAL_MAX_DELAY);
        continue; // volta pro começo do while(1) e manda READY de novo
    }
    // Se timeout ocorreu, calc retorna sem processar, loop volta a send READY
    // Se completo, prossegue

    uint32_t tick_start = HAL_GetTick();
    
    uint8_t thr = otsu_threshold_fast(occurrence);

    uint32_t tick_end = HAL_GetTick();
    algo_time_ms = tick_end - tick_start;

    uart_send_results((int)thr, sum_pixels);

    /* Feedback visual (se existir LED2): pisca 2x */
    for (int i = 0; i < 2; i++) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(200);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(200);
    }
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
