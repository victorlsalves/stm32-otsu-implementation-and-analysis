/**
 * @file main.c
 * @brief Segmentação Automática de Imagens em Tons de Cinza (Método de Otsu)
 * @version 2.0
 * @date Outubro de 2025
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
 * utilizando o método de Otsu. O algoritmo determina o limiar ótimo que 
 * separa duas classes de pixels (fundo e objeto) pela maximização da 
 * variância entre classes.
 * 
 * @section alteracoes Desenvolvimento / Alterações / Melhorias
 * - 20/11-23/11 Versão inicial baseada em uma implementação em C++ com uso das bibliotecas **stb_image** e **stb_image_write**.
 * - 25/11-27/11 Substituição das bibliotecas externas **stb_image** e **stb_image_write** pela leitura direta de arquivos PGM (formato P2 - ASCII).
 * - 25/11-27/11 Uso de estruturas de dados estáticas (>8 KB) para atender aos requisitos da disciplina (sem alocação dinâmica ou recursividade).
 * - 28/11-29/11 Ajuste da lógica de arredondamento do limiar com `round()`, garantindo precisão idêntica ao scikit-image (Python).
 * - 18/12 Adicionado medição de tempo para a função de Otsu.
 * - 18/12 Modificado para executar Otsu 10000 vezes e calcular tempo médio para maior precisão.
 * 
 * @section entrada Entrada
 * - Arquivo **foto.pgm** no formato P2 (texto), em greyscale, dimensão 90x90 pixels de intensidade de 0 a 255.
 * 
 * @section saida Saída
 * - Exibição do limiar de Otsu (real e arredondado).  
 * - Tempo de computação da função Otsu em ms.
 * - Arquivo **binarizada.pgm** (formato P2), com a imagem binarizada (0 = fundo, 255 = objeto).
 * 
 * @section uso Como Usar
 * 1. Compilar:
 *    @code
 *    gcc main.c -lm -o otsu
 *    @endcode
 * 2. Executar:
 *    @code
 *    ./otsu
 *    @endcode
 * 3. O programa lerá **foto.pgm** e criará **binarizada.pgm** como resultado.
 * 
 * @section plataforma Plataforma Alvo
 * - Sistemas operacionais: **Windows** ou **Linux**  
 * - Compilador: **GCC (GNU C Compiler)**
 * 
 * @section licenca Licença
 * Código livre para uso acadêmico e educacional, mediante citação dos autores.  
 * © 2025 Victor Linhares Alves e Mabiu Coelho. Todos os direitos reservados.
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <windows.h>

/**  Número máximo de tons de cinza (0–255). */
#define MAX_GRAY 256
/** Dimensões fixas da imagem. */
#define WIDTH 90
#define HEIGHT 90
/** Quantidade total de pixels. */
#define SIZE (WIDTH * HEIGHT)

/* ============================================================================
 *  Estruturas de dados globais
 * ============================================================================
 */

/** Matriz que armazena a imagem original em tons de cinza (entrada). */
uint8_t imagem[HEIGHT][WIDTH];

/** Matriz para armazenar a imagem binarizada (saída). */
uint8_t binarizada[HEIGHT][WIDTH];

/** Histograma bruto — quantidade de pixels em cada nível de cinza. */
uint32_t occurrence[MAX_GRAY];

/** Histograma normalizado — probabilidade de ocorrência de cada nível. */
double histogram[MAX_GRAY];

/* ============================================================================
 *  Função: read_pgm
 * ============================================================================
 */
/**
 * @brief Lê uma imagem no formato PGM P2 (ASCII) e armazena na matriz `img`.
 *
 * @param filename Nome do arquivo de entrada (.pgm)
 * @param img Matriz 2D onde os pixels serão armazenados
 * @param width Ponteiro que armazenará a largura lida do arquivo
 * @param height Ponteiro que armazenará a altura lida do arquivo
 *
 * @return 1 se o arquivo for lido com sucesso, 0 em caso de erro
 *
 * @global img[][] será preenchida com os valores dos pixels
 */
int read_pgm(const char *filename, uint8_t img[HEIGHT][WIDTH], int *width, int *height) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        printf("Erro ao abrir arquivo PGM %s\n", filename);
        return 0;
    }

    char header[3];
    if (fscanf(f, "%2s", header) != 1 || header[0] != 'P' || header[1] != '2') {
        printf("Formato inválido (esperado P2)\n");
        fclose(f);
        return 0;
    }

    // Ignora comentários iniciados com '#'
    int c;
    while ((c = fgetc(f)) == '#') {
        while (fgetc(f) != '\n');
    }
    ungetc(c, f);

    if (fscanf(f, "%d %d", width, height) != 2) {
        printf("Erro ao ler dimensões\n");
        fclose(f);
        return 0;
    }

    int maxval;
    if (fscanf(f, "%d", &maxval) != 1 || maxval != 255) {
        printf("Valor máximo inválido (esperado 255)\n");
        fclose(f);
        return 0;
    }

    for (int y = 0; y < *height; y++) {
        for (int x = 0; x < *width; x++) {
            int val;
            if (fscanf(f, "%d", &val) != 1 || val < 0 || val > 255) {
                printf("Erro ao ler pixel (%d, %d)\n", y, x);
                fclose(f);
                return 0;
            }
            img[y][x] = (uint8_t)val;
        }
    }

    fclose(f);
    return 1;
}

/* ============================================================================
 *  Função: write_pgm
 * ============================================================================
 */
/**
 * @brief Escreve a imagem binarizada no formato PGM P2 (ASCII).
 *
 * @param filename Nome do arquivo de saída (.pgm)
 * @param img Matriz 2D com a imagem binarizada
 * @param width Largura da imagem
 * @param height Altura da imagem
 */
void write_pgm(const char *filename, uint8_t img[HEIGHT][WIDTH], int width, int height) {
    FILE *f = fopen(filename, "w");
    if (!f) {
        printf("Erro ao abrir arquivo PGM para escrita %s\n", filename);
        return;
    }

    fprintf(f, "P2\n");
    fprintf(f, "# Imagem binarizada gerada pelo método de Otsu\n");
    fprintf(f, "%d %d\n", width, height);
    fprintf(f, "255\n");

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            fprintf(f, "%d ", img[y][x]);
        }
        fprintf(f, "\n");
    }

    fclose(f);
}

/* ============================================================================
 *  Função: calc_histogram
 * ============================================================================
 */
/**
 * @brief Calcula o histograma bruto e o histograma normalizado da imagem.
 *
 * @param img Matriz contendo a imagem original
 * @param width Largura da imagem
 * @param height Altura da imagem
 *
 * @global occurrence[] será atualizado com as contagens de pixel
 * @global histogram[] será atualizado com as probabilidades normalizadas
 */
void calc_histogram(uint8_t img[HEIGHT][WIDTH], int width, int height) {
    for (int i = 0; i < MAX_GRAY; i++) {
        occurrence[i] = 0;
        histogram[i] = 0.0;
    }

    // Contagem das ocorrências
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            occurrence[img[y][x]]++;
        }
    }

    double total = (double)(width * height);
    for (int i = 0; i < MAX_GRAY; i++) {
        histogram[i] = occurrence[i] / total;
    }
}

/* ============================================================================
 *  Função: otsu_threshold
 * ============================================================================
 */
/**
 * @brief Calcula o limiar ótimo de Otsu com base na variância entre classes.
 *
 * @param hist Vetor com o histograma normalizado da imagem
 * @return Valor real (double) do limiar ótimo
 */
double otsu_threshold(double *hist) {
    double prob[MAX_GRAY];
    double mean[MAX_GRAY];
    double max_between = 0.0;
    double threshold = 0.0;

    prob[0] = hist[0];
    mean[0] = 0.0;

    for (int i = 1; i < MAX_GRAY; i++) {
        prob[i] = prob[i - 1] + hist[i];
        mean[i] = mean[i - 1] + i * hist[i];
    }

    for (int i = 0; i < MAX_GRAY - 1; i++) {
        if (prob[i] > 0.0 && prob[i] < 1.0) {
            double val = mean[MAX_GRAY - 1] * prob[i] - mean[i];
            double between = (val * val) / (prob[i] * (1.0 - prob[i]));
            if (between > max_between) {
                max_between = between;
                threshold = i;
            }
        }
    }

    return threshold;
}

/* ============================================================================
 *  Função: apply_threshold
 * ============================================================================
 */
/**
 * @brief Aplica o limiar calculado à imagem original, produzindo a imagem binarizada.
 *
 * @param src Imagem original
 * @param dst Imagem binarizada (saída)
 * @param width Largura da imagem
 * @param height Altura da imagem
 * @param threshold Limiar inteiro arredondado
 */
void apply_threshold(uint8_t src[HEIGHT][WIDTH], uint8_t dst[HEIGHT][WIDTH],
                     int width, int height, int threshold) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dst[y][x] = (src[y][x] < threshold) ? 0 : 255;
        }
    }
}

/* ============================================================================
 *  Função principal
 * ============================================================================
 */
/**
 * @brief Ponto de entrada principal do programa.
 *
 * Etapas executadas:
 * 1. Leitura da imagem "foto.pgm"
 * 2. Cálculo do histograma e normalização
 * 3. Cálculo do limiar ótimo de Otsu
 * 4. Arredondamento do limiar usando round()
 * 5. Aplicação do limiar e geração da imagem binarizada
 * 6. Salvamento do resultado em "binarizada.pgm"
 *
 * @return 0 se executado com sucesso, 1 em caso de erro na leitura do arquivo
 */
int main() {
    int width, height;

    if (!read_pgm("foto.pgm", imagem, &width, &height)) {
        return 1;
    }

    printf("Imagem carregada: %dx%d\n", width, height);

    calc_histogram(imagem, width, height);

    LARGE_INTEGER frequency, start, end;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    volatile double sum = 0.0;
    const int iterations = 1000000;
    for (int i = 0; i < iterations; i++) {
        sum += otsu_threshold(histogram);
    }
    double t_real = sum / iterations;

    QueryPerformanceCounter(&end);
    double total_time_s = ((double)(end.QuadPart - start.QuadPart) / frequency.QuadPart);
    double time_ms = total_time_s * 1000.0 / iterations;

    printf("Tempo total para %d iterações: %.6f s\n", iterations, total_time_s);
    printf("Tempo médio por iteração: %.6f ms\n", time_ms);

    int t = (int) round(t_real); // Arredondamento correto
    printf("Limiar de Otsu (real) = %.6f\n", t_real);
    printf("Limiar de Otsu (arredondado) = %d\n", t);
    printf("Tempo de computação de Otsu: %.6f ms\n", time_ms);

    apply_threshold(imagem, binarizada, width, height, t);

    write_pgm("binarizada.pgm", binarizada, width, height);
    printf("Imagem binarizada salva como binarizada.pgm\n");

    printf("Tamanho total da estrutura de imagem: %lu bytes\n", sizeof(imagem));
    return 0;
}