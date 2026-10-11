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
 * 
 * @section entrada Entrada
 * - Arquivo **foto.pgm** no formato P2 (texto), em greyscale, dimensão 90x90 pixels de intensidade de 0 a 255.
 * 
 * @section saida Saída
 * - Exibição do limiar de Otsu (real e arredondado).  
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
    int c = fgetc(f);
    while (c == '#') {
        while (fgetc(f) != '\n' && !feof(f));
        c = fgetc(f);
    }
    ungetc(c, f);

    int maxval;
    fscanf(f, "%d %d %d", width, height, &maxval);
    if (*width != WIDTH || *height != HEIGHT) {
        printf("Dimensões incorretas (esperado %dx%d, recebido %dx%d)\n",
               WIDTH, HEIGHT, *width, *height);
        fclose(f);
        return 0;
    }

    if (maxval != 255) {
        printf("Aviso: valor máximo diferente de 255 (%d)\n", maxval);
    }

    // Leitura da matriz de pixels (dois laços aninhados)
    for (int y = 0; y < *height; y++) {
        for (int x = 0; x < *width; x++) {
            int pixel;
            fscanf(f, "%d", &pixel);
            img[y][x] = (uint8_t) pixel;
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
 * @brief Salva uma matriz de pixels no formato PGM P2 (ASCII).
 *
 * @param filename Nome do arquivo de saída (.pgm)
 * @param img Matriz 2D contendo os valores de pixel (0–255)
 * @param width Largura da imagem
 * @param height Altura da imagem
 */
void write_pgm(const char *filename, uint8_t img[HEIGHT][WIDTH], int width, int height) {
    FILE *f = fopen(filename, "w");
    fprintf(f, "P2\n%d %d\n255\n", width, height);
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
    double t_real = otsu_threshold(histogram);
    int t = (int) round(t_real); // Arredondamento correto
    printf("Limiar de Otsu (real) = %.6f\n", t_real);
    printf("Limiar de Otsu (arredondado) = %d\n", t);

    apply_threshold(imagem, binarizada, width, height, t);

    write_pgm("binarizada.pgm", binarizada, width, height);
    printf("Imagem binarizada salva como binarizada.pgm\n");

    printf("Tamanho total da estrutura de imagem: %lu bytes\n", sizeof(imagem));
    return 0;
}
