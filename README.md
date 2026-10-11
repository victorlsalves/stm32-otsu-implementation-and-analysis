## Sobre o Projeto
O trabalho foi desenvolvido em três fases principais para a disciplina de Sistemas Embarcados (SEMB):
1. **Implementação Pura (C):** Construção do algoritmo de Otsu do zero, sem uso de bibliotecas de terceiros como OpenCV, para garantir o controle total das rotinas de memória.
2. **Embarcando o Sistema:** Adaptação e compilação cruzada do algoritmo C nativo para a arquitetura do microcontrolador STM32.
3. **Análise de Desempenho (Profiling):** Execução de baterias de testes (scripts Python) para analisar as métricas de tempo de execução, consumo de memória e uso de corrente elétrica da placa em comparação direta com um PC convencional.
