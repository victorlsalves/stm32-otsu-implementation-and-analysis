from skimage import filters
import numpy as np
import matplotlib.pyplot as plt
import serial
import time

# ----------------------------------------------------
# Ler arquivo PGM (P2)
# ----------------------------------------------------
def read_pgm(filename):
    with open(filename, "r") as f:
        assert f.readline().strip() == "P2"
        line = f.readline().strip()
        while line.startswith("#"):
            line = f.readline().strip()
        width, height = map(int, line.split())
        maxval = int(f.readline().strip())
        assert maxval == 255
        data = []
        for line in f:
            if line.strip() and not line.startswith("#"):
                data.extend(map(int, line.split()))
        img = np.array(data, dtype=np.uint8).reshape((height, width))
        return img

# ----------------------------------------------------
# Funções de hash
# ----------------------------------------------------
def fnv1a64(data):
    h = 14695981039346656037
    prime = 1099511628211
    for byte in data:
        h ^= byte
        h = (h * prime) & 0xFFFFFFFFFFFFFFFF
    return h

def fletcher32(data):
    s1 = 0xFFFF
    s2 = 0xFFFF
    i = 0
    n = len(data)
    counter = 0
    while i < n:
        s1 += data[i]
        s2 += s1
        i += 1
        counter += 1
        if counter >= 360:
            s1 = (s1 & 0xFFFF) + (s1 >> 16)
            s2 = (s2 & 0xFFFF) + (s2 >> 16)
            counter = 0
    s1 = (s1 & 0xFFFF) + (s1 >> 16)
    s2 = (s2 & 0xFFFF) + (s2 >> 16)
    return (s2 << 16) | s1

# ----------------------------------------------------
# Carregar imagem original
# ----------------------------------------------------
img = read_pgm("foto5.pgm")

# ----------------------------------------------------
# Calcular limiar de Otsu com scikit-image (local)
# ----------------------------------------------------
raw_threshold_local = filters.threshold_otsu(img)
t_local = int(round(raw_threshold_local))  # arredondamento igual ao do .c

print(f"Limiar local (Otsu - scikit-image): {raw_threshold_local:.6f}") # desnecessario
print(f"Limiar local arredondado para inteiro: {t_local}")

# Calcular soma e hashes locais
pixels = img.tobytes()
sum_local = np.sum(img.astype(np.uint32))
fnv_local = fnv1a64(pixels)
flet_local = fletcher32(pixels)

# ----------------------------------------------------
# Comunicação via UART com a placa STM32
# ----------------------------------------------------
port = 'COM9'  # Ajuste o COM aqui, as vezes o COM muda quando a placa é diferente, verificar COM na IDE
ser = serial.Serial(port, 38400, timeout=10)

# limpar buffer
ser.reset_input_buffer()

# aguarda o READY da placa
while True:
    line = ser.readline().decode('utf-8').strip()
    if line == "READY":
        break
    
# INICIAR CONTAGEM DE TEMPO AQUI
t0 = time.perf_counter()

# envia pixels
ser.write(pixels)
#time.sleep(0.1)  # pequeno delay dps do envio

# Recebe resultados
t_board = None
sum_board = None
fnv_board = None
flet_board = None
time_algo_board = None
ram_global = None
ram_stack = None
ram_total = None

while True:
    line = ser.readline().decode('utf-8').strip()
    if not line:
        continue
    if line.startswith("THR:"):
        t_board = int(line.split(":")[1])
    elif line.startswith("SUM:"):
        sum_board = int(line.split(":")[1])
    elif line.startswith("FNV1A64:"):
        # Parse sem '0x' e concat high low
        hex_str = line.split(":")[1][2:]  # remove '0x'
        fnv_board = int(hex_str, 16)
    elif line.startswith("FLETCH32:"):
        flet_board = int(line.split(":")[1], 16)
    elif line.startswith("TIME_ALGO:"):
        time_algo_board = int(line.split(":")[1])
    elif line.startswith("OTSURAM_GLOBAL:"):
        ram_global = int(line.split(":")[1])
        print(line)
    elif line.startswith("OTSURAM_STACK_MIN:"):
        ram_stack = int(line.split(":")[1])
        print(line)
    elif line.startswith("OTSURAM_TOTAL:"):
        ram_total = int(line.split(":")[1])
        print(line)
    elif line.startswith("OTSURAM"):
        print(line)
    if (t_board is not None and sum_board is not None and fnv_board is not None and
    flet_board is not None and time_algo_board is not None and
    ram_global is not None and ram_stack is not None and ram_total is not None):
        break
    
# ENCERRAR CONTAGEM DE TEMPO
t1 = time.perf_counter()
elapsed_ms = (t1 - t0) * 1000  # tempo total em milissegundos


ser.close() 

# ----------------------------------------------------
# Validar resultados
# ----------------------------------------------------
print(f"Limiar recebido da placa: {t_board}")
print(f"Soma recebida da placa: {sum_board}")
print(f"FNV1A64 recebido da placa: 0x{fnv_board:016X}")
print(f"FLETCH32 recebido da placa: 0x{flet_board:08X}")
print(f"Soma local: {sum_local}")
print(f"FNV1A64 local: 0x{fnv_local:016X}")
print(f"FLETCH32 local: 0x{flet_local:08X}")

print(f"\nTempo TOTAL (envio -> processamento -> resposta): {elapsed_ms:.3f} ms")
print(f"Tempo do algoritmo de Otsu na placa: {time_algo_board} ms")
print(f"RAM Otsu (global): {ram_global} bytes")
print(f"RAM Otsu (stack min): {ram_stack} bytes")
print(f"RAM Otsu (total): {ram_total} bytes")

if sum_board == sum_local and fnv_board == fnv_local and flet_board == flet_local and t_board == t_local:
    print("Validação bem-sucedida: Limiares iguais e hashes correspondem!")
else:
    if sum_board != sum_local:
        print("Erro: Soma não corresponde!")
    if fnv_board != fnv_local:
        print("Erro: FNV1A64 não corresponde!")
    if flet_board != flet_local:
        print("Erro: FLETCH32 não corresponde!")
    if t_board != t_local:
        print("Erro: Limiares não iguais!")

# ----------------------------------------------------
# Gerar imagem binarizada local (referência)
# ----------------------------------------------------
binarizada_referencia = np.where(img < t_local, 0, 255).astype(np.uint8)

# ----------------------------------------------------
# Exibir imagens e histograma
# ----------------------------------------------------
fig, axs = plt.subplots(1, 3, figsize=(12, 4))

axs[0].imshow(img, cmap='gray')
axs[0].set_title("Original (PGM)")
axs[0].axis("off")

axs[1].imshow(binarizada_referencia, cmap='gray')
axs[1].set_title("Binarizada Python")
axs[1].axis("off")

hist, bins = np.histogram(img.flatten(), bins=256, range=(0, 255))
axs[2].bar(bins[:-1], hist, width=1.0, color='gray')
axs[2].axvline(x=t_local, color='red', linestyle='--', linewidth=2)
axs[2].set_title(f"Histograma (Limiar = {t_local})")
axs[2].set_xlim(0, 255)
axs[2].set_xlabel("Níveis de cinza")
axs[2].set_ylabel("Frequência")

plt.tight_layout()
plt.show()