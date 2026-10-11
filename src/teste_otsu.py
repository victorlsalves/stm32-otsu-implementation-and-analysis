from skimage import filters
import numpy as np
import matplotlib.pyplot as plt
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
# Carregar imagem original
# ----------------------------------------------------
img = read_pgm("foto5.pgm")
# ----------------------------------------------------
# Calcular limiar de Otsu com scikit-image
# ----------------------------------------------------
t0 = time.perf_counter()
raw_threshold = filters.threshold_otsu(img)
t1 = time.perf_counter()

t = int(round(raw_threshold))  # arredondamento igual ao do .c

elapsed_ms = (t1 - t0) * 1000

print(f"Limiar (Otsu - scikit-image): {raw_threshold:.6f}")
#print(f"Limiar arredondado para inteiro: {t}")
print(f"\nTempo TOTAL (envio -> processamento -> resposta): {elapsed_ms:.3f} ms")

# ----------------------------------------------------
# Gerar imagem binarizada (referência)
# ----------------------------------------------------
binarizada_referencia = np.where(img < t, 0, 255).astype(np.uint8)

# Ler imagem binarizada gerada pelo programa em C
binarizada_c = read_pgm("binarizada5.pgm")

# Comparar pixel a pixel
if binarizada_c.shape != binarizada_referencia.shape:
    print("Dimensões diferentes entre imagens!")
else:
    diff = np.mean(np.abs(binarizada_referencia - binarizada_c))
    iguais = np.all(binarizada_referencia == binarizada_c)
    print(f"Diferença média por pixel: {diff}")
    print("Imagens são idênticas?", "SIM" if iguais else "NÃO")

# ----------------------------------------------------
# exibir imagens e histograma
# ----------------------------------------------------
fig, axs = plt.subplots(1, 4, figsize=(16, 4))

axs[0].imshow(img, cmap='gray')
axs[0].set_title("Original (PGM)")
axs[0].axis("off")

axs[1].imshow(binarizada_referencia, cmap='gray')
axs[1].set_title("Binarizada Python")
axs[1].axis("off")

axs[2].imshow(binarizada_c, cmap='gray')
axs[2].set_title("Binarizada (C)")
axs[2].axis("off")

hist, bins = np.histogram(img.flatten(), bins=256, range=(0, 255))
axs[3].bar(bins[:-1], hist, width=1.0, color='gray')
axs[3].axvline(x=t, color='red', linestyle='--', linewidth=2)
axs[3].set_title(f"Histograma (Limiar = {t})")
axs[3].set_xlim(0, 255)
axs[3].set_xlabel("Níveis de cinza")
axs[3].set_ylabel("Frequência")

plt.tight_layout()
plt.show()
