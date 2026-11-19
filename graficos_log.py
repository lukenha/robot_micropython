"""
Gráficos da Avaliação Somativa 7 (roda no PC, NÃO no EV3).

Uso:
    python graficos_log.py log_adapt_XXXX.csv [log_conv_YYYY.csv ...]

Gera, para cada log: erro x tempo e kP_atual x erro (salvos em PNG).
Com mais de um log, os gráficos de erro x tempo saem sobrepostos para o
comparativo P adaptativo x P convencional.

Para baixar os CSV do EV3: no VS Code, painel "EV3 DEVICE BROWSER",
clique com o botão direito no arquivo -> Upload.

Requer: pip install matplotlib
"""

import csv
import os
import sys

import matplotlib.pyplot as plt


def ler(caminho):
    t, erro, kp = [], [], []
    with open(caminho, newline='') as f:
        for linha in csv.DictReader(f):
            t.append(float(linha['tempo_ms']) / 1000.0)
            erro.append(float(linha['erro']))
            kp.append(float(linha['kp']))
    return t, erro, kp


def main(arquivos):
    if not arquivos:
        print(__doc__)
        return

    logs = [(os.path.splitext(os.path.basename(a))[0], ler(a)) for a in arquivos]

    # Erro x tempo
    plt.figure(figsize=(10, 4))
    for nome, (t, erro, _) in logs:
        plt.plot(t, erro, linewidth=1, label='%s (%.2f s)' % (nome, t[-1]))
    plt.axhline(0, color='gray', linewidth=0.8)
    plt.xlabel('tempo (s)')
    plt.ylabel('erro (setpoint - reflexão, %)')
    plt.title('Erro x tempo')
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig('erro_x_tempo.png', dpi=150)

    # kP_atual x erro
    plt.figure(figsize=(6, 4))
    for nome, (_, erro, kp) in logs:
        plt.scatter(erro, kp, s=6, label=nome)
    plt.xlabel('erro (%)')
    plt.ylabel('kP_atual ((mm/s)/%)')
    plt.title('kP_atual x erro')
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig('kp_x_erro.png', dpi=150)

    print('Gerados: erro_x_tempo.png, kp_x_erro.png')
    plt.show()


if __name__ == '__main__':
    main(sys.argv[1:])
