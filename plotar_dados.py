"""Plota altura e PWM dos motores a partir do .csv gerado por capturar_serial.py.

Uso: pip install pandas matplotlib
     python plotar_dados.py dados_XXX_XXX.csv
"""

import sys
from io import StringIO

import matplotlib.pyplot as plt
import pandas as pd

COLUNAS = ["t_ms", "altura_cm", "setpoint_cm", "u1_N",
           "pwm_m1", "pwm_m2", "pwm_m3", "pwm_m4"]


def ler_ganhos_pid(caminho):
    """Retorna (Kp, Ki, Kd) da linha '#PARAMS', ou None se nao existir."""
    with open(caminho, "r") as f:
        for linha in f:
            linha = linha.strip()
            if linha.startswith("#PARAMS"):
                partes = linha.split(",")
                if len(partes) == 4:
                    try:
                        return tuple(float(p) for p in partes[1:])
                    except ValueError:
                        return None
    return None


def carregar_dados(caminho):
    with open(caminho, "r") as f:
        linhas = [linha for linha in f if not linha.lstrip().startswith("#")]
    df = pd.read_csv(StringIO("".join(linhas)), names=COLUNAS, header=0)

    n_antes = len(df)
    df = df.apply(pd.to_numeric, errors="coerce").dropna().reset_index(drop=True)
    if n_antes > len(df):
        print(f"Aviso: {n_antes - len(df)} linha(s) invalida(s) descartada(s).")
    return df


def main():
    if len(sys.argv) < 2:
        print("Uso: python plotar_dados.py <arquivo.csv>")
        sys.exit(1)

    caminho = sys.argv[1]
    ganhos = ler_ganhos_pid(caminho)
    df = carregar_dados(caminho)
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(df["t_s"], df["altura_cm"], label="Altura medida", linewidth=1.2)
    ax1.plot(df["t_s"], df["setpoint_cm"], label="Setpoint", linestyle="--", linewidth=1.2)
    ax1.set_ylabel("Altura (cm)")
    if ganhos is not None:
        kp, ki, kd = ganhos
        ax1.set_title(f"Altura vs Tempo (Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f})")
    else:
        ax1.set_title("Altura vs Tempo")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    for i in range(1, 5):
        ax2.plot(df["t_s"], df[f"pwm_m{i}"], label=f"M{i}", linewidth=1.0)
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("PWM (0-4095)")
    ax2.set_title("Sinal de controle (PWM) vs Tempo")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    nome_saida = caminho.rsplit(".", 1)[0] + "_grafico.png"
    plt.savefig(nome_saida, dpi=150)
    print(f"Grafico salvo em: {nome_saida}")
    plt.show()


if __name__ == "__main__":
    main()
