"""Captura a telemetria CSV do ESP32 pela serial e permite enviar comandos
(setpoint em cm, 'a' para ativar o controle, 's' para parar os motores).

Uso: pip install pyserial
     python capturar_serial.py
Ctrl+C encerra a captura. Feche o monitor serial da IDE antes de executar.
"""

import sys
import threading
from datetime import datetime

import serial

PORTA = "COM12"
BAUDRATE = 115200

parar = False


def linha_de_dados(linha):
    partes = linha.split(",")
    if len(partes) < 5:
        return False
    try:
        float(partes[0])
        return True
    except ValueError:
        return False


def enviar_comandos(ser):
    while not parar:
        try:
            comando = input()
        except EOFError:
            break
        if parar:
            break
        ser.write((comando + "\n").encode("utf-8"))


def main():
    global parar

    nome_arquivo = f"dados_{datetime.now():%Y%m%d_%H%M%S}.csv"

    try:
        ser = serial.Serial(PORTA, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta {PORTA}: {e}")
        sys.exit(1)

    print(f"Conectado em {PORTA}. Salvando em {nome_arquivo}")
    print("Ctrl+C para parar.\n")

    threading.Thread(target=enviar_comandos, args=(ser,), daemon=True).start()

    linhas = 0
    with open(nome_arquivo, "w", newline="") as f:
        try:
            while True:
                linha = ser.readline().decode("utf-8", errors="ignore").strip()
                if not linha:
                    continue

                if linha.startswith(("t_ms", "#PARAMS")) or linha_de_dados(linha):
                    f.write(linha + "\n")
                    f.flush()
                    linhas += 1
                    if linhas % 100 == 0:
                        print(f"  {linhas} linhas capturadas")
                else:
                    print(f"  ESP32> {linha}")
        except KeyboardInterrupt:
            print(f"\nCaptura encerrada: {linhas} linhas em {nome_arquivo}")
        finally:
            parar = True
            ser.close()


if __name__ == "__main__":
    main()
