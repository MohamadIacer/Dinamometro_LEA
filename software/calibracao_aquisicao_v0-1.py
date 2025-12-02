import serial
import time
import matplotlib.pyplot as plt
import sys
import msvcrt  # detectar ENTER sem travar

# === CONFIGURAÇÕES Serial ===
PORTA = "COM6"
BAUD = 230400
TIMEOUT = 0.01
espera = 15.0     # segundos para garantir inicialização

# === CONFIGURAÇÕES Calibração ===
TEMPO_CALIBRACAO = 2  # segundos por massa
massas = [4.66, 10.75, 16.59, 23.96, 28.62, 33.63, 38.00]  # g
braco = 26 + 15  # mm → torque

# === CONFIGURAÇÕES Ensaio ===
TEMPO_AQUISICAO = 5.0
TEMPO_ZERO = 5.0
RAMPA_STEP = 5
RAMPA_DELAY = 1

setpoints = [
                 73.30,   # 700 rpm
                 83.78,   # 800 rpm

                 94.25,   # 900 rpm
                 99.48,   # 950 rpm
                 104.72,  # 1000 rpm
                 109.96,  # 1050 rpm
                 115.19,  # 1100 rpm
                 120.43,  # 1150 rpm
                 125.66,  # 1200 rpm
                 130.90,  # 1250 rpm
                 136.14,  # 1300 rpm
                 141.37,  # 1350 rpm
                 146.61,  # 1400 rpm
                 151.84,  # 1450 rpm
                 157.08,  # 1500 rpm

                 167.55,  # 1600 rpm
                 178.02,  # 1700 rpm
                 188.50,  # 1800 rpm
                 198.97,  # 1900 rpm
                 209.44   # 2000 rpm
                 
            ]
################################################################################


# ============================================================
# SPINNER
# ============================================================

def spinner(i):
    símbolos = "|/-\\"
    return símbolos[i % len(símbolos)]

# ============================================================
# LEITURA DE LINHA
# ============================================================

def ler_linha(ser):
    try:
        linha = ser.readline()
        if not linha:
            return None
        linha = linha.decode("utf-8", errors="ignore").strip()
        if not linha:
            return None
        return [float(x) for x in linha.split("\t")]
    except:
        return None

# ============================================================
# COLETA SEM ATRASO + TELEMETRIA EM TEMPO REAL (SOMENTE ENSAIO)
# ============================================================

def coletar_janela(ser, duracao_s, sp=None):
    """
    Coleta dados reais durante duracao_s segundos,
    drenando o buffer por completo e mostrando telemetria
    em tempo real (opção 2) somente quando sp != None.
    """

    ser.reset_input_buffer()  
    time.sleep(0.002)

    t0 = time.time()
    resultados = []
    in_waiting_max = 0
    
    ultimo_print = t0
    intervalo_print = 0.8  # no máximo 10 Hz para evitar acumulo de backlog

    while time.time() - t0 < duracao_s:

        # maior backlog observado
        if ser.in_waiting > in_waiting_max:
            in_waiting_max = ser.in_waiting

        # drena
        while ser.in_waiting > 0:
            dados = ler_linha(ser)
            if dados and len(dados) >= 8:
                timestamp = time.time()
                resultados.append([timestamp] + dados)

                # Telemetria SOMENTE no modo ensaio (sp != None)
                if sp is not None:
                    agora = timestamp
                    if agora - ultimo_print >= intervalo_print:
                        sys.stdout.write("\r"
                            f"SP={sp:.2f} | "
                            f"VelSet={dados[0]:.2f} | VelReal={dados[1]:.2f} | "
                            f"Pos={dados[2]:.4f} | "
                            f"Ax={dados[3]:.4f} | Ay={dados[4]:.4f} | Az={dados[5]:.4f} | "
                            f"V1={dados[6]:.4f} | V2={dados[7]:.4f}       "
                        )
                        sys.stdout.flush()
                        ultimo_print = agora

    print()  # pular linha
    return resultados, in_waiting_max

# ============================================================
# CALIBRAÇÃO
# ============================================================

def calibrar(ser, massas):
    input("\nPressione ENTER para iniciar a calibração...")

    leituras = []
    todas_amostras = []

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,5))

    i = 0
    while i < len(massas):
        m = massas[i]
        print(f"\nColoque a massa de {m} g.")
        resposta = input("Pressione ENTER para medir ou 'd' para descartar a última: ")

        if resposta.lower() == "d" and i > 0:
            ultima_massa = massas[i-1]
            todas_amostras = [am for am in todas_amostras if am[0] != ultima_massa]
            leituras.pop()
            i -= 1
            print(f" Medição da massa {ultima_massa} g descartada.")
            continue
        else:
            print(f"Medição para {m} g...")

        # coleta sem atraso (sem telemetria)
        amostras, _ = coletar_janela(ser, TEMPO_CALIBRACAO, sp=None)

        for a in amostras:
            todas_amostras.append([m, a[7]])  # a[7] = V1 original

        n = len(amostras)
        media = sum(a[7] for a in amostras) / n if n else 0

        if len(leituras) > i:
            leituras[i] = media
        else:
            leituras.append(media)

        print(f"Massa {m} g -> média leitura: {media:.4f}")

        # gráfico igual antes
        ax.clear()
        ax.set_xlabel("Massa [g]")
        ax.set_ylabel("Leitura [int]")
        ax.set_title("Calibração da célula de carga - Canal 1")
        ax.grid(True, linestyle="--", alpha=0.6)

        massas_plot = [am[0] for am in todas_amostras]
        tensoes_plot = [am[1] for am in todas_amostras]
        ax.scatter(massas_plot, tensoes_plot, s=15, alpha=0.5)

        ax.plot(massas[:len(leituras)], leituras, "o-", color="tab:red")
        plt.pause(0.05)

        if i == len(massas) - 1:
            escolha = input("\nÚltima medida concluída. Pressione ENTER para concluir ou 'd' para descartar a última: ").lower()
            if escolha == "d":
                todas_amostras = [am for am in todas_amostras if am[0] != m]
                leituras.pop()
                continue
            else:
                print(" Concluindo calibração, feche a janela do gráfico...")
                i += 1
        else:
            i += 1

    plt.ioff()
    plt.show()

    nome = f"calibracao_samples_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    with open(nome, "w") as f:
        f.write("Massa[g]\tTorque[N.mm]\tLeitura[int]\n")
        for m, v in todas_amostras:
            f.write(f"{m}\t{m*braco*9.81*(1e-3):5f}\t{v}\n")

    print(f"\n Amostras da calibração salvas em {nome}")
    return "ok"

# ============================================================
# RAMPA
# ============================================================

def aplicar_rampa(ser, atual, alvo):
    if atual == alvo:
        return alvo

    passo = RAMPA_STEP if alvo > atual else -RAMPA_STEP
    sp = atual

    while (passo > 0 and sp < alvo) or (passo < 0 and sp > alvo):
        sp += passo
        if (passo > 0 and sp > alvo) or (passo < 0 and sp < alvo):
            sp = alvo
        ser.write(f"T{sp}\n".encode())
        
        sys.stdout.write(f"\rAplicando rampa [rad/s]: {sp} ")
        sys.stdout.flush()
        time.sleep(RAMPA_DELAY)

    return alvo

# ============================================================
# AQUISIÇÃO DOS SETPOINTS
# ============================================================

def aquisitar_varios_setpoints(ser, setpoints, tempo, sp_inicial, atual):
    dados_gerais = []

    print(f"\nAplicando rampa até {sp_inicial} rad/s...")
    atual = aplicar_rampa(ser, atual, sp_inicial)

    print(f"\nSetpoint inicial atingido: {sp_inicial} rad/s")
    print("Aperte ENTER para começar o ensaio.")

    while True:
        if msvcrt.kbhit() and msvcrt.getwch() == "\r":
            break
        time.sleep(0.05)

    # TEMPO ZERO (ESPERA, SEM COLETA)
    print(f"\nHold inicial por {TEMPO_ZERO}s...")

    # descarta tudo que está no buffer antes do hold
    ser.reset_input_buffer()
    time.sleep(0.002)

    t0 = time.time()
    i_spin = 0
    while time.time() - t0 < TEMPO_ZERO:
        restante = TEMPO_ZERO - (time.time() - t0)
        sys.stdout.write(f"\rAguardando estabilização: {restante:4.1f}s {spinner(i_spin)}  ")
        sys.stdout.flush()
        i_spin += 1
        time.sleep(0.05)

    print("\rHold concluído.                                 ")

    # ENSAIO PRINCIPAL
    for sp in setpoints:
        atual = aplicar_rampa(ser, atual, sp)

        print(f"\r--- Setpoint {sp} rad/s ---           ")
        amostras, _ = coletar_janela(ser, tempo, sp=sp)

        for a in amostras:
            dados_gerais.append([sp] + a)

    return dados_gerais, atual

# ============================================================
# SALVA TXT
# ============================================================

def salvar_txt(dados):
    nome = f"aquisicao_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    with open(nome, "w") as f:
        f.write("Setpoint\tTimeStamp\tVelSet\tVelReal\tPos\tAx\tAy\tAz\tV1\tV2\n")
        for linha in dados:
            f.write("\t".join(str(x) for x in linha) + "\n")
    print(f"\n Dados salvos em {nome}")

# ============================================================
# MENU PRINCIPAL
# ============================================================

def main():
    with serial.Serial(PORTA, BAUD, timeout=TIMEOUT) as ser:
        t0 = time.time()
        i_spin = 0

        while time.time() - t0 < espera:
            sys.stdout.write(f"\rGarantindo inicialização... {(espera-(time.time() - t0)):4.1f} {spinner(i_spin)}  ")
            sys.stdout.flush()
            i_spin += 1
            time.sleep(0.05)

        print(f"\rConectado em {PORTA} @ {BAUD}                                          ")

        while True:
            print("\n=== MENU ===")
            print("1 - Calibração")
            print("2 - Ensaio")
            print("3 - Sair")
            op = input("Escolha: ")

            if op == "1":
                print("Iniciando calibração...")
                ser.write(b"M3\n") #zera o ads
                time.sleep(3)
                ser.write(b"M0\n") #volta pro modo normal canal 0
                time.sleep(3)
                calibrar(ser, massas)

            elif op == "2":
                atual = 0

                while True:
                    print("Iniciando ensaio...")
                    ser.write(b"M3\n") #zera o ads
                    time.sleep(3)
                    ser.write(b"M0\n") #volta pro modo normal canal 0
                    time.sleep(3)
                    sp_inicial = float(input("\nDigite o SETPOINT inicial (rad/s): "))

                    dados, ultimo_sp = aquisitar_varios_setpoints(
                        ser, setpoints, TEMPO_AQUISICAO, sp_inicial, atual
                    )

                    salvar_txt(dados)

                    print("\n Voltando para o set inicial...")
                    aplicar_rampa(ser, ultimo_sp, sp_inicial)
                    atual = sp_inicial

                    repetir = input("\nDeseja fazer outro ensaio? (s/n): ").lower()
                    if repetir != "s":
                        tunel = input("\nDesligue o túnel. Desligado? (s/n): ").lower()
                        while tunel != "s":
                            tunel = input("Desligue o túnel. Desligado? (s/n): ").lower()
                        break

                print("\nEncerrando... aplicando rampa até 0 rpm")
                aplicar_rampa(ser, atual, 0)
                ser.write(b"T0\n")

            elif op == "3":
                print("Saindo...")
                break

            else:
                print("Opção inválida.")


if __name__ == "__main__":
    main()
