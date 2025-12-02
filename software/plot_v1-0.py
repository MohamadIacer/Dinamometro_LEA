import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# === CONFIGURAÇÕES CALIBRAÇÃO===
ARQUIVOS = [
    "calibracao_samples_20251129_190617.txt", #TXT para usar na calibração (quantos quiser)
    "calibracao_samples_20251129_233432.txt"
] 

COL_MASSA = "Massa[g]"
COL_LEITURA = "Leitura[int]"
BRAÇO_MM = 26 + 15       # mm → torque
# === CONFIGURAÇÕES ENSAIO===
D_rotor = 22e-2          # metros
V_vento = 7.0            # m/s
rho = 1.225              # kg/m³

coef_load = [0.004075/1000, 945.025/1000]
coef_freio = [11.394/1000, -1.577/1000]             #ignorar isso aqui

R_rotor = D_rotor/2

ARQUIVO = "aquisicao_20251119_173920.txt"            #TXT do ensaio

# ============================================================
# FUNÇÕES

def curva_de_calibracao(arquivos, col_massa, col_leitura, braco_mm):
    todos_dados = []
    todos_medios = []

    for arquivo in arquivos:

        # === LEITURA ===
        try:
            df = pd.read_csv(arquivo, sep="\t")
        except:
            df = pd.read_csv(arquivo, sep=",")

        # renomeia para garantir padronização
        df = df.rename(columns={
            df.columns[0]: col_massa,
            df.columns[1]: "Torque_original_ignore",
            df.columns[2]: col_leitura
        })

        df = df.dropna()

        # === CÁLCULO DO TORQUE USANDO A MASSA ===
        df["Torque[N.mm]"] = df[col_massa] / 1000 * 9.81 * braco_mm

        # === REMOVE OUTLIERS NA LEITURA ===
        Q1 = df[col_leitura].quantile(0.25)
        Q3 = df[col_leitura].quantile(0.75)
        IQR = Q3 - Q1
        df = df[(df[col_leitura] >= Q1 - 2*IQR) &
                (df[col_leitura] <= Q3 + 2*IQR)]

        # === AGRUPA POR MASSA ===
        agrupado = df.groupby(col_massa, as_index=False).agg({
            col_leitura: "mean",
            "Torque[N.mm]": "mean"
        })

        todos_dados.append(df[[col_leitura, "Torque[N.mm]"]])
        todos_medios.append(agrupado[[col_leitura, "Torque[N.mm]"]])

    # ================================================================
    # CURVA DE CALIBRAÇÃO FINAL
    # ================================================================
    dados = pd.concat(todos_dados, ignore_index=True)
    medios = pd.concat(todos_medios, ignore_index=True)

    x = medios[col_leitura].values
    y = medios["Torque[N.mm]"].values

    a, b = np.polyfit(x, y, 1)
    y_pred = a * x + b
    r2 = 1 - np.sum((y - y_pred)**2) / np.sum((y - y.mean())**2)

    # ================================================================
    # PLOT
    # ================================================================
    plt.figure(figsize=(8, 6))

    plt.scatter(dados[col_leitura], dados["Torque[N.mm]"],
                s=10, alpha=0.2, label="Dados brutos")

    plt.scatter(medios[col_leitura], medios["Torque[N.mm]"],
                s=50, edgecolor="k", label="Médias por massa")

    x_plot = np.linspace(min(x), max(x), 200)
    plt.plot(x_plot, a * x_plot + b, linewidth=3,
             label=f"T = {a:.6f}·ADC + {b:.6f}\nR²={r2:.5f}")

    plt.xlabel("Leitura ADC [inteiro]")
    plt.ylabel("Torque [N·mm]")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.show()

    print(f"Curva final: T = {a:.6f}·ADC + {b:.6f}")
    print(f"R² = {r2:.5f}")

    # retorna se quiser usar depois
    return a, b, r2, dados, medios

# ============================================================
# MENU PRINCIPAL
# ============================================================
def moving_average(signal, window):
    pad = window // 2
    padded = np.pad(signal, pad, mode='reflect')
    kernel = np.ones(window)/window
    return np.convolve(padded, kernel, mode='valid')

def media_por_patamar(df, coluna_patamar, coluna_valor):
    """
    df: dataframe contendo os dados
    coluna_patamar: nome da coluna que indica o patamar (ex: 'omega_set')
    coluna_valor: coluna do sinal que você quer agrupar (ex: 'torque')
    """
    return df.groupby(coluna_patamar)[coluna_valor].mean()

def aplicar_calibracao(signal, coeficientes):
    """
    Aplica uma calibração polinomial aos dados.
    
    signal: array-like com os dados brutos (ex: tensões)
    coeficientes: lista com coeficientes do polinômio
                  no formato do numpy (do termo mais alto ao mais baixo)

    Ex: coeficientes = [a, b, c] → a*x^2 + b*x + c
    """
    # Cria o polinômio a partir dos coeficientes
    poly = np.poly1d(coeficientes)
    
    # Aplica o polinômio
    return poly(signal)

def calcular_cp(omega, torque, rho, V, diametro):
    """
    Calcula o coeficiente de potência (Cp) de um rotor.

    omega: velocidade angular (rad/s)
    torque: torque (N·m)
    rho: densidade do ar (kg/m³)
    V: velocidade do escoamento (m/s)
    diametro: diâmetro do rotor (m)
    """
    
    # potência mecânica
    P = torque * omega  # W
    
    # área varrida
    A = np.pi * (diametro/2)**2
    
    # potência disponível no vento (Betz denominator)
    P_disp = 0.5 * rho * A * V**3
    
    # Cp
    Cp = P / P_disp
    
    return Cp


def calcular_tsr(omega, V, diametro, eps=1e-12):
    """
    Calcula o TSR (tip-speed ratio).

    Parameters
    ----------
    omega : array-like or scalar
        Velocidade angular em rad/s.
    V : array-like or scalar
        Velocidade do escoamento em m/s.
    diametro : float
        Diâmetro do rotor em metros.
    eps : float, optional
        Valor mínimo para V para evitar divisão por zero (default 1e-12).

    Returns
    -------
    tsr : numpy.ndarray or float
        TSR correspondente (mesma forma de omega/V).
    """
    omega = np.asarray(omega)
    V = np.asarray(V)
    R = diametro / 2.0

    # evita divisão por zero: substitui V muito pequenos por eps
    V_safe = np.where(np.abs(V) < eps, eps, V)

    tsr = (omega * R) / V_safe

    # opcional: se V for ~0, podemos querer marcar com nan ao invés de valor grande
    tsr = np.where(np.abs(V) < eps, np.nan, tsr)

    return tsr

def main():
    a = b = None   # coeficientes ainda não calculados

    while True:
        print("\n=== MENU ===")
        print("1 - Curva de calibração")
        print("2 - Curvas do ensaio")
        print("3 - Sair")
        op = input("Escolha: ")

        # -------------------------------------------------------------
        # 1) CURVA DE CALIBRAÇÃO
        # -------------------------------------------------------------
        if op == "1":
            a, b, r2, dados, medios = curva_de_calibracao(
                ARQUIVOS,
                COL_MASSA,
                COL_LEITURA,
                BRAÇO_MM
            )
            print("\nCoeficientes obtidos:")
            print(f"a = {a:.6f}, b = {b:.6f}")

        # -------------------------------------------------------------
        # 2) CURVAS DO ENSAIO
        # -------------------------------------------------------------
        elif op == "2":

            print("\nAperte enter para usar coeficientes da calibração ou insira coeficientes manuais (a,b) em N/mm por ADC.")
            inp_load = input("Coeficientes célula de carga (a,b) [N/mm por ADC]: ")

            # --- usar coeficientes da calibração
            if inp_load.strip() == "":
                if a is None or b is None:
                    print("\nERRO: Nenhuma calibração foi feita ainda!")
                    print("Vá para a opção 1 primeiro.")
                    continue
                coef_load = [a/1000, b/1000]  # converter para N.m
                print(f"Usando coeficientes da calibração: a={a:.6f}, b={b:.6f}")

            else:
                # --- interpretar coeficientes manuais
                try:
                    parts = inp_load.split(",")
                    if len(parts) != 2:
                        raise Exception
                    a_manual = float(parts[0].strip())
                    b_manual = float(parts[1].strip())
                    coef_load = [a_manual/1000, b_manual/1000]  # converter para N.m
                    print(f"Usando coeficientes manuais: a={a_manual}, b={b_manual}")
                except:
                    print("Entrada inválida. Use o formato: 0.000123, -0.4567")
                    continue



            df = pd.read_csv(ARQUIVO, sep="\t")

            set_omega = df.iloc[:, 0]
            real_omega = df.iloc[:, 2]
            pos_rotor = df.iloc[:, 3]
            acc_x = df.iloc[:, 4]
            acc_y = df.iloc[:, 5]
            acc_z = df.iloc[:, 6]
            V_load = df.iloc[:, 7]
            V_freio = df.iloc[:, 8]


            #USAR VALOR ÍMPAR PARA A JANELA. GARANTE RETORNO COM MESMA QUNATIDADE DE ELEMENTOS.
            V_load_avg = moving_average(V_load, 201)
            V_freio_avg = moving_average(V_freio, 201)
            real_omega_avg = moving_average(real_omega, 301)

            df["set_omega"] = set_omega
            df["V_load_avg"] = V_load_avg
            df["real_omega_avg"] = real_omega_avg
            df["V_freio_avg"] = V_freio_avg

            medias_V_load_omega = media_por_patamar(df, 'set_omega', 'V_load_avg')
            medias_torque_load_omega = aplicar_calibracao(medias_V_load_omega, coef_load)

            medias_V_freio_omega = media_por_patamar(df, 'set_omega', 'V_freio_avg')
            medias_torque_freio_omega = aplicar_calibracao(medias_V_freio_omega, coef_freio)

            medias_real_omega = media_por_patamar(df, 'set_omega', 'real_omega_avg')

            cp_load = calcular_cp(medias_real_omega, medias_torque_load_omega, rho, V_vento, D_rotor)
            #cp_freio = calcular_cp(medias_real_omega, medias_torque_freio_omega, rho, V_vento, D_rotor)

            tsr = calcular_tsr(medias_real_omega,V_vento, D_rotor)



            plt.figure(figsize=(14, 4))
            plt.xlabel("Amostras")
            plt.ylabel("Velocidade angular (rad/s)")
            plt.title("Velocidade angular - Setpoint vs Real (média móvel)")
            plt.plot(set_omega, label="Setpoint")
            plt.plot(real_omega_avg, label="Real (média móvel)")
            plt.grid()
            plt.legend()
            plt.tight_layout()

            plt.figure(figsize=(14, 4))
            plt.scatter(range(len(pos_rotor)), pos_rotor % (2*np.pi), s=1)
            plt.gca().set_yticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
            plt.yticks(
                [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
                [r'0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$']
            )
            plt.xlabel("Amostras")
            plt.ylabel("Posição angular do rotor (rad)")
            plt.title("Posição angular do rotor")
            plt.grid()
            plt.tight_layout()

            plt.figure(figsize=(14, 4))
            plt.xlabel("Amostras")
            plt.ylabel("ADC bruto (bits)")
            plt.title("Medida célula de carga ADS1256 - Média móvel")
            plt.plot(V_load)
            plt.plot(V_load_avg)
            plt.grid()
            plt.tight_layout()

            plt.figure()
            plt.xlabel("Amostras")
            plt.ylabel("Tensão de freio (Volts)")
            plt.title("Medida tensão de freio - Média móvel")
            plt.plot(V_freio)
            plt.plot(V_freio_avg)
            plt.grid()
            plt.tight_layout()

            plt.figure()
            plt.title("Leitura média por patamar de velocidade angular - Carga")
            plt.xlabel("Velocidade angular (rad/s)")
            plt.ylabel("ADC bruto (bits)")
            plt.plot(medias_V_load_omega,"*-")
            plt.grid()
            plt.tight_layout()

            plt.figure()
            plt.title("Torque célula de carga ADS1256 - Média por patamar de velocidade angular")
            plt.xlabel("Velocidade angular (rad/s)")
            plt.ylabel("Torque (N.mm)")
            plt.plot(medias_torque_load_omega*1000,"*-")
            plt.grid()
            plt.tight_layout()


            plt.figure()
            plt.title("Tensão média por patamar de velocidade angular - Freio")
            plt.xlabel("Velocidade angular (rad/s)")
            plt.ylabel("Tensão de freio (Volts)")
            plt.plot(medias_V_freio_omega,"*-")
            plt.grid()
            plt.tight_layout()

            plt.figure()
            plt.title("Torque do freio - Média por patamar de velocidade angular")
            plt.xlabel("Velocidade angular (rad/s)")
            plt.ylabel("Torque (N.mm)")
            plt.plot(-medias_torque_freio_omega*1000,"*-")
            plt.grid()
            plt.tight_layout()

            plt.figure()
            plt.title("Coeficiente de potência Cp vs TSR - Carga")
            plt.xlabel("TSR")
            plt.ylabel("Cp")
            plt.plot(tsr,cp_load,"*-")
            plt.grid()
            plt.tight_layout()

            plt.figure()
            plt.title("Coeficiente de potência Cp vs TSR - Comparação freio e célula de carga")
            plt.xlabel("TSR")
            plt.ylabel("Cp")
            plt.plot(tsr,-cp_freio,"*-", label="Freio")
            plt.plot(tsr,cp_load,"*-", label="Célula de carga")
            plt.grid()
            plt.legend()
            plt.tight_layout()


            plt.show()

        elif op == "3":
            print("Saindo...")
            break

        else:
            print("Opção inválida.")


if __name__ == "__main__":
    main()

