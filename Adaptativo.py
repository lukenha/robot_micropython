#!/usr/bin/env pybricks-micropython

"""
Avaliação Somativa 7 - Robótica Autônoma (PUCPR)
Seguidor de linha com controlador P-ADAPTATIVO + rede Servidor-Cliente
ROBÔ SERVIDOR (NUMERO_ROBO = 0)                  (arquivo de entrega)
----------------------------------------------------------------------

O mesmo arquivo serve para os dois papéis (constante NUMERO_ROBO):
  0       -> SERVIDOR (primeiro da fila)
  1, 2... -> CLIENTE  com esse número

Sequência do SERVIDOR (enunciado, item 2):
  0. Calibra, espera os NUM_CLIENTES se conectarem e espera CENTRO.
  1. Segue a linha (P adaptativo, velocidade máxima) até a parede a 300 mm.
  2. Para.  3. Gira 90°.  4. Envia "client_1: go" (libera o Cliente 1).
  5. Anda 500 mm.  6. Gira 90°.  7. Anda 2000 mm.  8. Para.
  9. Espera o último cliente terminar ("client_n: parked").
     Enquanto anda e espera, repassa a vez: ao receber "client_k: done"
     (ou "client_k: parked") envia "client_k+1: go".
 10. Sequência sonora: Servidor -> Cliente 1 ... Cliente n -> Servidor
     ("client_k: end" / resposta "client_k: end done").

Sequência do CLIENTE k (enunciado, item 3):
  espera "client_k: go" (responde "client_k: ack"), segue a linha até a
  parede, para, gira 90°, envia "client_k: done", anda 500 mm, gira 90°,
  anda (2000 - 250*k) mm, para, envia "client_k: parked"; no fim toca o
  som ao receber "client_k: end" e responde "client_k: end done".

Controlador P adaptativo (dica do enunciado):
  erro     = setpoint - reflexão
  kP_atual = kP_min + (kP_max - kP_min) * |erro| / erro_max
  correção = kP_atual * (V_BASE/V_REF) * erro   (mm/s em cada roda),
             limitada a +-CORR_MAX_FRAC * v
  roda_E   = v_base - LADO * correção
  roda_D   = v_base + LADO * correção
Erro e kP_atual são registrados num CSV no EV3 para os gráficos.

Este arquivo também é importado pelo programa de testes (Teste_Adaptativo.py):
a conexão Bluetooth só é feita quando ESTE arquivo é executado.

Antes de rodar:
  - Bricks pareados por Bluetooth (sem conectar). O SERVIDOR é iniciado
    antes dos clientes; os clientes usam o nome deste brick no SERVER_NAME.
  - Ajuste NUMERO_ROBO e NUM_CLIENTES.
  - Robô parado sobre a linha, com o sensor de cor na borda da linha.

Requer LEGO EV3 MicroPython v2.0.
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button, Stop
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile
from pybricks.messaging import (BluetoothMailboxClient, BluetoothMailboxServer,
                                TextMailbox)

import math
import utime


# =====================================================================
# 1. CONFIGURAÇÃO
# =====================================================================

# --- Rede (combinar com a turma) ---
NUMERO_ROBO = 2                 # 0 = SERVIDOR; 1, 2, ... = Cliente com esse nº
NUM_CLIENTES = 1                # (Servidor) quantos clientes vão se conectar
MAILBOX_NAME = 'greeting'       # mesmo nome de caixa em todos os robôs
SERVER_NAME = 'ev3dev'          # (Cliente) nome Bluetooth do brick Servidor
T_RESPOSTA_SOM = 6000           # ms: (Servidor) espera máx. pelo som de cada
                                # cliente no fim, para um robô travado não
                                # parar a sequência

CLIENT_ID = 'client_%d' % NUMERO_ROBO
MSG_GO = CLIENT_ID + ': go'             # Servidor -> Cliente: iniciar
MSG_ACK = CLIENT_ID + ': ack'           # Cliente -> Servidor: "go" recebido
MSG_DONE = CLIENT_ID + ': done'         # Cliente -> Servidor: parou e girou 90°
MSG_PARKED = CLIENT_ID + ': parked'     # Cliente -> Servidor: terminou de andar
MSG_END = CLIENT_ID + ': end'           # Servidor -> Cliente: som final
MSG_END_DONE = CLIENT_ID + ': end done' # Cliente -> Servidor: som final tocado

# --- Portas (mesma Base Motriz do Labirinto.py) ---
PORTA_MOTOR_ESQ = Port.A
PORTA_MOTOR_DIR = Port.D
PORTA_GIRO = Port.S3
PORTA_COR = Port.S2
PORTA_US = Port.S1              # ultrassônico fixo na frente

# --- Geometria (mm) ---
DIAM_RODA = 55.5
BITOLA = 104.0

# --- Seguidor de linha: calibração (sobrescrita pela tela de calibração) ---
PRETO_PADRAO = 7                # reflexão (%) sobre o preto
BRANCO_PADRAO = 60              # reflexão (%) sobre o branco
AJUSTE_SETPOINT = 0             # ajuste fino somado a (preto + branco) / 2
ARQ_CALIB = 'calib_linha.txt'

# --- Seguidor de linha: controlador ---
LADO_LINHA = 1                  # 1: segue a borda ESQUERDA da faixa preta
                                # -1: segue a borda DIREITA
                                # (se o robô fugir da linha logo ao sair, inverta)
ADAPTATIVO = True               # False = P convencional (para o comparativo)
V_BASE = 130.0                  # mm/s velocidade base (subir até o limite)
KP_MIN = 0.4                    # (mm/s)/% ganho com erro ~0 (reta: suave)
KP_MAX = 3.0                    # (mm/s)/% ganho com erro máximo: 2,5*40 = 100,
                                # a correção chega no limite só perto do erro máx.
KP_FIXO = 1.8                   # (mm/s)/% ganho do P convencional
V_REF_GANHO = 100.0             # mm/s em que os kP acima foram sintonizados:
                                # o kP é multiplicado por V_BASE / V_REF_GANHO,
                                # então ao subir V_BASE a curva feita para o
                                # mesmo erro continua igual (corrige na proporção)
CORR_MAX_FRAC = 1.0             # |correção| <= CORR_MAX_FRAC * v. Com 1.0 a roda
                                # de dentro no máximo PARA; nunca gira para trás
                                # (sem isso o robô girava no lugar e cruzava a linha)
REDUCAO_CURVA = 0.3             # 0 = desligado; 0.3 = reduz até 30% da
                                # velocidade numa curva (erro sustentado)
FILTRO_CURVA = 0.3              # s: a redução usa o erro FILTRADO (média móvel
                                # com sinal). Na oscilação da reta o erro vai e
                                # volta e a média fica ~0 (não freia); na curva
                                # o erro fica de um lado só (freia).
LOOP_MS = 2                     # período do laço de controle
LOG_MS = 20                     # período do registro (erro, kP)

# --- Parede ---
DIST_PAREDE = 300               # mm
T_PARADA = 0.10                 # s: detecta em 300 + v*T_PARADA para compensar
                                # a frenagem e PARAR perto dos 300 mm
CONFIRMA_US = 2                 # leituras seguidas abaixo do limite

# --- Ações após a parede ---
GIRO_1 = 90                     # graus (+ = direita/horário)
GIRO_2 = 90                     # mesmo sentido: o robô volta paralelo à linha
DIST_1 = 500.0                  # mm
DIST_2 = 2000.0 - 300.0 * NUMERO_ROBO  # mm (2000 no Servidor, 1750 no Cliente 1)
DIST_SEGURANCA = 70             # mm: na reta final para antes se houver um
                                # robô à frente (fila curta entre clientes)

# --- Movimento (valores calibrados no Labirinto.py) ---
V_RETA = 280.0                  # mm/s
V_MIN = 45.0                    # mm/s
ACEL = 650.0                    # mm/s²
DESACEL = 500.0                 # mm/s²
T_ANTEC = 0.07                  # s: atraso das rodas ao frear
KP_RUMO = 6.0                   # (graus/s) por grau de desvio de rumo nas retas
V_RODA_MAX = 300.0              # mm/s por roda (~620 graus/s no motor)
ACEL_MOTOR = 4000               # graus/s²: aceleração máxima de cada motor.
                                # Quanto maior, mais rápido a roda atende a
                                # correção (menos atraso = menos zigue-zague).
                                # 0 = manter o padrão do Pybricks.

# --- Giro com giroscópio (valores do Labirinto.py) ---
W_MAX = 300.0                   # graus/s
ALFA_GIRO = 500.0               # graus/s²
KP_GIRO = 6.0                   # (graus/s)/grau no fim do giro
W_MIN = 20.0                    # graus/s
TOL_GIRO = 1.5                  # graus
GIRO_SENTIDO = 1                # 1 se o giroscópio aumenta girando p/ a direita
ESCALA_GIRO = 1.0               # copiar "escala_giro" do calib.txt do labirinto


# =====================================================================
# 2. HARDWARE
# =====================================================================

ev3 = EV3Brick()
motor_esq = Motor(PORTA_MOTOR_ESQ)
motor_dir = Motor(PORTA_MOTOR_DIR)
cor = ColorSensor(PORTA_COR)

try:
    giro = GyroSensor(PORTA_GIRO)
    giro.angle()                # ativa o modo ângulo com o robô parado
except OSError:
    giro = None                 # sem giroscópio: giros pelos encoders


def abrir_ultrassonico():
    """Ultrassônico na porta do manual (S4); se não estiver lá, procura."""
    portas = [PORTA_US] + [p for p in (Port.S1, Port.S2, Port.S3, Port.S4)
                           if p != PORTA_US]
    for p in portas:
        try:
            return UltrasonicSensor(p)
        except OSError:
            pass
    ev3.screen.print('ULTRASSONICO')
    ev3.screen.print('NAO ENCONTRADO!')
    ev3.speaker.beep(300, 800)
    raise OSError('Ultrassonico nao conectado')


us = abrir_ultrassonico()


def configurar_motores():
    """Sobe o limite de aceleração dos motores (só aumenta, nunca reduz).
    Retorna (padrão, usado) para conferência."""
    padrao = usado = None
    for m in (motor_esq, motor_dir):
        try:
            vel, acel, atu = m.control.limits()
            padrao = acel
            if ACEL_MOTOR and ACEL_MOTOR > acel:
                m.control.limits(vel, ACEL_MOTOR, atu)
            usado = m.control.limits()[1]
        except Exception as erro:
            print('limits nao alterado:', erro)
    print('Aceleracao dos motores: padrao', padrao, '-> usada', usado)
    return padrao, usado


ACEL_PADRAO, ACEL_USADA = configurar_motores()
MM_POR_GRAU = math.pi * DIAM_RODA / 360.0

# Função chamada a cada volta dos laços de giro e reta (o Servidor usa
# para atender a rede enquanto anda). None = nada.
tarefa_fundo = None

SONS_NUMERO = (SoundFile.ZERO, SoundFile.ONE, SoundFile.TWO, SoundFile.THREE,
               SoundFile.FOUR, SoundFile.FIVE, SoundFile.SIX, SoundFile.SEVEN,
               SoundFile.EIGHT, SoundFile.NINE, SoundFile.TEN)


# =====================================================================
# 3. UTILITÁRIOS
# =====================================================================

def sinal(x):
    return 1 if x > 0 else (-1 if x < 0 else 0)


def tela(linhas):
    ev3.screen.clear()
    for linha in linhas:
        ev3.screen.print(linha)


def esperar_soltar():
    while ev3.buttons.pressed():
        wait(10)


def aplicar(v, d):
    """
    Rodas: esquerda = v + d, direita = v - d (mm/s).
    Se passar do limite do motor, reduz v e mantém a diferença d
    (prioridade para a correção - lição do labirinto).
    """
    if abs(d) > V_RODA_MAX:
        d = sinal(d) * V_RODA_MAX
    if abs(v) + abs(d) > V_RODA_MAX:
        v = sinal(v) * (V_RODA_MAX - abs(d))
    motor_esq.run((v + d) / MM_POR_GRAU)
    motor_dir.run((v - d) / MM_POR_GRAU)


def aplicar_w(v, w):
    """v em mm/s, w em graus/s (+ = horário/direita)."""
    aplicar(v, w * math.pi / 180.0 * BITOLA / 2.0)


def parar():
    motor_esq.hold()
    motor_dir.hold()


# =====================================================================
# 4. CALIBRAÇÃO DO SENSOR DE COR
# =====================================================================

CAL = {'preto': PRETO_PADRAO, 'branco': BRANCO_PADRAO,
       'ajuste': AJUSTE_SETPOINT}


def setpoint():
    return (CAL['preto'] + CAL['branco']) / 2.0 + CAL['ajuste']


def erro_max():
    sp = setpoint()
    return max(sp - CAL['preto'], CAL['branco'] - sp, 1.0)


def carregar_calib():
    try:
        with open(ARQ_CALIB) as f:
            for linha in f:
                if '=' in linha:
                    k, v = linha.strip().split('=')
                    if k in CAL:
                        CAL[k] = float(v)
    except OSError:
        pass


def salvar_calib():
    try:
        with open(ARQ_CALIB, 'w') as f:
            for k in CAL:
                f.write('%s=%s\n' % (k, CAL[k]))
    except OSError:
        pass


def calibrar():
    """
    Tela de calibração (valores salvos de antes já vêm preenchidos):
      ESQ   : grava PRETO  (sensor sobre a faixa preta)
      DIR   : grava BRANCO (sensor sobre o branco)
      CIMA / BAIXO : ajuste fino do setpoint (+1 / -1)
      CENTRO: confirma
    """
    carregar_calib()
    esperar_soltar()
    while True:
        r = cor.reflection()
        b = ev3.buttons.pressed()
        if Button.LEFT in b:
            CAL['preto'] = r
            ev3.speaker.beep(500, 60)
        elif Button.RIGHT in b:
            CAL['branco'] = r
            ev3.speaker.beep(900, 60)
        elif Button.UP in b:
            CAL['ajuste'] += 1
        elif Button.DOWN in b:
            CAL['ajuste'] -= 1
        elif Button.CENTER in b:
            if CAL['branco'] - CAL['preto'] >= 10:
                break
            ev3.speaker.beep(300, 300)      # preto e branco muito próximos
        if b:
            esperar_soltar()
        tela(['CALIBRACAO LINHA',
              'refl: %d' % r,
              'E preto : %d' % CAL['preto'],
              'D branco: %d' % CAL['branco'],
              'setpoint: %.1f' % setpoint(),
              'C/B ajuste: %+d' % CAL['ajuste'],
              'CENTRO: ok'])
        wait(60)
    esperar_soltar()
    salvar_calib()


# =====================================================================
# 5. SEGUIDOR DE LINHA P ADAPTATIVO
# =====================================================================

def seguir_linha(adaptativo=None, registrar=True, abortavel=False):
    """
    Segue a linha até a parede (DIST_PAREDE). Retorna (tempo_ms, log, motivo).
    log = lista de (tempo_ms, reflexão, erro, kP_atual, correção).
    motivo = 'parede' ou 'abortado' (botão, só se abortavel=True).
    """
    if adaptativo is None:
        adaptativo = ADAPTATIVO
    sp = setpoint()
    emax = erro_max()
    escala_v = V_BASE / V_REF_GANHO
    erro_filt = 0.0
    t_ant = 0
    log = []
    abaixo = 0
    n = 0
    motivo = 'parede'
    t_log = -LOG_MS
    relogio = StopWatch()

    while True:
        t = relogio.time()
        refl = cor.reflection()
        erro = sp - refl
        frac = min(abs(erro) / emax, 1.0)

        if adaptativo:
            kp = KP_MIN + (KP_MAX - KP_MIN) * frac
        else:
            kp = KP_FIXO
        kp = kp * escala_v                      # mesma geometria em qualquer V

        # Velocidade: só reduz com erro sustentado (curva), não na oscilação
        alfa = min(1.0, (t - t_ant) / (1000.0 * FILTRO_CURVA))
        t_ant = t
        erro_filt += alfa * (erro - erro_filt)
        v = V_BASE * (1.0 - REDUCAO_CURVA * min(abs(erro_filt) / emax, 1.0))
        correcao = kp * erro
        corr_max = CORR_MAX_FRAC * v
        if correcao > corr_max:
            correcao = corr_max
        elif correcao < -corr_max:
            correcao = -corr_max
        aplicar(v, -LADO_LINHA * correcao)

        # Parede à frente (compensando a distância de frenagem)
        if us.distance() < DIST_PAREDE + v * T_PARADA:
            abaixo += 1
            if abaixo >= CONFIRMA_US:
                break
        else:
            abaixo = 0

        if registrar and t - t_log >= LOG_MS:
            log.append((t, refl, erro, kp, correcao))
            t_log = t

        n += 1
        if abortavel and n % 20 == 0 and ev3.buttons.pressed():
            motivo = 'abortado'
            break

        wait(LOOP_MS)

    parar()
    return relogio.time(), log, motivo


def salvar_log(log, prefixo):
    """Grava o log em CSV na pasta do programa no EV3. Retorna o nome."""
    nome = '%s_%d.csv' % (prefixo, int(utime.time()))
    try:
        with open(nome, 'w') as f:
            f.write('tempo_ms,reflexao,erro,kp,correcao\n')
            for linha in log:
                f.write('%d,%d,%.2f,%.3f,%.1f\n' % linha)
    except OSError:
        return 'ERRO AO SALVAR'
    return nome


# =====================================================================
# 6. MOVIMENTOS APÓS A PAREDE
# =====================================================================

def girar_encoder(delta):
    """Giro no lugar só pelos encoders (se não houver giroscópio)."""
    ang = delta * BITOLA / DIAM_RODA
    motor_esq.run_angle(400, ang, Stop.HOLD, False)
    motor_dir.run_angle(400, -ang, Stop.HOLD, True)
    return 0.0


def girar(delta):
    """
    Gira delta graus no lugar (+ = direita) com o giroscópio: acelera até
    W_MAX, freia pela raiz do erro e termina com trecho linear (perfil do
    labirinto). Retorna o erro final em graus.
    """
    if giro is None:
        return girar_encoder(delta)

    giro.reset_angle(0)
    relogio = StopWatch()
    t_lim = 1500 + 12 * abs(delta)
    t_ant = 0
    w = 0.0
    estavel = 0
    erro = delta

    while True:
        t = relogio.time()
        h = GIRO_SENTIDO * ESCALA_GIRO * giro.angle()
        erro = delta - h

        if abs(erro) <= TOL_GIRO:
            parar()
            w = 0.0
            estavel += 1
            if estavel >= 2:
                break
            if tarefa_fundo is not None:
                tarefa_fundo()
            wait(LOOP_MS)
            continue
        estavel = 0

        if t > t_lim:
            break

        dt = max(1, t - t_ant) / 1000.0
        t_ant = t
        w_alvo = min(W_MAX, math.sqrt(2.0 * ALFA_GIRO * abs(erro)),
                     KP_GIRO * abs(erro))
        w_alvo = max(W_MIN, w_alvo)
        if w_alvo < w:
            w = w_alvo
        else:
            w = min(w_alvo, w + ALFA_GIRO * dt)

        aplicar_w(0, sinal(erro) * w)
        if tarefa_fundo is not None:
            tarefa_fundo()
        wait(LOOP_MS)

    parar()
    return erro


def reto(dist, v_max=None, seguranca=None):
    """
    Anda dist mm em linha reta (rampa de aceleração/frenagem, rumo mantido
    pelo giroscópio). Com 'seguranca' (mm), para antes se o ultrassônico
    vir algo mais perto que isso. Retorna (mm percorridos, motivo).
    """
    if v_max is None:
        v_max = V_RETA
    motor_esq.reset_angle(0)
    motor_dir.reset_angle(0)
    if giro is not None:
        giro.reset_angle(0)

    relogio = StopWatch()
    t_ant = 0
    v = 0.0
    abaixo = 0
    motivo = 'ok'
    s = 0.0

    while True:
        s = (motor_esq.angle() + motor_dir.angle()) / 2.0 * MM_POR_GRAU
        falta = dist - s
        if falta <= v * T_ANTEC:
            break

        if seguranca is not None:
            if us.distance() < seguranca:
                abaixo += 1
                if abaixo >= CONFIRMA_US:
                    motivo = 'obstaculo'
                    break
            else:
                abaixo = 0

        t = relogio.time()
        dt = max(1, t - t_ant) / 1000.0
        t_ant = t
        v_alvo = max(V_MIN, min(v_max, math.sqrt(2.0 * DESACEL * max(falta, 0))))
        if v_alvo < v:
            v = v_alvo
        else:
            v = min(v_alvo, v + ACEL * dt)

        w = 0.0
        if giro is not None:
            w = -KP_RUMO * GIRO_SENTIDO * ESCALA_GIRO * giro.angle()
        aplicar_w(v, w)
        if tarefa_fundo is not None:
            tarefa_fundo()
        wait(LOOP_MS)

    parar()
    return s, motivo


def acoes_pos_parede(ao_girar=None):
    """
    Itens 3 a 7 do enunciado. 'ao_girar' é chamado logo depois do 1º giro
    (no programa final: envia "client_1: done" ao Servidor).
    Retorna (erro_giro1, erro_giro2, mm_reta_final, motivo_reta_final).
    """
    e1 = girar(GIRO_1)
    if ao_girar is not None:
        ao_girar()
    reto(DIST_1)
    e2 = girar(GIRO_2)
    s, motivo = reto(DIST_2, seguranca=DIST_SEGURANCA)
    parar()
    return e1, e2, s, motivo


def som_final(numero=None):
    """Indicação sonora do robô: fala o número (0 = Servidor) e bipa."""
    if numero is None:
        numero = NUMERO_ROBO
    ev3.speaker.set_volume(100)
    if numero < len(SONS_NUMERO):
        ev3.speaker.play_file(SONS_NUMERO[numero])
    ev3.speaker.beep(1000, 300)


# =====================================================================
# 7. COMUNICAÇÃO
# =====================================================================

def esperar_resposta(mbox, esperada, timeout_ms):
    """(Servidor) Espera 'esperada' chegar, até timeout_ms. True se chegou."""
    relogio = StopWatch()
    while relogio.time() < timeout_ms:
        if mbox.read() == esperada:
            print('Recebido:', esperada)
            return True
        wait(20)
    print('Sem resposta:', esperada)
    return False


class RedeServidor:
    """
    Repasse da vez entre os clientes, sem bloquear o robô: processar() é
    chamado a cada volta dos laços de giro/reta e na espera final.

    A caixa do Servidor guarda só a ÚLTIMA mensagem (de qualquer cliente),
    então:
      - lê sem parar e trata cada mensagem nova uma vez;
      - "parked" também vale como "done": se o "done" de um cliente for
        sobrescrito, a vez passa no "parked" dele (atrasa, mas não trava).
    """

    def __init__(self, mbox, n_clientes):
        self.mbox = mbox
        self.n = n_clientes
        self.ultima = None
        self.vez = 0                 # cliente que está correndo agora
        self.estacionados = set()

    def enviar(self, msg):
        self.mbox.send(msg)
        print('Enviado:', msg)

    def liberar(self, k):
        self.enviar('client_%d: go' % k)
        self.vez = k

    def iniciar(self):
        """Item 3 do Servidor: libera o Cliente 1."""
        if self.n > 0:
            self.liberar(1)

    def processar(self):
        msg = self.mbox.read()
        if msg is None or msg == self.ultima:
            return
        self.ultima = msg
        print('Recebido:', msg)
        try:
            nome, acao = msg.split(': ', 1)
            k = int(nome.split('_')[1])
        except (ValueError, IndexError):
            return
        if acao == 'parked':
            self.estacionados.add(k)
        if acao in ('done', 'parked') and k == self.vez and k < self.n:
            self.liberar(k + 1)

    def terminou(self):
        return self.n == 0 or self.n in self.estacionados


def esperar_mensagem(mbox, esperadas):
    """Bloqueia até chegar uma mensagem NOVA que esteja em 'esperadas'
    (mensagens para outros clientes são ignoradas)."""
    while True:
        mbox.wait()
        msg = mbox.read()
        print('Recebido:', msg)
        if msg in esperadas:
            return msg


# =====================================================================
# 8. PROGRAMA FINAL
# =====================================================================

def main_cliente():
    calibrar()

    client = BluetoothMailboxClient()
    mbox = TextMailbox(MAILBOX_NAME, client)
    tela(['Conectando a', SERVER_NAME + '...'])
    client.connect(SERVER_NAME)
    tela(['Conectado!', 'Aguardando o', 'Servidor...'])
    ev3.speaker.beep(600, 100)

    while True:
        msg = esperar_mensagem(mbox, (MSG_GO, MSG_END))

        if msg == MSG_GO:
            # "ack" imediato: apaga do Servidor qualquer mensagem antiga
            mbox.send(MSG_ACK)

            tempo, log, _ = seguir_linha()              # 1. segue até a parede
            # 2-7. para, gira, avisa o Servidor, 500 mm, gira, 1800 mm, para
            e1, e2, s, motivo = acoes_pos_parede(
                ao_girar=lambda: mbox.send(MSG_DONE))
            mbox.send(MSG_PARKED)                       # 8. terminou

            nome = salvar_log(log, 'log_final')
            tela(['Linha: %.2f s' % (tempo / 1000.0),
                  'Giros: %.1f %.1f' % (e1, e2),
                  'Reta: %d mm %s' % (s, motivo),
                  nome,
                  'Aguardando fim...'])

        elif msg == MSG_END:
            som_final()                                 # 9. som do Cliente
            mbox.send(MSG_END_DONE)
            break

    tela(['Fim!'])
    wait(2000)


def main_servidor():
    global tarefa_fundo
    calibrar()

    server = BluetoothMailboxServer()
    mbox = TextMailbox(MAILBOX_NAME, server)
    rede = RedeServidor(mbox, NUM_CLIENTES)

    if NUM_CLIENTES > 0:
        tela(['SERVIDOR', 'Aguardando %d' % NUM_CLIENTES, 'cliente(s)...'])
        server.wait_for_connection(NUM_CLIENTES)
        ev3.speaker.beep(600, 100)

    tela(['SERVIDOR',
          '%d cliente(s) OK' % NUM_CLIENTES,
          '',
          'CENTRO: largar'])
    esperar_soltar()
    while Button.CENTER not in ev3.buttons.pressed():
        wait(20)
    esperar_soltar()
    wait(300)                                   # tirar a mão do robô

    tempo, log, _ = seguir_linha()              # 1. segue até a parede

    # 2-8. para, gira, libera o Cliente 1, 500 mm, gira, 2000 mm, para.
    # Durante os movimentos a rede é atendida (repasse da vez).
    tarefa_fundo = rede.processar
    e1, e2, s, motivo = acoes_pos_parede(ao_girar=rede.iniciar)

    # 9. espera o último cliente terminar (CENTRO pula a espera)
    tela(['Linha: %.2f s' % (tempo / 1000.0),
          'Giros: %.1f %.1f' % (e1, e2),
          'Reta: %d mm %s' % (s, motivo),
          'Esperando clientes',
          'CENTRO: pular'])
    while not rede.terminou():
        rede.processar()
        if Button.CENTER in ev3.buttons.pressed():
            break
        wait(20)
    tarefa_fundo = None

    # 10. sequência sonora: Servidor, Cliente 1 ... Cliente n, Servidor
    som_final(0)
    for k in range(1, NUM_CLIENTES + 1):
        rede.enviar('client_%d: end' % k)
        esperar_resposta(mbox, 'client_%d: end done' % k, T_RESPOSTA_SOM)
    som_final(0)
    ev3.speaker.play_file(SoundFile.GOOD_JOB)

    nome = salvar_log(log, 'log_final')
    tela(['Fim!', 'Linha: %.2f s' % (tempo / 1000.0), nome])
    wait(3000)


if __name__ == '__main__':
    if NUMERO_ROBO == 0:
        main_servidor()
    else:
        main_cliente()