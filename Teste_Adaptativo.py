#!/usr/bin/env pybricks-micropython

"""
Avaliação Somativa 7 - PROGRAMA DE TESTES (não precisa do Servidor)
--------------------------------------------------------------------

Usa as MESMAS funções e parâmetros do Adaptativo.py (importa o arquivo),
então tudo o que for ajustado lá vale aqui e vice-versa.
O Adaptativo.py precisa estar na MESMA pasta deste arquivo no EV3.

Menu:
  CIMA   : sequência completa (seguidor + ações após a parede)
  ESQ    : só o seguidor de linha (para na parede)
  DIR    : só as ações após a parede (gira, 500 mm, gira, 1800 mm)
  BAIXO  : alterna P ADAPTATIVO / P CONVENCIONAL (comparativo)
  CENTRO : recalibrar o sensor de cor
Durante o seguidor, qualquer botão interrompe o teste.

Cada teste do seguidor grava um CSV (log_adapt_*.csv ou log_conv_*.csv)
com tempo, reflexão, erro, kP_atual e correção, para os gráficos.
"""

from pybricks.parameters import Button
from pybricks.tools import wait

import Adaptativo as R


modo_adaptativo = R.ADAPTATIVO


def nome_modo():
    return 'ADAPT' if modo_adaptativo else 'CONV'


def esperar_centro():
    R.esperar_soltar()
    while Button.CENTER not in R.ev3.buttons.pressed():
        wait(20)
    R.esperar_soltar()


def largada():
    """Espera soltar o botão e dá 1 s para tirar a mão do robô."""
    R.esperar_soltar()
    R.tela(['Largando...'])
    R.ev3.speaker.beep(700, 80)
    wait(1000)


def rodar_seguidor():
    tempo, log, motivo = R.seguir_linha(adaptativo=modo_adaptativo,
                                        abortavel=True)
    wait(300)                       # robô assentado antes de medir
    dist = R.us.distance()
    prefixo = 'log_adapt' if modo_adaptativo else 'log_conv'
    nome = R.salvar_log(log, prefixo)
    if log:
        erros = [abs(linha[2]) for linha in log]
        med = sum(erros) / len(erros)
        sat = 100.0 * sum(1 for x in erros if x >= 25) / len(erros)
        kp_txt = '|e| %.1f sat %d%%' % (med, sat)   # menores = segue melhor
    else:
        kp_txt = '|e| -'
    return tempo, dist, motivo, nome, len(log), kp_txt


def teste_seguidor():
    largada()
    tempo, dist, motivo, nome, n, kp_txt = rodar_seguidor()
    R.ev3.speaker.beep(1000, 100)
    R.tela(['SEGUIDOR %s' % nome_modo(),
            'fim: %s' % motivo,
            'tempo: %.2f s' % (tempo / 1000.0),
            'parede: %d mm' % dist,
            kp_txt,
            nome,
            'CENTRO: voltar'])
    esperar_centro()


def teste_acoes():
    largada()
    e1, e2, s, motivo = R.acoes_pos_parede(
        ao_girar=lambda: R.ev3.speaker.beep(1200, 60))   # = envio do "done"
    R.ev3.speaker.beep(1000, 100)
    R.tela(['ACOES POS-PAREDE',
            'erro giro1: %.1f' % e1,
            'erro giro2: %.1f' % e2,
            'reta: %d/%d mm' % (s, R.DIST_2),
            'fim reta: %s' % motivo,
            'CENTRO: voltar'])
    esperar_centro()


def teste_completo():
    largada()
    tempo, dist, motivo, nome, n, kp_txt = rodar_seguidor()
    if motivo != 'parede':
        R.tela(['Seguidor abortado', 'CENTRO: voltar'])
        esperar_centro()
        return
    e1, e2, s, motivo2 = R.acoes_pos_parede(
        ao_girar=lambda: R.ev3.speaker.beep(1200, 60))
    R.ev3.speaker.beep(1000, 100)
    R.tela(['COMPLETO %s' % nome_modo(),
            'linha: %.2f s' % (tempo / 1000.0),
            'parede: %d mm' % dist,
            'giros: %.1f %.1f' % (e1, e2),
            'reta: %d mm %s' % (s, motivo2),
            nome,
            'CENTRO: voltar'])
    esperar_centro()


# ------------------------------------------------------------------ menu

R.calibrar()

while True:
    R.tela(['TESTES (sem servidor)',
            'CIMA : completo',
            'ESQ  : so seguidor',
            'DIR  : so acoes',
            'BAIXO: P %s' % nome_modo(),
            'CENTRO: calibrar',
            'refl %d  US %d' % (R.cor.reflection(), R.us.distance()),
            'V %d acel %s' % (R.V_BASE, R.ACEL_USADA)])

    b = R.ev3.buttons.pressed()
    if Button.UP in b:
        teste_completo()
    elif Button.LEFT in b:
        teste_seguidor()
    elif Button.RIGHT in b:
        teste_acoes()
    elif Button.DOWN in b:
        modo_adaptativo = not modo_adaptativo
        R.ev3.speaker.beep(800, 60)
        R.esperar_soltar()
    elif Button.CENTER in b:
        R.calibrar()

    wait(100)