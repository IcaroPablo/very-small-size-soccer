#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Script para definicao da estrategia que deve ser usada pelo timfrom scipy.spatial import distancee
durante a partida.
"""
from scipy.spatial import distance
from esquemaEstrategias import *

class Arena:
    def __init__(self):
        self.coordGolAmigo = [485, 255] # cte. coordenanda central do gol amigo.
        self.coordGolInimigo = [15, 255] # cte. coordenanda central do gol inimigo.
        self.coordPontosDeFalta = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        self.coordPontosDePenalti = [0, 0] # sera alterado pela interface grafica
        self.coordQuinaSupGol = [[0, 0], [0, 0]] # sera alterado pela interface grafica
        self.coordQuinaInfGol = ((0, 0), (0, 0)) # sera alterado pela interface grafica
        self.coordMeioDeCampo = [256, 256]

    def atualizacaoArena(self):
        """
        Funcao para modificar quaisquer parametros do campo.
        """
        print "modificando arena"


class Tatica:
    """
    Classe instanciada no main.py
    """
    def __init__(self, b, f, e, cp):
        self.b = b # objeto relacionado a bola, que é recebido
        self.f = f
        self.e = e
        self.cp = cp
        self.quadranteDaBola = (0, 0) # quadrante de localizacao da bola (x, y)
        self.coordQuadrante = (0, 0) # cte. coordenanda central do quadrante
        self.quadranteRobosAmigos = ((0, 0), (0, 0), (0, 0))
        self.posseDeBola = None # pode ser: amigo, inimigo e ninguem
        self.indiceAmigoBola = 0
        self.velocidadeBola = (0, 0) # (vx, vy)
        self.velocidadeInimigo = ((0, 0), (0, 0), (0, 0)) # ((v1_x, v1_y), (v2_x, v2_y), (v3_x, v3_y))
        self.estrategias = Estrategias(self.b, self.f, self.e, self.cp)
        self.arena = Arena()


    def atualizaTatica(self):
        """
        Função chamada continuamente no main.py, para a atualização da estratégia do time.
        """
        # self.quadranteDaBola = quadranteDaBola
        # self.coordQuadrante = coordQuadrante
        self.__defineEstrategia()

    def __defineEstrategia(self):
        """
        Função chamada na função atualizaTatica(), que define a estratégia a ser usada pelo time
        e chamada do script esquemaEstrategias.py.
        """
        self.__verificaPosseBola()

        self.f[self.indiceAmigoBola].angGolInimigo = self.__calculaAngulo(
                                                                          self.f[self.indiceAmigoBola].pose[0],
                                                                          self.f[self.indiceAmigoBola].pose[1],
                                                                          self.arena.coordGolInimigo[0],
                                                                          self.arena.coordGolInimigo[1],
                                                                          )
        # angulo do robô selecionado em relação ao gol inimigo.
        ang  = self.f[self.indiceAmigoBola].angGolInimigo
        # print ang

        self.estrategias.moveQuad(0, [1, 1])
        self.estrategias.moveGoleiro()

        '''
        if(self.posseDeBola == "amigo"):
            # (ang >= 0 or ang <= 180) and
            if(self.f[self.indiceAmigoBola].distGolInimigo > 20):
                self.estrategias.rumoAoGol(self.arena, self.indiceAmigoBola)
                # print "parte1"
        else:
            # print "parte2"
            indice = self.b.indice
            theta = self.f[self.b.indice].pose[2] - self.f[self.b.indice].ang
            self.estrategias.movimentacao(self.b.indice, theta)
        '''

    def __calculaDistancia(self, x1, y1, x2, y2):
        """
        Função para o calculo da distancia euclidiana entre dois pontos.
        Retornos
        ----------
        o valor da distancia.
        """
        return distance.euclidean((x1, y1), (x2, y2))

    def __alocacaoDePapeis(self):
        """
        Função a ser implementada corretamente.
        """
        distanciaGol = (0, 0, 0)
        for i in range(0, 3):
            distanciaGol = self.__calculaDistancia(
                                                   self.f[i].x, 
                                                   self.f[i].y, 
                                                   self.arena.coordGolInimigo[0],
                                                   self.arena.coordGolInimigo[1]
                                                   ) 
            if (self.posseDeBola == "amigo"):
                self.f[indiceAmigoBola].funcao == "atacante"

    def __verificaPosseBola(self):
        """
        Analisa se os robôs aliados tem a posse da bola, feito
        isso atraves da distancia dos robôs em relacao a bola.
        Também especifica, através de um indice, o robô amigo que 
        está com a bola.
        """
        # limiar de distância da bola em relação ao robô, para indicar
        # posse de bola.
        limiar = 45
        for i in range(0, 3):
            if(self.f[i].dA <= limiar):
                self.posseDeBola = "amigo"
                self.indiceAmigoBola = i
                self.f[i].distGolInimigo = self.__calculaDistancia(self.arena.coordGolInimigo[0],
                                                                   self.arena.coordGolInimigo[1],
                                                                   self.f[i].pose[0],
                                                                   self.f[i].pose[1]
                                                                   )
                break
            else:
                self.posseDeBola = "inimigo"

    def __calculaAngulo(self, xb, yb, xr, yr):
        """
        Calculo de ângulo de forma genérica, considerando que o valor de y aumenta
        no decorrer da imagem.

        Parâmetros
        ----------
        xb : int
            posicão x da bola.
        yb : int
            posicão y da bola
        xr : int
            posicão x do robô
        yr : int
            posicão y do robô

        Retornos
        ----------
        int
            Retorna o ângulo entre os pontos (xb, yb) e (xr, yr)
        """
        y, x = -1*(yb-yr), xb-xr
        if((x > 0 and y > 0) or (x < 0 and y > 0)):
            return np.degrees(np.arctan2(y, x))
        elif((x < 0 and y < 0) or (x > 0 and y < 0)):
            return np.degrees(np.arctan2(y, x))+360
        elif(x == 0):
            if(y >= 0):
                return 90
            else:
                return 270
        elif(y == 0):
            if(x >= 0):
                return 0
            else:
                return 180

    def densidadeInimigos(self, QuadDen):
        """
            Descricao:
                Função que aponta quais quadrantes possuem mais de um robô
            Ultilizacao:
                self.densidadeInimigos(QuadDen)
            Parametros:
                1) QuadDen - vetor instanciado em main.py contendo 
                os quadrantes onde os robos estão posicionados
            Retornos:
                vetorDen [mda, mdi, mdg] - o primeiro elemento do vetorDen indica o quadrante 
                onde há maior densidade de amigos; o segundo elemento do vetorDen indica o quadrante
                onde há maior densidade de inimigos; o terceiro elemento do vetorDen é um vetor
                que possui como elementos os quadrantes onde há maior densidade de robôs no geral,
                independente de serem inimigos ou amigos
        """
        self.QuadDen = QuadDen
        mdi = 0 # mdi: maior densidade de inimigos
        mda = 0 # mda: maior densidade de amigos
        mdg = [0] # mdg: maior densidade de robos no geral
        
        # para a densidade de inimigos
        for d in range(4,6):
            if self.QuadDen[3] == self.QuadDen[d]:
                mdi = self.QuadDen[d]  

        # para a densidade de amigos
        for d in range(1,3):
            if self.QuadDen[0] == self.QuadDen[d]:
                mda = self.QuadDen[d]  

        # para a densidade de robos no geral        
        for d in range(0, 5):
            for d2 in range(d+1, 6):
                if self.QuadDen[d] == self.QuadDen[d2]:
                    if mdg[0] == 0:
                        mdg[0] = self.QuadDen[d]
                    else:
                        mdg.append(self.QuadDen[d])

        vetorDen = [mda, mdi, mdg] 
        return vetorDen
