#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Script para controle e comunicação com o vrep.
"""

import numpy as np
import vrep

class Estrategias:
	def __init__(self, b, f, e, cp):
		self.b = b
		self.f = f
		self.e = e
		self.cp = cp
		self.__inicializaMotores()
	
	def __inicializaMotores(self):
	    """
	    Inicializa objetos do vrep para o controle do movimento.
	    """
	    if self.cp.clientID !=-1:
	        # id de comunicacao do python com o vrep.
	        for i in range(0, 3):
	            code, self.f[i].mtrE = vrep.simxGetObjectHandle(self.cp.clientID, "mtrEsqFri" + str(i), vrep.simx_opmode_oneshot_wait)
	            code, self.f[i].mtrD = vrep.simxGetObjectHandle(self.cp.clientID, "mtrDirFri" + str(i), vrep.simx_opmode_oneshot_wait)
	            #code, e[i].mtrE = vrep.simxGetObjectHandle(clientID, ("mtrDirEn" + str(i+1)), vrep.simx_opmode_oneshot_wait)
	    else:
	        print "O arquivo da cena do VREP nao esta executando..."
	        vrep.simxFinish(self.cp.clientID)       # Now close the connection to V-REP:
	        quit()

	def __calculaAngulo(self, xb, yb, xr, yr):
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

	def movimentacao(self, indice, theta):
		# R - raio de rotacao do robo / l - distancia do centro de uma roda a outra.
		# ok---w, ve, vd, R, theta, l = 0, 0, 0, 2.0, 0.0, 15.0
		# w, ve, vd, R, l = 0, 0, 0, 2.0, 15.0
		w, ve, vd, R, l = 0, 0, 0, 2.0, 15.0
		self.cp.velocidade = self.b.velocidade + self.cp.velocidade/2
		# self.cp.velocidade = self.cp.velocidade + self.b.velocidade
		w = R*np.sin(np.deg2rad(theta))
		# velocidades a serem atribuidas as rodas esquerda e direita.
		ve = self.cp.velocidade - w*l/2.0
		vd = self.cp.velocidade + w*l/2.0
		for i in xrange(0, 3):
			if(i != indice):
				code = vrep.simxSetJointTargetVelocity(self.cp.clientID, self.f[i].mtrD, 0, vrep.simx_opmode_oneshot)
				code = vrep.simxSetJointTargetVelocity(self.cp.clientID, self.f[i].mtrE, 0, vrep.simx_opmode_oneshot)
			else:
				# ve = 0
				# vd = 0
				code = vrep.simxSetJointTargetVelocity(self.cp.clientID, self.f[i].mtrD, int(vd), vrep.simx_opmode_oneshot)
				code = vrep.simxSetJointTargetVelocity(self.cp.clientID, self.f[i].mtrE, int(ve), vrep.simx_opmode_oneshot)

	def rumoAoGol(self, arena, indice):
		theta = self.f[indice].angGolInimigo - self.f[indice].pose[2]
		self.cp.velocidade = 40 + self.b.velocidade
		self.movimentacao(indice, (theta))

	def moveSimples(self, robo, veld, vele):
		"""
			Descricao:
				Função que move as rodas do robô desejado com a velocidade desejada
			Ultilizacao:
				self.moveSimples(robo, veld, vele)
			Parametros:
				1) robo - número do robo aliado desejado int.
				2) veld - velocidade da roda direita int.
				3) vele - velocidade da roda esquerda int.
			Retornos:
				Sem retornos.
		"""
		self.veld = veld
		self.vele = vele

		vrep.simxSetJointTargetVelocity(self.cp.clientID, self.f[robo].mtrD, self.vele,  vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(self.cp.clientID, self.f[robo].mtrE, self.veld,  vrep.simx_opmode_oneshot)

	def giraRobo(self, robo, mini, maxi, atual):
		"""
			Descrição:
				Gira o robô aliado desejado para uma faixa de angulo desejada
			Ultilização:
				self.giraRobo(robo, mini, maxi, atual)
			Parametros:
				1) robo - número do robo aliado desejado int.
				2) mini - ângulo mínimo aceitável para a faixa de ângulo int.
				3) maxi - ângulo máximo aceitável para a faixa de ângulo int.
				4) atual - ângulo atual do robô (orientação) int.
			Retornos:
				Sem retornos.
		"""
		ref = (maxi + mini)/2
		dif = 0
		vel = 0
		sin = 1

		if atual < mini or atual > maxi:
			if atual > (ref + 180):	atual = atual - 360

			dif = atual - ref
			if abs(dif) > 40: vel = 20 * dif/abs(dif)
			if abs(dif) <= 40 and abs(dif) > 12: vel = dif/2
			if abs(dif) <= 5: vel = 0 

		self.moveSimples(robo, -vel, vel)

	def moveGoleiro(self):  # obs mudança de lado no segundo tempo, definir variáveis para as coordenadas
		"""
			Descricao:
				Função que move o goleiro para acompanhar a bola no eixo y no intervalo determinado.
			Ultilizacao:
				self.moveGoleiro()
			Parametros:
				Sem parâmetros
			Retornos:
				Sem retornos.
		"""
		vel = 0
		dif = 0
		ang = 0

		# fora de posição x
		if self.f[2].pose[0] < 433 or self.f[2].pose[0] > 474:
			ang = self.__calculaAngulo(453, 257, self.f[2].pose[0], self.f[2].pose[1])
			theta = self.f[2].pose[2] - ang
			self.movimentacao(2, theta)

		# fora do ângulo
		elif self.f[2].pose[2] < 80 or self.f[2].pose[2] > 100: self.giraRobo(2, 80, 100, self.f[2].pose[2])

		# movimentação (apoś correção da posição)
		else:
			dif = self.f[2].pose[1] - self.b.pose[1]  # diferença de posições em y

			# cálculo da velocidade
			if abs(dif) > 30: vel = ((dif/2) + self.b.velocidade * (dif/abs(dif)))
			elif abs(dif) > 10 and abs(dif) <= 30: vel = ((dif/3) + self.b.velocidade * (dif/abs(dif)))
			elif abs(dif) >= 0 and abs(dif) <= 10: vel = 0
			
			# robô acompanha a bola na frente da área do gol
			if self.b.pose[1] >= 200 and self.b.pose[1] <= 320:	self.moveSimples(2, vel, vel)
			else: self.moveSimples(2, 0, 0)

	def moveQuad(self, robo, quadrante):
		"""
			Descricao:
				Função que move o robô desejado para o quadrante desejado.
			Ultilizacao:
				self.moveQuad(robo, quadrante)
			Parametros:
				1) robo - número do robo aliado desejado int.
				2) quadrante - quadrante desejado [linha, coluna].
			Retornos:
				Sem retornos.
		"""
		cen = 0
		ang = 0

		# dicionario para pontos internos de cada quadrante
		dictQuad = {str([0, 0]): [96, 113],
					str([0, 1]): [203, 124],
					str([0, 2]): [313, 116],
					str([0, 3]): [423, 122],
					str([1, 0]): [100, 122],
					str([1, 1]): [209, 214],
					str([1, 2]): [309, 217],
					str([1, 3]): [420, 210],
					str([2, 0]): [100, 306],
					str([2, 1]): [207, 302],
					str([2, 2]): [314, 302],
					str([2, 3]): [426, 304],
					str([3, 0]): [101, 395],
					str([3, 1]): [210, 401],
					str([3, 2]): [312, 401],
					str([3, 3]): [413, 401]}

		cen = dictQuad[str(quadrante)]
		ang = self.__calculaAngulo(cen[0], cen[1], self.f[robo].pose[0], self.f[robo].pose[1])
		theta = self.f[robo].pose[2] - ang

		if self.f[robo].quadrante[0] != quadrante[0] or self.f[robo].quadrante[1] != quadrante[1]:
			self.movimentacao(robo, theta)

		else:
			self.moveSimples(0, 0, 0)