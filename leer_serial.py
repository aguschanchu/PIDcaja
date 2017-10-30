# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 15:43:38 2017

@author: Agustin
"""

import serial, time
import ast
import visa
rm = visa.ResourceManager('@py')
inst = rm.open_resource("USB0::1155::30016::SPD00002140064::0::INSTR")
arduino = serial.Serial("/dev/ttyACM0", 9600)

outputdir = "/home/iteda/Dropbox/ITeDA/Sistema de control/autotune/codigo 1.0/"

## Comunicacion con Arduino

#=====================================

def sendToArduino(sendStr):
  arduino.write(sendStr)


#======================================

def recvFromArduino():
  startMarker = 91
  endMarker = 93

  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many

  # wait for the start character
  while  ord(x) != startMarker:
    x = arduino.read()
  # save data until the end marker is found
  while ord(x) != endMarker:
    if ord(x) != startMarker:
      ck = ck + x.decode('utf-8')
      byteCount += 1
    x = arduino.read()
  return(ck)


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded

    global startMarker, endMarker

    msg = ""
    while msg.find("READY") == -1:

      while arduino.inWaiting() == 0:
        pass

      msg = recvFromArduino()


      print(msg)

#======================================

def cambiarParametros(td):
	numLoops = len(td)
	waitingForReply = False
	waitForArduino()
	n = 0
	while n < numLoops:
		teststr = td[n]
		if waitingForReply == False:
			waitForArduino()
		sendToArduino(teststr.encode())
		print("Enviado -- LOOP NUM " + str(n) + " TEST STR " + teststr)
		waitingForReply = True
		dataRecvd=""
		paquetesRecibidos = 0
		while waitingForReply:
			while arduino.inWaiting() == 0:
    				pass
			dataRecvd = recvFromArduino()
			paquetesRecibidos+=1
			#Esperamos a confirmacion de lo recibido
			if dataRecvd.find("OK") != -1:
				waitingForReply = False
			print("Recibido  " + dataRecvd)
			#Ya enviamos el mensaje, y esperamos una cierta cantidad de paquetes para recibir confirmacion
			#Si se demora mucho, enviamos otro
			if paquetesRecibidos>20:
				waitForArduino()
				sendToArduino(teststr.encode())
				print("Enviado -- LOOP NUM " + str(n) + " TEST STR " + teststr)
				paquetesRecibidos=0
		n += 1
		print("===========")
		time.sleep(5)

def registroArduino(arduino,script_dir):
   #Guardamos los datos que entrega el arduino via serial
    linea = arduino.readline().decode('utf-8')
    print(linea)
    with open(script_dir+"output.txt",'a') as file:
        file.write(linea+"\n")
    try:
    	linea = ast.literal_eval(linea)
    except:
        linea = ""
    if type(linea) == list:
        #Veamos por casos, que tipo de informacion recibimos
        if type(linea[0]) == int:
	        with open(script_dir+"sensor"+str(linea[0])+".txt",'a') as file:
	            file.write(str(linea[1])+','+str(linea[2])+"\n")
        '''
        elif linea[0] == "v":
            #Cambiamos valor de voltaje, y almacenamos únicamente cuando es alterado
            inst.write("CH1:VOLTage "+str(linea[1]))
            with open(outputdir+"voltajeDC"+".txt",'a') as file:
                file.write(str(linea[1])+','+str(linea[2])+"\n")
        elif linea[0] == "e":
            with open(outputdir+"estado"+".txt",'a') as file:
                lineaw = str(linea[1])
                for i in range(2,7):
                    lineaw=lineaw+','+str(linea[i])
                file.write(lineaw+"\n")
        '''



#Abre el lector serial. El arudino debería strings en el siguiente forato:
#NSensor, Temp, Tiemo
#(donde temp, está en celcius)
#Ej: [2, 27.83, 12.2]
# Se utiliza "v" para comunicarse con fuente de DC
# Se utiliza "e" para enviar un estado del PID con el formato
# [e,tiempo,kp,ki,kd,setpoint,texto]
inst.write("CH1:VOLTage 0")
while True:
    registroArduino(arduino,outputdir)
else:
    print(linea)
arduino.close()
