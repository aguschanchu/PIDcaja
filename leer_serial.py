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
periodoPerturbacion=30 #Periodo en minutos
dutyCyclePerturbacion=0.4 # 0<x<1
esperaInicial = 240 #La idea es no perturbarlo, mientras hace el autotune, no? En minutos
time.sleep(2)
#Abre el lector serial. El arudino debería strings en el siguiente forato:
#NSensor, Temp
#(donde temp, está en celcius)
#Ej: N2, 27.83
inicio = time.time()
ultimaModificacionPerturbacion = inicio
dutyModificacion=False
voltaje = 0
inst.write("CH1:VOLTage "+str(voltaje))
while True:
    #Guardamos los datos que entrega el arduino via serial
    linea = arduino.readline().decode('utf-8')
    print(linea)
    with open(outputdir+"output.txt",'a') as file:
        file.write(linea+"\n")
    try:
    	linea = ast.literal_eval(linea)
    except:
        linea = ""
    if type(linea) == list:
        with open(outputdir+"sensor"+str(linea[0])+".txt",'a') as file:
            file.write(str(linea[1])+','+str(time.time()-inicio)+"\n")
    #Realizamos modificaciones en el perturbador
    if dutyModificacion == False and (time.time() - ultimaModificacionPerturbacion) > periodoPerturbacion*60*dutyCyclePerturbacion and (time.time()-inicio) > esperaInicial * 60:
        voltaje = 3
        dutyModificacion = True
        inst.write("CH1:VOLTage "+str(voltaje))
    if (time.time() - ultimaModificacionPerturbacion) > periodoPerturbacion*60 and (time.time()-inicio) > esperaInicial * 60:
        voltaje = 6
        dutyModificacion = False
        ultimaModificacionPerturbacion = time.time()
        inst.write("CH1:VOLTage "+str(voltaje))
    #Guardamos el valor del voltaje
    with open(outputdir+"sensor"+str(7)+".txt",'a') as file:
        file.write(str(voltaje)+','+str(time.time()-inicio)+"\n")


else:
    print(linea)
arduino.close()
