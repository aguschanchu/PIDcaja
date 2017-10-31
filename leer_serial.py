# -*- coding: utf-8 -*-
"""
Este programa unicamente se encarga de la comunicacion con Arduino, y escribe los datos en output dir. Para enviar
informacion, hacerlo via redis.
Variables monitoreadas:
-setpoint
Forma de cambiar el setpoint:
r.set("setpoint",nuevoSetpoint)
"""

import serial, time
import ast
import redis
import traceback
arduino = serial.Serial("/dev/ttyACM0", 9600)
outputdir = "/home/iteda/Dropbox/ITeDA/Scripts/Lecroy/resultados/camara/"
r = redis.StrictRedis(host='localhost', port=30000, db=0)

############ FIN DE CONFIG
if not r.exists('setpoint'):
    print("No existe setpoint en DB, no vamos a cargarlo")
else:
    lastSetpoint = float(r.get('setpoint').decode('UTF-8'))

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
        #Veamos por casos, que tipo de informacion recibimos
        if type(linea[0]) == int:
	        with open(outputdir+"sensor"+str(linea[0])+".txt",'a') as file:
	            file.write(str(linea[1])+','+str(linea[2])+"\n")
	        if linea[0] == 7:
	            r.set('control',linea[1])
        elif linea[0] == "e":
            with open(outputdir+"estado"+".txt",'a') as file:
                lineaw = str(linea[1])
                for i in range(2,7):
                    lineaw=lineaw+','+str(linea[i])
                file.write(lineaw+"\n")
            #actualizamos sensor control

    #Leemos informacion de la DB, y enviamos, si es que hay cambios
    if not r.exists('setpoint'):
        print("No existe setpoint en DB, no vamos a cargarlo")
    else:
        newSetpoint=float(r.get('setpoint').decode('UTF-8'))
        if newSetpoint != lastSetpoint:
            mensaje='[setpoint,'+str(newSetpoint)+']'
            print("Nuevo setpoint recibido")
            lastSetpoint=newSetpoint
            #Cambiamos correctamente el setpoint? Esperemos a que eso ocurra
            cambiado = False
            intentos = 31
            print(newSetpoint, lastSetpoint)
            while not cambiado:
                linea = arduino.readline().decode('utf-8')
                intentos = intentos + 1
                print(intentos)
                s="setpoint: "+str(newSetpoint)
                if 'setpoint' in linea:
                    #es el lugar donde justo dice el setpoint
                    recv = float(linea[10:14])
                else:
                    recv = -10
                if recv == newSetpoint:
                    cambiado = True
                    print("OK")
                if intentos > 30:
                    arduino.write(mensaje.encode())
                    intentos=0
                print(linea)
                #Lo anterior no funciona, entonces, salvo que de problemas...
                cambiado=True


'''
except:
    arduino.close()
    traceback.print_exc()
    '''
#Abre el lector serial. El arudino debería strings en el siguiente forato:
#NSensor, Temp, Tiemo
#(donde temp, está en celcius)
#Ej: [2, 27.83, 12.2]
# Se utiliza "v" para comunicarse con fuente de DC
# Se utiliza "e" para enviar un estado del PID con el formato
# [e,tiempo,kp,ki,kd,setpoint,texto]
'''
inst.write("CH1:VOLTage 0")
while True:
    registroArduino(arduino,outputdir)
else:
    print(linea)
arduino.close()
'''
