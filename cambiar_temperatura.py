import redis
import argparse
import time

r = redis.StrictRedis(host='localhost', port=30000, db=0)
parser = argparse.ArgumentParser(description='Cambia valor de setpoint de la DB y espera a que la temperatura se estabilice')
parser.add_argument('-s','--setpoint',help='Nuevo setpoint (float)')
parser.add_argument('-e','--estabilizador',help='Si es 1, es programa se apaga cuando la temperatura se estabilice en el nuevo setpoint')
args = parser.parse_args()

#Mandamos el nuevo setpoint a la DB
r.set('setpoint',args.setpoint)

'''
Si nos piden que esperemos a que estabilice, vamos a leer la temperatura de la DB. Si abs(temperaturaControl-setpoint)<0.2 durante
mas de 15 minutos, vamos a asumir que se estbilizo la temperatura, y, por lo tanto, salir
'''
estabilizado = False
reloj=time.time()
setpoint=float(args.setpoint)
while args.estabilizador == '1' and not estabilizado:
	#Obtenemos temperatura de control de la DB
	control = float(r.get("control").decode('UTF-8'))
	print("Temperatura de control: " + str(control))
	print("Setpoint: " + str(setpoint))
	if abs(control-setpoint) > 0.2:
		reloj = time.time()
	#Si no se excedio de los limites anteriores durante mas de 15 min, salimos
	if time.time()-reloj > 60*15:
		estabilizado=True
	time.sleep(10)
