//*****************************************************************************
//*** Control de Brazo Robótico de 3 Grados de Libertad 					***
//*** Utiliza servos RX-10. Control mediante BleagleBone Black y Robocape	***
//*** Funciones para mover el brazo según ángulo y posición. Así como para	***
//*** desplazarse en trayectoria lineal o libre 							***
//*** Desarrollado por Juan Carlos Brenes. Mayo 2016. UPV, Valencia.		***
//*****************************************************************************

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include "BlackLib.h"
#include "GPIO.h"
#include <bitset>
//#include <stdexcept>

using namespace std;
using namespace BlackLib;
using namespace exploringBB;

//ID de los servos
const unsigned int idServoHombro1= 60;
const unsigned int idServoHombro2= 61;
const unsigned int idServoCodo= 62;
//Dimensiones de la herramienta y el brazo
const float longTool= 0.081;
float longitudesBrazo[5] = {0.073, 0.026, 0, 0.067, (0.039+longTool)};
unsigned int servoID[]= {idServoHombro1, idServoHombro2, idServoCodo};

const int sample=30;  //cantidad de divisiones para la trayectoria
const int twait=30; //tiempo de espera en mseg

int angDescanso[3]={150,110,240}; //Ángulos para la posición de descanso del brazo

//Puntos para implementar la trayectoria de la letra H
float posAtaqueH1[3]={-0.055, -0.15, 0.125};
float posAtaqueH2[3]={-0.035, -0.15, 0.125};
float posAtaqueH3[3]={-0.055, -0.135, 0.125};
float posH1i[3]={-0.055, -0.15, 0.155};
float posH1f[3]={-0.055, -0.12, 0.155};
float posH2i[3]={-0.035, -0.15, 0.155};
float posH2f[3]={-0.035, -0.12, 0.155};
float posH3i[3]={-0.055, -0.135, 0.155};
float posH3f[3]={-0.035, -0.135, 0.155};

//Puntos para implementar la trayectoria de una letra O cuadrada.
float posAtaqueO[3]={-0.025, -0.15, 0.125};
float posO1i[3]={-0.025, -0.15, 0.155};
float posO1f[3]={-0.025, -0.12, 0.155};
float posO2f[3]={-0.005, -0.12, 0.155};
float posO3f[3]={-0.005, -0.15, 0.155};

//Puntos para implementar la trayectoria de una letra L.
float posAtaqueL[3]={0.005, -0.15, 0.125};
float posL1i[3]={0.005, -0.15, 0.155};
float posL1f[3]={0.005, -0.12, 0.155};
float posL2f[3]={0.025, -0.12, 0.155};

//Puntos para implementar la trayectoria de la letra A cuadrada
float posAtaqueA2[3]={0.055, -0.15, 0.125};
float posAtaqueA1[3]={0.035, -0.15, 0.125};
float posAtaqueA3[3]={0.055, -0.135, 0.125};
float posA2i[3]={0.055, -0.15, 0.155};
float posA2f[3]={0.055, -0.12, 0.155};
float posA1i[3]={0.035, -0.15, 0.155};
float posA1f[3]={0.035, -0.12, 0.155};
float posA3f[3]={0.055, -0.135, 0.155};
float posA3i[3]={0.035, -0.135, 0.155};

//Función que calcula el checksum del mensaje a ser enviado al servo.
//Recibe el id, longitud, instrucción y vector con los parámetros
//Regresa un entero positivo con el valor del chucksum
unsigned int AXGetCheckSum (int id, int len, int inst, char param[]){

	unsigned int chksm= id + len + inst;

	for (unsigned int i=0; i < (sizeof(param)-1); i++) {
		chksm=chksm + (int)param[i];
	}
	chksm=~(chksm % 256); //se hace operacion NOT, y como un int es de 4 bytes, se hace modulo por 256 para obtener solo el byte menos significativo
	return chksm;
}

//Muestra en la consola un string en formato binario
void showMsgBinary (string msg){

	std::cout << "Info: Mensaje en binario: ";
	for (unsigned int i = 0; i < msg.size(); i += 2) {
	    std::cout << std::bitset<8>(msg[i]) << " ";
	    std::cout << std::bitset<8>(msg[i + 1]) << " ";
	}
	std::cout << endl;

}

//Funcion para mover los servos. Recibe el id del servo y la posición deseada
//Retorna un string con el mensaje a ser enviado por la UART
string AXServoMove (unsigned int  id, unsigned int pos){

    unsigned int instruction= 0x03; //Instruccion de escritura en el servo
    unsigned int address= 0x1E; //Direccion de Goal Position en el servo

    if ((pos<0) or (pos>300)){ //la posicion maxima es 300 y la minima 0
    	std::cout << "Error: Posición deseada fuera de rango. Rango permitido 0-300 grados."<<endl;
    }

	unsigned int value = round(pos * 3.41); //Convierte la pos a un valor de 0-1023
	unsigned int valueHigh= round(value / 256); //obtiene el byte superior del valor de posicion
	unsigned int valueLow= value % 256; 		//obtiene el byte inferior del valor de posicion
	char parameters[]={char(address), char(valueLow), char(valueHigh)}; //almacena la dirección y el valor de posición en un vector. Esto debido a que los parámetros pueden ser variables
	unsigned int length= sizeof(parameters)+2;	//calcula la longitud del paquete en base a la cantidad de parámetros
	unsigned int checksum = AXGetCheckSum(id, length, instruction, parameters);

	std::ostringstream buff; //buffer de salida
	//agrega al buffer el paquete de instruccion para el servo
	buff << char(0xFF)<<char(0xFF)<<char(id)<<char(length)<<char(instruction)<<parameters[0]<<parameters[1]<<parameters[2]<<char(checksum);

	//***DEBUG: Descomentar para ver la trama binaria enviada
	//showMsgBinary(buff.str());

	return buff.str();
}

//Función que calcula la cinemática inversa del brazo. Recibe los puntos x, y, z del espacio.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IK3ServoArm (float xh, float yh, float zh, float armLen[5], int ang[3]){

	std::cout << "Info: X= "<<xh << "  Y= "<<yh << "  Z= "<<zh <<endl;

	//Ecuación para encontrar el ángulo del servo del hombro horizontal
	int servoBase = round( atan2(zh,xh)* 180 / M_PI );
	ang[0]=servoBase;

	//Ecuaciones para encontrar el ángulo del servo del codo
	float s = sqrt(pow(xh,2) + pow(zh,2))-armLen[2];
	float t = -yh - armLen[0] - armLen[1];
	float Lra= sqrt(pow(s,2) + pow(t,2));
	float C5=  (pow(Lra,2) - pow(armLen[3],2) - pow(armLen[4],2)) / (2 * armLen[3] * armLen[4]);
	float angleElbowRad = -atan2( sqrt(1-pow(C5,2)), C5 );
	int servoElbow= round(angleElbowRad * 180 / M_PI );
	ang[2]=servoElbow;

	//Ecuaciones para encontrar el ángulo del servo del hombro vertical
	float gamma1= atan2(s,t);
	float gamma2= atan2( (armLen[4]*sin(-angleElbowRad)), (armLen[3]+armLen[4]*cos(-angleElbowRad)));
	int servoShoulder = round( -(gamma1-gamma2) * 180 / M_PI );
	ang[1]=servoShoulder;

}

//Función que recibe 3 ángulos calculados de la cinemática inversa y los ajusta a las especificaciones del robot Bioloid
//y los servos RX10. Modifica por referencia un vector con el valor de los ángulos en grados
void IKAdjust3RX (int ang[3]){

	//Cuando los 2 ángulos son cero, en casi todos los casos corresponde a un punto no alcanzable
	if ((ang[1]==0) || (ang[2]==0)){
			std::cout << "Error: La posición en el espacio no es alcanzable."<<endl;
	}

	//Ajusta el ángulo de la base en base.
	int servoBaseRX10= 152-(90-ang[0]); //150: ajuste del RX, 90-: ajuste con el eje de coordenadas
	if ((servoBaseRX10<0) || (servoBaseRX10>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo de la base."<<endl;
		ang[0]=150;  //Pone la posición por defecto del servo
	}else{
		ang[0]=servoBaseRX10;
	}

	ang[0]=servoBaseRX10;

	//Ajusta el ángulo del hombro
	int servoShoulderRX10= 150-(ang[1]); //150-: ajuste del RX
	//Revisa los valores máximos y minimos del servo
	if ((servoShoulderRX10<0) || (servoShoulderRX10>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo del hombro."<<endl;
		ang[1]=150;  //Pone la posición por defecto del servo
	}else if (servoShoulderRX10<60){
			std::cout << "Error: La posición deseada provoca colosión en el servo del hombro. Servo saturado a su valor mínimo."<<endl;
		ang[1]=60;
	}else if (servoShoulderRX10>240){
		std::cout << "Error: La posición deseada provoca colosión en el servo del hombro. Servo saturado a su valor máximo."<<endl;
		ang[1]=240;
	}else{
		ang[1]=servoShoulderRX10;
	}

	//Ajusta el ángulo del codo
	int servoElbowRX10= 150-(ang[2]); //150-: ajuste del RX
	//Revisa los valores máximos y minimos del servo
	if ((servoElbowRX10<0) || (servoElbowRX10>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo del codo."<<endl;
		ang[2]=150; //Pone la posición por defecto del servo
	}else if (servoElbowRX10<60){
			std::cout << "Error: La posición deseada provoca colisión en el servo del codo. Servo saturado a su valor mínimo."<<endl;
		ang[2]=60;
	}else if (servoElbowRX10>240){
		std::cout << "Error: La posición deseada provoca colosión en el servo del codo. Servo saturado a su valor máximo."<<endl;
		ang[2]=240;
	}else{
		ang[2]=servoElbowRX10;
	}

	std::cout << "Info: Ang Corregidos. Base: "<<ang[0]<< "  Hombro: "<<ang[1]<<"  Codo: "<<ang[2]<<endl;
}

//Función que calcula una trayectoria lineal para la punta del brazo. Recibe el punto de inicio y el punto final, así como las dimensiones del brazo.
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTraj (float pos1[3], float pos2[3], float armLen[5], int trajAng[3][sample]){

	//Se ejecuta el proceso para cada instante de tiempo. La cantidad de instantes está dada por la variable Sample
	for (int i=1;i<=sample;i++){
		//Se divide el trayecto entre la cantidad de instantes de tiempo. Obteniendo una serie de puntos intermedios
		//Se recorren uno por uno estos puntos
		float pos_xi= ( (pos2[0]-pos1[0])/sample*i ) + pos1[0] ;
		float pos_yi= ( (pos2[1]-pos1[1])/sample*i ) + pos1[1] ;
		float pos_zi= ( (pos2[2]-pos1[2])/sample*i ) + pos1[2] ;
		//En cada punto del trayecto se calcula la Cinemática Inversa
		int angles[3]={0,0,0};
	    IK3ServoArm (pos_xi, pos_yi, pos_zi, armLen, angles); //Calcula la Cinemática Inversa
	    IKAdjust3RX (angles);		//Corrige los ángulos para ajustarlos a los servos RX
	    //Se almacenan los ángulos para cada punto del trayecto lineal
	    trajAng[0][i]= angles[0];
	    trajAng[1][i]= angles[1];
	    trajAng[2][i]= angles[2];
	}
}

//Función que calcula una trayectoria libre entre 2 puntos. Recibe el punto de inicio y el punto final, así como las dimensiones del brazo.
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void freeTraj (float pos1[3], float pos2[3], float armLen[5], int trajAng[3][sample]){

	//Se calcula la Cinemática Inversa para el punto inicial
	int angles1[3]={0,0,0};
	IK3ServoArm (pos1[0], pos1[1], pos1[2], armLen, angles1); //Calcula la Cinemática Inversa
	IKAdjust3RX (angles1);		//Corrige los ángulos para ajustarlos a los servos RX
	//Se calcula la Cinemática Inversa para el punto final
	int angles2[3]={0,0,0};
	IK3ServoArm (pos2[0], pos2[1], pos2[2], armLen, angles2); //Calcula la Cinemática Inversa
	IKAdjust3RX (angles2);			//Corrige los ángulos para ajustarlos a los servos RX

	//Se ejecuta el proceso para cada instante de tiempo. La cantidad de instantes está dada por la variable Sample
	for (int i=1;i<=sample;i++){
		//Se divide el ángulo inicial y el final entre la cantidas de instántes de tiempo. Obteniendo una serie de ángulos intermedios
		int angleBase_i= round( (float)(angles2[0]-angles1[0])/sample*i + angles1[0] ) ;
		int angleShoulder_i= round( (float)(angles2[1]-angles1[1])/sample*i + angles1[1] );
		int angleElbow_i= round( (float)(angles2[2]-angles1[2])/sample*i + angles1[2] );
		//Se almacenan los ángulos para cada instante de tiempo
	    trajAng[0][i]= angleBase_i;
	    trajAng[1][i]= angleShoulder_i;
	    trajAng[2][i]= angleElbow_i;
	}
}


int main(int argc, char *argv[]) {

	//Apertura e inicializacion de la UART
	BlackLib::BlackUART uartServos( BlackLib::UART2,BlackLib::Baud38400,BlackLib::ParityNo,BlackLib::StopOne,BlackLib::Char8);
	bool isOpened = uartServos.open( BlackLib::ReadWrite | BlackLib::NonBlock );
	    if( !isOpened )	    {
	        std::cout << "UART DEVICE CAN\'T OPEN.;" << std::endl;
	        exit(1);	    }

	    std::cout << std::endl;
	    std::cout << "Device Path     : " << uartServos.getPortName() << std::endl;
	    std::cout << "Read Buf. Size  : " << uartServos.getReadBufferSize() << std::endl;
	    std::cout << "BaudRate In/Out : " << uartServos.getBaudRate( BlackLib::input) << "/"
	                                      << uartServos.getBaudRate( BlackLib::output) << std::endl;
	    std::cout << "Character Size  : " << uartServos.getCharacterSize() << std::endl;
	    std::cout << "Stop Bit Size   : " << uartServos.getStopBits() << std::endl;
	    std::cout << "Parity          : " << uartServos.getParity() << std::endl << std::endl;

	//Establecer la dirección de comunicación para el convertidor MAX485 de la RoboCape
	GPIO UART2_IO(49); //Pin que determina la dirección de la UART2
	UART2_IO.setDirection(GPIO::OUTPUT); //Se configura como salida
	UART2_IO.setValue(GPIO::HIGH);          //Enviar
	//UART_IO_49.setValue(GPIO::LOW);        //Recibir


	//Revisa que se tengan al menos 2 argumentos de entrada
	if (argc < 2){
		std::cout << "Error: Argumentos insuficientes. Se necesitan ingresar al menos un paŕametro en la instrucción." << std::endl;
		exit(1);
	}

	//*** Descanso ***
	//El brazo se mueve hacia la posición de descanso
	if (strcmp (argv[1], "-r")==0){

		std::cout << "Info: Movimiento a posición descanso." << std::endl;
	    for (int i=0; i<3; i++){
			//Se llama la funcion de mover Servo, la cual regresa un string que es el mensaje a enviar
	    	string sal= AXServoMove(servoID[i], angDescanso[i]);
			uartServos.write(sal);
		}
	}

	//*** Movimiento a ángulos ***
	//El brazo se mueve a la posición especificada por los 3 ángulos del argumento
	else if (strcmp (argv[1], "-a")==0){

		std::cout << "Info: Movimiento según los ángulos de entrada." << std::endl;
		//Revisa que se tengan al menos 3 argumentos de entrada
		if (argc < 5){
			std::cout << "Error: Argumentos insuficientes. Se necesitan 3 valores de ángulos." << std::endl;
			exit(1);
		}
		//Advierte si hay más argumentos de la cuenta
		if (argc > 5){
			std::cout << "Advertencia: Muchos argumentos. Se necesitan 3 valores de ángulos. Se utilizarán sólo los 3 primeros argumentos." << std::endl;
		}
		//Revisa que los 3 argumentos sean números
		int ang[3];
		for (int i=2; i<5; i++) {
			if ( ! (istringstream(argv[i]) >> ang[i-2]) ) {
		    	std::cout << "Error: Argumento inválido. Debe ingresar un número entero."<<endl;
		    	exit(1);
			}
		}
	    for (int i=0; i<3; i++){
			//Se llama la funcion de mover Servo, la cual regresa un string que es el mensaje a enviar
	    	string sal= AXServoMove(servoID[i], ang[i]);
			uartServos.write(sal);
		}
	}


	//*** Movimimiento a un punto ***
	//El brazo se mueve a la posicion especificada por las coordenadas x,y,z
	else if (strcmp (argv[1], "-p")==0){

		std::cout << "Info: Movimiento según la posición de entrada." << std::endl;
		//Revisa que se tengan al menos 3 argumentos de entrada
		if (argc < 5){
			std::cout << "Error: Argumentos insuficientes. Se necesitan 3 valores: x y z" << std::endl;
			exit(1);
		}
		//Advierte si hay más argumentos de la cuenta
		if (argc > 5){
			std::cout << "Advertencia: Muchos argumentos. Se necesitan 3 valores de posición. Se utilizarán sólo los 3 primeros argumentos." << std::endl;
		}
		//Revisa que los 3 argumentos sean números
		float pos[3];
		for (int i=2; i<5; i++) {
			if ( ! (istringstream(argv[i]) >> pos[i-2]) ) {
		    	std::cout << "Error: Argumento inválido. Debe ingresar un número."<<endl;
		    	exit(1);
			}
		}
		//Se calcula la Cinemática Inversa para el punto ingresado
	    int servoAngles[3]=  {0, 0, 0};
	    IK3ServoArm (pos[0], pos[1], pos[2], longitudesBrazo, servoAngles); //Calcula la Cinemática Inversa
	    IKAdjust3RX (servoAngles);		//Corrige los ángulos para ajustarlos a los servos RX
	    for (int i=0; i<3; i++){
			//Se llama la funcion de mover Servo, la cual regresa un string que es el mensaje a enviar
	    	string sal= AXServoMove(servoID[i], servoAngles[i]);
			uartServos.write(sal);
		}
	}

	//*** Movimiento Lineal ***
	//Se mueve el brazo en una trayectoria lineal
	else if (strcmp (argv[1], "-l")==0){

		std::cout << "Info: Movimiento Lineal según las posiciones de entrada." << std::endl;
		//Revisa que se tengan al menos 6 argumentos de entrada
		if (argc < 8){
			std::cout << "Error: Argumentos insuficientes. Se necesitan 6 valores: coordenadas del punto inicial y coordenadas del punto final" << std::endl;
			exit(1);
		}
		//Advierte si hay más argumentos de la cuenta
		if (argc > 8){
			std::cout << "Advertencia: Muchos argumentos. Se necesitan 6 valores de posición. Se utilizarán sólo los 6 primeros argumentos." << std::endl;
		}
		//Revisa que los 6 argumentos sean números
		float posIniLin[3];
		float posFinLin[3];
		for (int i=2; i<5; i++) {
			if ( ! (istringstream(argv[i]) >> posIniLin[i-2]) ) {
		    	std::cout << "Error: Argumento inválido. Debe ingresar un número."<<endl;
		    	exit(1);
			}
		}
		for (int i=5; i<8; i++) {
			if ( ! (istringstream(argv[i]) >> posFinLin[i-5]) ) {
		    	std::cout << "Error: Argumento inválido. Debe ingresar un número."<<endl;
		    	exit(1);
			}
		}
		//Se calcula la trayectoria desde el punto inicial hasta el final
		int returnAnglesLin[3][sample];
		linearTraj (posIniLin, posFinLin, longitudesBrazo, returnAnglesLin);
		//Se mueve el brazo a través de la trayectoria
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){
    	    	string sal= AXServoMove(servoID[i], returnAnglesLin[i][j]);
    			uartServos.write(sal);
    		}
    	    //std::cout << "B: " << returnAnglesLin[0][j] << "  H: " << returnAnglesLin[1][j] << "  C: " << returnAnglesLin[2][j] << std::endl;
    	    usleep(twait*1000);
    	}
	}

	//*** Movimiento Libre ***
	//Se mueve el brazo en una trayectoria libre
	else if (strcmp (argv[1], "-f")==0){

			std::cout << "Info: Movimiento Libre según las posiciones de entrada." << std::endl;
			//Revisa que se tengan al menos 6 argumentos de entrada
			if (argc < 8){
				std::cout << "Error: Argumentos insuficientes. Se necesitan 6 valores: coordenadas del punto inicial y coordenadas del punto final" << std::endl;
				exit(1);
			}
			//Advierte si hay más argumentos de la cuenta
			if (argc > 8){
				std::cout << "Advertencia: Muchos argumentos. Se necesitan 6 valores de posición. Se utilizarán sólo los 6 primeros argumentos." << std::endl;
			}
			//Revisa que los 6 argumentos sean números
			float posIniFree[3];
			float posFinFree[3];
			for (int i=2; i<5; i++) {
				if ( ! (istringstream(argv[i]) >> posIniFree[i-2]) ) {
			    	std::cout << "Error: Argumento inválido. Debe ingresar un número."<<endl;
			    	exit(1);
				}
			}
			for (int i=5; i<8; i++) {
				if ( ! (istringstream(argv[i]) >> posFinFree[i-5]) ) {
			    	std::cout << "Error: Argumento inválido. Debe ingresar un número."<<endl;
			    	exit(1);
				}
			}
			//Se calcula la trayectoria desde el punto inicial hasta el final
			int returnAnglesFree[3][sample];
			freeTraj (posIniFree, posFinFree, longitudesBrazo, returnAnglesFree);
			//Se mueve el brazo a través de la trayectoria
			for (int j=1;j<=sample;j++){
	    	    for (int i=0; i<3; i++){
	    	    	string sal= AXServoMove(servoID[i], returnAnglesFree[i][j]);
	    			uartServos.write(sal);
	    		}
	    	    //std::cout << "B: " << returnAnglesFree[0][j] << "  H: " << returnAnglesFree[1][j] << "  C: " << returnAnglesFree[2][j] << std::endl;
	    	    usleep(twait*1000);
	    	}
	}


	//*** Demostración ***
	//Se mueve el brazo a puntos predefinidos para una demostración (generación de letras)
	else if (strcmp (argv[1], "-d")==0){

		int returnAnglesDemo[3][sample];

		//----------LETRA H ---------------
		freeTraj (posAtaqueH1, posH1i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posH1i, posH1f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posH1f, posAtaqueH2, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posAtaqueH2, posH2i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posH2i, posH2f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posH2f, posAtaqueH3, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posAtaqueH3, posH3i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posH3i, posH3f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posH3f, posAtaqueO, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}


		//----------LETRA O ---------------
		freeTraj (posAtaqueO, posO1i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posO1i, posO1f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posO1f, posO2f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posO2f, posO3f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posO3f, posO1i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posO1i, posAtaqueL, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}


		//----------LETRA L ---------------
		freeTraj (posAtaqueL, posL1i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posL1i, posL1f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posL1f, posL2f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posL2f, posAtaqueA1, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}


		//----------LETRA A ---------------
		freeTraj (posAtaqueA1, posA1i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posA1i, posA1f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posA1f, posAtaqueA2, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posAtaqueA2, posA2i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posA2i, posA2f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posA2f, posAtaqueA3, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posAtaqueA3, posA3i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posA3i, posA3f, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posA3f, posAtaqueA1, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posAtaqueA1, posA1i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		linearTraj (posA1i, posA2i, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

		freeTraj (posA2i, posAtaqueA2, longitudesBrazo, returnAnglesDemo);
		for (int j=1;j<=sample;j++){
    	    for (int i=0; i<3; i++){ uartServos.write(AXServoMove(servoID[i], returnAnglesDemo[i][j])); }
    	    usleep(twait*1000);}

	}
	else {
			std::cout << "Error: Parámetro de entrada desconocido." << std::endl;
	}



    uartServos.close();

	return 0;
}

