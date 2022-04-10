# project_crontol

 
Control PID de temperatura. En el proyecto se puede elegir el setpoint (30, 35, 40, 45) y el tipo de control (PID, PI o P). Para elevar la temperatura se utiliza un lámpara.
Para el desarrollo del control PID se utilizó el primer método de Ziegler-Nichols, el cual se basa en la respuesta del sistema en lazo abierto a una entrada escalón unitario, que se obtiene en forma experimental.
Aunque al final se utilizo matlab para obtener un mejor desempeño. 
Las pruebas se llevaron a cabo en las mismas condiciones de temperatura  y tomando mediciones cada un segundo, a partir de esos datos realizamos los cálculos para obtener las ganancias correspondiente.
Control PID diseñado a través de un microprocesador arduino,  este detecta el cruce por cero de una onda sinusoidal y regula la tensión que tendrá una lámpara, que será la encargada de darle calor al sistema.
