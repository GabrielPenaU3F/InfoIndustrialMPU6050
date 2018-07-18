import processing.serial.*;

final String puertoArduino = "COM4";
Serial puerto;

float ax, ay, az; float ax_previous, ay_previous, az_previous; String ax_str, ay_str, az_str; float ax_off=0, ay_off=0, az_off=0;
float vx, vy, vz; float vx_previous, vy_previous, vz_previous;

int intervalo = 0;
int first_reading_flag = 0;
boolean calibrating;
float x_centro, y_centro; //Coordenadas del centro del dibujo
float [] hq = null;
float [] q = new float [4];
float [] Euler = new float [3]; // psi, theta, phi

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600;
PImage topside,downside,frontside,rightside;

//IMPORTANTE: Mover todo el combo Arduino-sensor para mejor funcionamiento


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- INITIAL SETUP ----- -----------------------------------------
//-------------------------------------------------------------------------------------------------------

void setup() {
  
  size(800,600, P3D);
  textureMode(NORMAL);
  fill(255);
  stroke(color(44,48,32));
  
  font = loadFont("CourierNew36.vlw"); 
  topside = loadImage("MPU6050 A.png");//Top Side
  downside = loadImage("MPU6050 B.png");//Botm side
  frontside = loadImage("MPU6050 E.png"); //Wide side
  rightside = loadImage("MPU6050 F.png");// Narrow side
  
  puerto = new Serial(this, puertoArduino, 115200);  
  puerto.write('r');
  
  delay(100);
  puerto.clear();
 
}


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- DRAW THE CUBE -----------------------------------------------
//-------------------------------------------------------------------------------------------------------

void drawCube() {  
  
  pushMatrix();
  actualizar_posicion();
  translate(x_centro, y_centro, 0);

  //scale(5,5,5);
  scale(10);

  rotateZ(-Euler[2]);
  rotateX(-Euler[1]);
  rotateY(-Euler[0]);
  
  topboard(topside);
  botomboard(downside);
  sideboarda(frontside);
  sideboardb(rightside);
  
  
  popMatrix();
    
}

void actualizar_posicion() {
  float desplazamiento_x = calcular_desplazamiento_x();
  float desplazamiento_y = calcular_desplazamiento_y();
  x_centro = x_centro + desplazamiento_x*1000;
  y_centro = y_centro + desplazamiento_y*1000;
}


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- CENTERING THE IMAGE -----------------------------------------
//-------------------------------------------------------------------------------------------------------

void keyPressed() {
  if(key == 'h') {
    println("pressed h");
    
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hq = quatConjugate(q);
    centrar_dibujo();
    
  }
  else if(key == 'n') {
    println("pressed n");
    hq = null;
  }
}

void centrar_dibujo() {
  pushMatrix();
  translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50);
  x_centro = VIEW_SIZE_X/2;
  y_centro = VIEW_SIZE_Y/2 + 50;
  popMatrix();
}


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- THE READINGS ------------------------------------------------
//-------------------------------------------------------------------------------------------------------

void obtener_valores_sensor(String lectura_accel_gyro[]) {

  String lectura_accel[] = split(lectura_accel_gyro[0], ","); //Separo los valores correspondientes a los tres ejes del acelerometro
  String lectura_cuaternion[] = split(lectura_accel_gyro[1], ","); //Separo los valores correspondientes a las cuatro componentes del cuaternion
  
  obtener_valores_acelerometro(lectura_accel);
  obtener_valores_cuaternion(lectura_cuaternion);
  
}

//Siempre se guardan la lectura actual y la inmediata anterior, para hacer calculos de posicion
//Velocidad en MRUV: v(t) = v0 + a*t
void obtener_valores_acelerometro(String lectura_accel[]) {

  if (calibrating == true) {
    
    ax_off = float(lectura_accel[0]);
    ay_off = float(lectura_accel[1]);
    az_off = float(lectura_accel[2]);
    
  } else {
  
    if (first_reading_flag == 0) { //Primera lectura
      
      ax_previous = float(lectura_accel[0]) - ax_off;
      ay_previous = float(lectura_accel[1]) - ay_off;
      az_previous = float(lectura_accel[2]) - az_off;
      
      vx_previous = 0;
      vy_previous = 0;
      vz_previous = 0;
      
      first_reading_flag++;
      
    } else if (first_reading_flag == 1) { //Segunda lectura
        
      ax = float(lectura_accel[0]) - ax_off;
      ay = float(lectura_accel[1]) - ay_off;
      az = float(lectura_accel[2]) - az_off;
      
      vx = get_velocidad_x();
      vy = get_velocidad_y();
      vz = get_velocidad_z();
    
    } else {
    
      ax_previous = ax;
      ay_previous = ay;
      az_previous = az;
      
      ax = float(lectura_accel[0]) - ax_off;
      ay = float(lectura_accel[1]) - ay_off;
      az = float(lectura_accel[2]) - az_off;
      
      vx_previous = vx;
      vy_previous = vy;
      vz_previous = vz;
      
      vx = get_velocidad_x();
      vy = get_velocidad_y();
      vz = get_velocidad_z();
    
    }
  }
  
}


void obtener_valores_cuaternion(String lectura_cuaternion[]) {
  
  q[0] = float(lectura_cuaternion[0]);
  q[1] = float(lectura_cuaternion[1]);
  q[2] = float(lectura_cuaternion[2]);
  q[3] = float(lectura_cuaternion[3]);
  
}


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- DISPLACEMENT CALCULATIONS -----------------------------------
//-------------------------------------------------------------------------------------------------------


/*LOS EJES QUE SE MUEVEN SON:
  Eje Y del MPU -> Eje X del dibujo
  Eje Z del MPU -> Eje Y del dibujo
*/

float calcular_desplazamiento_x() {
  
  if (first_reading_flag > 0) { //Ya hubo lecturas anteriores
  
    float desplazamiento = get_desplazamiento_y();
    
    if (x_centro + desplazamiento > VIEW_SIZE_X) {
      return 0;
    } else if (x_centro + desplazamiento < 0) {
      return 0;
    } else return desplazamiento;
    
  } else return 0;

}

float calcular_desplazamiento_y() {
 
  if (first_reading_flag > 0) {
    
    float desplazamiento = get_desplazamiento_z();
  
    if (y_centro + desplazamiento > VIEW_SIZE_Y) {;
      return 0;
    } else if (y_centro + desplazamiento < 0) {
      return 0;
    } else return desplazamiento;
    
  } else return 0;
  
}


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- MAIN DRAW LOOP ----------------------------------------------
//-------------------------------------------------------------------------------------------------------

void draw() {

   if (millis() - intervalo > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        puerto.write('r');
        intervalo = millis();
    }
  
  if (puerto.available() > 0) {
    
    String lectura = puerto.readStringUntil('\n');
    
    if (lectura != null) {
      
      background(#000000);
      textFont(font, 20);
      textAlign(LEFT, TOP);
      if (millis() < 8000) {
        
         text("Calibrating... wait a few seconds", 20, 500);
         calibrating = true;
         String lectura_accel_gyro[] = split(trim(lectura), ";"); //Separo los valores correspondientes al acelerometro de los correspondientes al giroscopo
         obtener_valores_sensor(lectura_accel_gyro);
         centrar_dibujo();
         
      } else {

        calibrating = false;
        String lectura_accel_gyro[] = split(trim(lectura), ";"); //Separo los valores correspondientes al acelerometro de los correspondientes al giroscopo
        obtener_valores_sensor(lectura_accel_gyro);
    
        if(hq != null) { // use home quaternion
          quaternionToEuler(quatProd(hq, q), Euler);
          text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
        }
        else {
          quaternionToEuler(q, Euler);
          text("Point  X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
        }
        
        text("Q:\n" + q[0] + "\n" + q[1] + "\n" + q[2] + "\n" + q[3], 20, 20);
        text("Euler Angles:\nYaw (psi)  : " + degrees(Euler[0]) + "\nPitch (theta): " + degrees(Euler[1]) + "\nRoll (phi)  : " + degrees(Euler[2]), 200, 20);
        text("Acceleration: \nX=" + ax + "\nY=" + ay + "\nZ=" + az + "\n", 20, 130);
        
        drawCube(); 
      }
      delay(20);
      
    }
    
  } 
  
}
  
//-------------------------------------------------------------------------------------------------------
//----------------------------------------- QUATERNION FUNCTIONS ----------------------------------------
//-------------------------------------------------------------------------------------------------------

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];
  
  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  
  return q;
}

// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
  float [] q = new float[4];
  
  float halfAngle = angle / 2.0;
  float sinHalfAngle = sin(halfAngle);
  q[0] = cos(halfAngle);
  q[1] = -axis[0] * sinHalfAngle;
  q[2] = -axis[1] * sinHalfAngle;
  q[3] = -axis[2] * sinHalfAngle;
  
  return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];
  
  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];
  
  return conj;
}


//-------------------------------------------------------------------------------------------------------
//----------------------------------------- MEAN ACCELERATIONS ------------------------------------------
//-------------------------------------------------------------------------------------------------------


//Las aceleraciones 'promedio' corresponden a la pendiente de la recta secante a la curva de aceleración, pasando por los dos puntos en que medimos
//El teorema del valor medio de Lagrange justifica esta aproximación

float get_aceleracion_promedio_x() {
  return (ax - ax_previous)/0.020;
}

float get_aceleracion_promedio_y() {
  return (ay - ay_previous)/0.020;
}

float get_aceleracion_promedio_z() {
  return (az - az_previous)/0.020;
}

float get_velocidad_x() {
  return vx_previous + ax_previous*0.020 + (ax - ax_previous)*(0.020*0.020)/0.040;
}

float get_velocidad_y() {
  return vy_previous + ay_previous*0.020 + (ay - ay_previous)*(0.020*0.020)/0.040;
}

float get_velocidad_z() {
  return vz_previous + az_previous*0.020 + (az - az_previous)*(0.020*0.020)/0.040;
}

float get_desplazamiento_y() {
  return 200*(vy_previous*0.020 + ay_previous*(0.020*0.020)/2 + (ay - ay_previous)*(0.020*0.020*0.020)/0.120);
}

float get_desplazamiento_z() {
  return 200*(vz_previous*0.020 + az_previous*(0.020*0.020)/2 + (az - az_previous)*(0.020*0.020*0.020)/0.120);
}

//-------------------------------------------------------------------------------------------------------
//----------------------------------------- DRAW SIDES --------------------------------------------------
//-------------------------------------------------------------------------------------------------------

void topboard(PImage imag) {
  beginShape(QUADS);
  texture(imag);
  // -Y "top" face
  vertex(-10, -0.5, -7.5, 0, 0);
  vertex( 10, -0.5, -7.5, 0.5, 0);
  vertex( 10, -0.5,  7.5, 0.5, 0.5);
  vertex(-10, -0.5,  7.5, 0, 0.5);

  endShape();
}

void botomboard(PImage imag) {
  beginShape(QUADS);
  texture(imag);

  // +Y "bottom" face
  vertex(-10,  0.5,  7.5, 0, 0);
  vertex( 10,  0.5,  7.5, 0.5, 0);
  vertex( 10,  0.5, -7.5, 0.5, 0.5);
  vertex(-10,  0.5, -7.5, 0, 0.5);
    
  endShape();
}


void sideboarda(PImage imag) {
  beginShape(QUADS);
  texture(imag);

  // +Z "front" face
  vertex(-10, -0.5,  7.5, 0, 0);
  vertex( 10, -0.5,  7.5, 0.5, 0);
  vertex( 10,  0.5,  7.5, 0.5, 0.5);
  vertex(-10,  0.5,  7.5, 0, 0.5);

  // -Z "back" face
  vertex( 10, -0.5, -7.5, 0, 0);
  vertex(-10, -0.5, -7.5, 0.5, 0);
  vertex(-10,  0.5, -7.5, 0.5, 0.5);
  vertex( 10,  0.5, -7.5, 0, 0.5);


  endShape();
}

void sideboardb(PImage imag) {
  beginShape(QUADS);
  texture(imag);

   // +X "right" face
  vertex( 10, -0.5,  7.5, 0, 0);
  vertex( 10, -0.5, -7.5, 0.5, 0);
  vertex( 10,  0.5, -7.5, 0.5, 0.5);
  vertex( 10,  0.5,  7.5, 0, 0.5);

  // -X "left" face
  vertex(-10, -0.5, -7.5, 0, 0);
  vertex(-10, -0.5,  7.5, 0.5, 0);
  vertex(-10,  0.5,  7.5, 0.5, 0.5);
  vertex(-10,  0.5, -7.5, 0, 0.5);

  endShape();
}
