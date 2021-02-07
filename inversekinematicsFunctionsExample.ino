#include <MatrixMath.h> //Get from https://github.com/eecharlie/MatrixMath
#include <math.h>

const float Pi = 3.141593;

//Jacobian Matrices
float J[3][4];  float invJ[4][3]; float Jt[4][3];
//Other Matrices
float I[4][4];  float D[4][4];  float sqD[4][4];  float inv[4][4];
//transform Matrices
float AB[4][4]; float BC[4][4]; float CD[4][4]; float DE[4][4]; float EF[4][4];
float AC[4][4]; float AD[4][4]; float AE[4][4]; float AF[4][4]; 

//Initial joint values
float q1; float q2; float q3; float q4; float q5; float q6;

//joint value limits
float lowerLim[1][4]; float UpperLim[1][4]; float q[1][4];

//Constnt radius
const float r = 175;

//Toolpoint
float X; float Y; float Z;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    lowerLim[0][0] = 0; //q1 lowerlimit
    lowerLim[0][1] = 0; //q2 lowerlimit
    lowerLim[0][2] = 0; //q3 lowerlimit
    lowerLim[0][3] = -4*Pi; //q5 lowerlimit

    UpperLim[0][0] = 70; //q1 Upperlimit
    UpperLim[0][1] = 70; //q2 Upperlimit
    UpperLim[0][2] = 70; //q3 Upperlimit
    UpperLim[0][3] = 4*Pi; //q5 Upperlimit
    //initial values
    q1 = 10; q2 = 10; q3 = 10; 
    q4 = 0; q5 = Pi/4; q6 = Pi/4;
    
    q[0][0] = q1;
    q[0][1] = q2;
    q[0][2] = q3;
    q[0][3] = q5;
}

void loop() {
  //Forward kinematics
  forwardkinematics(q1, q2, q3, q4, q5, q6);
  Serial.print("X=");
  Serial.println(X);
  Serial.print("Y=");
  Serial.println(Y); 
  Serial.print("Z=");
  Serial.println(Z);  
  delay(5000);

  //Inverse kinematics
  computeJacobian(q2, q3, q5);
  dampedLeastSquares();
  Matrix.Print((float*)invJ, 4, 3, "invJ"); //Jacobian inverse dq = invJ * dx;
}

//Basic functions:
void identity44(float I[4][4]){
  for(int ii = 0; ii < 4; ii++){
    for(int jj = 0; jj < 4; jj++){
      if(ii==jj){
        I[ii][jj] = 1;
      }
      else{
        I[ii][jj] = 0;
      }
    }   
  }
}

void TranslationTransform(float matrix[4][4],float x, float y, float z){
  matrix[0][3] = x; matrix[1][3] = y; matrix[2][3] = z;
}

void RotationX(float matrix[4][4], float theta){
  matrix[1][1] = cos(theta); matrix[1][2] = sin(theta); 
  matrix[2][1] = -sin(theta); matrix[2][2] = cos(theta); 
}

void RotationZ(float matrix[4][4], float theta){
  matrix[0][0] = cos(theta); matrix[0][1] = -sin(theta); 
  matrix[1][0] = sin(theta); matrix[1][1] = cos(theta); 
}

void computeJacobian(float q2, float q3, float q5){
  J[0][0] = 0; J[0][1] = -sin(q2/r)*sin(q5)-(q3*cos(q2/r)*sin(q5))/r; J[0][2] = -sin(q2/r)*sin(q5); J[0][3] = r*cos(q5)*(cos(q2/r)-1)-q3*sin(q2/r)*cos(q5); 
  J[1][0] = 0; J[1][1] =  sin(q2/r)*cos(q5)+(q3*cos(q2/r)*cos(q5))/r; J[1][2] =  sin(q2/r)*cos(q5); J[1][3] = r*sin(q5)*(cos(q2/r)-1)-q3*sin(q2/r)*sin(q5); 
  J[2][0] = 1; J[2][1] = cos(q2/r)-(q3*sin(q2/r))/r; J[2][2] = cos(q2/r); J[2][3] = 0;
}

void forwardkinematics(float q1, float q2, float q3, float q4, float q5, float q6){
  //Construct homogeneous transforms
  identity44(AB); //base to tube 1
  TranslationTransform(AB,0, 0, q1);

  identity44(BC); //tube 1 to q5 rotation
  RotationZ(BC, q5);

  identity44(CD); //q5 rotation to tube 2 tip
  RotationX(CD,q2/r);  
  TranslationTransform(CD,0, r*(1-cos(q2/r)), r*sin(q2/r));

  identity44(DE); //q2 to tube 3
  TranslationTransform(DE,0, 0, q3);

  identity44(EF); //tool rotation
  RotationZ(EF, q6);

  //Cascade transforms from base to tool point
  Matrix.Multiply((float*)AB, (float*)BC, 4, 4, 4, (float*)AC);
  Matrix.Multiply((float*)AC, (float*)CD, 4, 4, 4, (float*)AD);
  Matrix.Multiply((float*)AD, (float*)DE, 4, 4, 4, (float*)AE); 
  Matrix.Multiply((float*)AE, (float*)EF, 4, 4, 4, (float*)AF);

  //Output Forward Kinematics
  X = AF[0][3]; Y = AF[1][3]; Z = AF[2][3];
}

void dampedLeastSquares(){

  //damping matrix D
  identity44(D);
  for(int ii = 0; ii<4; ii++){
    D[ii][ii] = sq((2*q[0][ii] - UpperLim[0][ii] - lowerLim[0][ii])/(UpperLim[0][ii] - lowerLim[0][ii])) + 1;
  }
  //Matrix.Print((float*)D, 4, 4, "D");
  
  //Find inverse formula:  inv_J = inv(J'*J + D^2)*J'
  Matrix.Multiply((float*)D, (float*)D, 4, 4, 4, (float*)sqD);//square D
  //Matrix.Print((float*)sqD, 4, 4, "D^2");
  Matrix.Transpose((float*)J, 3, 4, (float*)Jt); //J' 4 by 3
  //Matrix.Print((float*)Jt, 4, 3, "Jt");
  Matrix.Multiply((float*)Jt, (float*)J, 4, 3, 4, (float*)inv);//J'*J
  //Matrix.Print((float*)inv, 4, 4, "Jt*J");
  Matrix.Add((float*)inv, (float*)sqD, 4, 4, (float*)inv); // J'J + D^2
  //Matrix.Print((float*)inv, 4, 4, "Jt*J + D^2");
  Matrix.Invert((float*)inv, 4);
  //Matrix.Print((float*)inv, 4, 4, "inv(Jt*J + D^2)");
  Matrix.Multiply((float*)inv, (float*)Jt, 4, 4, 3, (float*)invJ);//answer
}


