/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-serial-plotter
 */
 float f=50;
 float f2=10000;
 float Tt=1/f2;
 float T=1/f;
float i = 0;
float arr[21], arr_1[21], arr_2[21];
float arr2[201];
int x=0;
int x1,ls,lt=0;
float trig_val=0;
float A=2;
void setup() 
{
//  Serial.begin(115200);
DDRB |= (1<<5) | (1<<4) | (1<<3);
}

void loop() 
{
  if(i<=T)
  {
  for(i = 0; i <=T; i += T/20.0) 
  {
    float y1 = 1 * sin(i * 2*M_PI*f);
    float y2 = 2 * sin((i + 120)* M_PI / 180);
    float y3 = 5 * sin((i + 240)* M_PI / 180);
    
    arr[x]=y1;
    arr_1[x]=y2;
    arr_2[x]=y3;
    
    x+=1;
  }
      double slope=4/Tt;
    double slope2=-slope;
    double slope3=slope;
    double c=(1-(slope2*(Tt/4)));
    double c2=-(slope*Tt);
   for(float t = 0; t <=Tt; t += Tt/200.0) 
  {
         if(t<=(Tt/4))
    {
      trig_val=(A*(slope*t));
    }
    else if(t<=(3*Tt/4) and t>Tt/4)
     {
        trig_val=(A*(c+slope2*t));
     }
    else if(t>(3*(Tt/4)) and t<=Tt)
    {
        trig_val=(A*(c2+slope3*t));
    }
    arr2[x1]=trig_val;
    x1+=1;
  }
 
  }
  else
    {
//      
//    Serial.print("i: ");
//    Serial.print(arr[ls/100]);
//    
//    Serial.print("   j: ");
//    Serial.println(arr2[lt]);
      if(arr[ls/100]>arr2[lt])
      {
        PORTB |= (1<<5);
//        int v=1;
//            Serial.print("i: ");
//Serial.println(v);

      }
      else
      {
        PORTB &= (0<<5);
//        int v=0;
//                Serial.print("i: ");
//  Serial.println(v);

      }
      if(arr_1[ls/100]>arr2[lt])
      {
        PORTB |= (1<<4);
//        int v=1;
//            Serial.print("i: ");
//Serial.println(v);

      }
      else
      {
        PORTB &= (0<<4);
//        int v=0;
//                Serial.print("i: ");
//  Serial.println(v);

      }
      if(arr_2[ls/100]>arr2[lt])
      {
        PORTB |= (1<<3);
//        int v=1;
//            Serial.print("i: ");
//Serial.println(v);

      }
      else
      {
        PORTB &= (0<<3);
//        int v=0;
//                Serial.print("i: ");
//  Serial.println(v);

      }
      }
      if(ls/100>x)
      {
        ls=0;
        }
        else
        {
          ls+=1;
    }
          if(lt>x1)
      {
        lt=0;
        }
        else
        {
          lt+=1;
    }
}
