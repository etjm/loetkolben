
class PID
{
private:
    float m_p;
    float m_i;
    float m_d;
    float m_prevE;
    float m_sum;
    int* m_output;
    int m_minOutput;
    int m_maxOutput;
    float* m_input;
    float* m_setpoint;
    int sampleTime;

    unsigned long last_compute;
    
    int m_controlValue;
    bool m_running;

public:
    PID(float* input, int* output, float* setpoint, float p, float i, float d);
    void calc();
    //tbd void calcAndCall(float (*processVariable)(), void (*func)(float u));
    void setP(float p);
    void setI(float I);
    void setD(float D);
    void setParameters(float p,float i,float d);
    void print();

    float getP();
    float getI();
    float getD();
    
    void start();
    void stop();
    void resetInt();
    void resetPrev(float prevError);
    // void setSetpoint(float setpoint);   // r
    float getControlValue();            // u
};

/**
 * @brief Construct a new PID::PID object
 * 
 * @param input 
 * @param output 
 * @param setpoint 
 * @param p 
 * @param i 
 * @param d 
 */
PID::PID(float* input, int* output, float* setpoint, float p, float i, float d)
{
    m_input = input;
    m_output = output;
    m_setpoint = setpoint;
    m_p = p;
    m_i = i;
    m_d = d;
    m_prevE = 0.0;
    m_sum = 0.0;
    
    sampleTime = 100;   //default Controller Sample Time is 0.1 seconds
    // Default limits
    m_minOutput=0;
    m_maxOutput=255;
    m_controlValue = m_minOutput;
    m_running = false;
}

void PID::print()
{
    Serial.print("PID,");
    Serial.print(m_p);
    Serial.print(",");
    Serial.print(m_i);
    Serial.print(",");
    Serial.print(m_d);
    Serial.print(",\nr,y,e,p,i,d,c");
    Serial.println();
    return;
}

/* void PID::setMinMax(int min, int max){
    if(min>max)
        return;
    m_minOutput=min;
    m_maxOutput=max;
} */

void PID::setParameters(float p,float i,float d)
{
    m_p = p;
    m_i = i;
    m_d = d;
}


void PID::setP(float p)
{
  if (p>=0.0)
  {
    m_p = p;
  }
}
void PID::setI(float i)
{
  if (i>=0.0)
  {
    m_i = i;
  }
}
void PID::setD(float d)
{
  if (d>=0.0)
  {
    m_d = d;
  }
}
float PID::getP()
{
    return m_p;
}
float PID::getI()
{
    return m_i;
}
float PID::getD()
{
    return m_d;
}

void PID::resetInt()
{
    m_sum = 0.0;
}

void PID::resetPrev(float prevError = 0.0)
{
    m_prevE = prevError;
}

void PID::calc()
{
    if (!m_running)
        return;
    
    unsigned long compute_time = micros();
    unsigned long timeChange = (compute_time - last_compute);
    
    if(timeChange<sampleTime) return;

    
    float r = *m_setpoint;          // get current setpoint
    float y = *m_input;             // get current input value
    float e = r - y;                // Current error value
    // delta_t = t - m_prevT

    float p = e * m_p;

    // m_sum += e; // + (e*del t)
    // float i = m_i * m_sum;
    m_sum += (e * m_i); //(e * del t)

    // Overrun protection
    if (m_sum > m_maxOutput)
        m_sum = m_maxOutput;
    else if (m_sum < m_minOutput)
        m_sum = m_minOutput;

    float i = m_sum;

    float d = m_d * (e - m_prevE); // * delta_t
    m_prevE = e;
    m_controlValue = round(p + i + d);


    if (m_controlValue>m_maxOutput)
        m_controlValue = m_maxOutput;
    else if(m_controlValue<m_minOutput)
        m_controlValue = m_minOutput;

    *m_output = m_controlValue;
    
    Serial.print(r);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(e);
    Serial.print(",");
    Serial.print(p);
    Serial.print(",");
    Serial.print(i);
    Serial.print(",");
    Serial.print(d);
    Serial.print(",");
    Serial.println(m_controlValue);

    last_compute = compute_time;
}

float PID::getControlValue(){
    return m_controlValue;
}

void PID::start()
{
    // m_sum = *m_output;
    // m_prevE
    m_prevE = 0.0;
    m_sum = 0.0;
    m_running = true;
}

void PID::stop()
{
    m_running = false;
}