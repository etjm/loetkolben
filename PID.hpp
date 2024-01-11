
class PID
{
private:
    float m_p;
    float m_i;
    float m_d;
    float m_prevE;
    float m_sum;
    float m_setpoint;
    float m_controlValue;
    bool m_running;

public:
    PID(float p, float i, float d);
    void calc(float (*processVariable)());
    //tbd void calcAndCall(float (*processVariable)(), void (*func)(float u));
    void setP(float p);
    void setI(float I);
    void setD(float D);
    void setParameters(float p,float i,float d);
    float getP();
    float getI();
    float getD();
    void start();
    void stop();
    void resetInt();
    void resetPrev(float prevError);
    void setSetpoint(float setpoint);   // r
    float getControlValue();            // u
};

PID::PID(float p = 0.0, float i = 0.0, float d = 0.0)
{
    m_p = p;
    m_i = i;
    m_d = d; 
    m_prevE = 0.0;
    m_sum = 0.0;
    m_setpoint = 0.0;
    m_controlValue = 0.0;
    m_running = false;
}

void PID::setParameters(float p,float i,float d)
{
    m_p = p;
    m_i = i;
    m_d = d;

}


void PID::setP(float p)
{
    m_p = p;
}
void PID::setI(float i)
{
    m_i = i;
}
void PID::setD(float d)
{
    m_d = d;
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

void PID::calc(float (*processVariable)())
{
    if (m_running)
    {
        float r = m_setpoint;           //
        float y = processVariable();    //
        float e = r - y;                //Current error value

        float p = e * m_p;
        m_sum += e; //+(e*del t)
        float i = m_i * m_sum;

        float d = m_d * (e - m_prevE);  // * t - m_prevT
        m_prevE = e;
        m_controlValue  = p + i + d;
    }
}

void PID::setSetpoint(float setpoint){
    m_setpoint = setpoint;
}

float PID::getControlValue(){
    return m_controlValue;
}

void PID::start()
{
    m_running = true;
}
void PID::stop()
{
    m_running = false;
}