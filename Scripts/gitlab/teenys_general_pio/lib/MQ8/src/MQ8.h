//MQ8.h

class MQ8
{
    private:
        int analogPin;
        int voltage;
        int offset;
    public:
        MQ8();
        int readSensor();
        void setZero();
};