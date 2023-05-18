#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID(double kp, double ki, double kd);
        ~PID();
        void setOutputLimits(double min, double max);
        void setILimit(double iLimit);
        void setDLimit(double dLimit);

        // Returns the manipulated variable given a setpoint and current process value
        double calculate(double setpoint, double pv, double dt);
        void reset();

    private:
        double _kp;
        double _ki;
        double _kd;
        double _error;
        double _pre_error;
        double _integral;
        double _i_limit;
        double _d_limit;
        double _out_min;
        double _out_max;

        bool _limit_output;
};

#endif