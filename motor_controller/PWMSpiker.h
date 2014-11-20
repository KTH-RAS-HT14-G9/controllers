#ifndef PWMSPIKER_H
#define PWMSPIKER_H

#define SIGN(x) ( (x) < 0 ? -1.0 : ((x) > 0 ? 1.0 : 0.0) )

class PWMSpiker {
public:

    PWMSpiker()
    {
    }

    void set(int power_pwm, int sustain_pwm)
    {
        _power_pwm = power_pwm;
        _sustain_pwm = sustain_pwm;
    }

    int apply(double w_estimate, double w_desired)
    {
        if (w_desired > 0) {
            if (std::abs(w_estimate) < 1e-6)
                return SIGN(w_desired)*_power_pwm;
            else
                return SIGN(w_desired)*_sustain_pwm;
        }
        return 0;
    }

protected:
    int _power_pwm;
    int _sustain_pwm;
};

#endif // PWMSPIKER_H
