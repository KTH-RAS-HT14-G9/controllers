#ifndef PID_H
#define PID_H

#include <eigen3/Eigen/Core>

using namespace Eigen;

/**
  * Generic pid controller to be used on vectors.
  */
template <int nParams>
class pid
{
public:

    typedef Matrix<double, nParams, 1> Params;

    pid(const Params& k_p, const Params& k_i, const Params& k_d)
        :_kp(k_p)
        ,_ki(k_i)
        ,_kd(k_d)
    {
    }

    /**
      * Performs update step.
      * Derivative defined as follows:
      * d/dt e(t) = (e(t-1)-e(t))/dt
      *
      * @param state        Given state at timestep t
      * @param target       Given target state at timestep t
      * @param dt           Time difference between t and t-1.
      *                     (Time difference between two control() calls).
      *                     If dt = 0, the D-term is ignored.
      */
    Params control(const Params& state, const Params& target, double dt);

    /**
      * Resets the error accumulation of the i-Term.
      * Not necessary to call, if k_i = 0.
      */
    void reset();

    Params get_kp() { return _kp; }
    Params get_ki() { return _ki; }
    Params get_kd() { return _kd; }

    void set_kp(Params k_p) { _kp = Params(k_p); }
    void set_ki(Params k_i) { _ki = Params(k_i); }
    void set_kd(Params k_d) { _kd = Params(k_d); }

protected:

    Params _kp, _ki, _kd;
    Params _cum_error;
    Params _error;
    Params _last_error;
};

/**
  * Static 1D p and pd-controller functions.
  */
class pd {
public:
    /**
      * Simple 1D p-controller. Can be used if neither an instance nor the d or i part is used.
      */
    inline static double P_control(double k_p, double state, double target) {
        return k_p*(target-state);
    }

    /**
      * Simple 1D pd-controller. Can be used if neither an instance nor the i part is used.
      * Derivative defined as follows:
      * d/dt e(t) = (e(t-1)-e(t))/dt
      *
      * @param state_t      State at timestep t
      * @param target_t     Target state at timestep t
      * @param state_t_1    State at timestep t-1
      * @param target_t_1   Target state at timestep t-1
      * @param dt           Time difference between t and t-1. If dt = 0, the D-term is ignored
      */
    inline static double PD_control(double k_p, double k_d, double state_t, double target_t, double state_t_1, double target_t_1, double dt) {
        if (dt <= 0)
            return P_control(k_p, state_t, target_t);
        else
            return P_control(k_p, state_t, target_t) +
                k_d*( ((target_t-state_t) - (target_t_1-state_t_1))/dt );
    }
private:
    pd(){}
    ~pd(){}
};

//------------------------------------------------------------------------------
// Implementations

template<int nParams>
void pid<nParams>::reset()
{
    _cum_error.setZero();
}

template<int nParams>
Matrix<double, nParams, 1> pid<nParams>::control(const Params& state,
                                                 const Params& target,
                                                 double dt)
{
    _error = target - state;
    _cum_error += _error;

    if (dt > 0) // use pid
        return _error.cwiseProduct(_kp) + _cum_error.cwiseProduct(_ki) + (_last_error - _error).cwiseProduct(_kd/dt);
    else        // use only pi
        return _error.cwiseProduct(_kp) + _cum_error.cwiseProduct(_ki);
}

//------------------------------------------------------------------------------
// Convenience class for a 1d pid

class pid_1d : public pid<1>
{
public:
    pid_1d(double k_p, double k_i, double k_d)
        :pid<1>(pid<1>::Params(k_p), pid<1>::Params(k_i), pid<1>::Params(k_d))
    {
    }

    /**
      * Adapter function for pid<1>::control().
      */
    double control_1d(double state, double target, double dt) {
        const pid<1>::Params& u = pid<1>::control(pid<1>::Params(state),pid<1>::Params(target),dt);
        return u(0,0);
    }

    double get_kp_1d() { return _kp(0,0); }
    double get_ki_1d() { return _ki(0,0); }
    double get_kd_1d() { return _kd(0,0); }

    void set_kp_1d(double k_p) { _kp(0,0) = k_p; }
    void set_ki_1d(double k_i) { _ki(0,0) = k_i; }
    void set_kd_1d(double k_d) { _kd(0,0) = k_d; }
};

#endif // PID_H
