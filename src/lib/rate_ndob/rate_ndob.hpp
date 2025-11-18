#pragma once

#include <matrix/matrix/math.hpp>

using namespace matrix;

class RateNDOB {
public:
    RateNDOB() : d_hat_(Vector3f{0.f, 0.f, 0.f})
    {
        J_.setIdentity();
    	K_.setIdentity();
    }

    void setInertia(const Vector3f& J) {
        Matrix3f J_diag;
        J_diag.setIdentity();

        for(size_t i = 0; i < 3; i++) {
            J_diag(i, i) = J(i);
        }

	    J_ = J_diag;
    }

    void setGeometry(float cx, float cy) {
	    cx_ = cx;
	    cy_ = cy;
    }

    void setMotor(float k1, float k2) {
	    k1_ = k1;
	    k2_ = k2;
    }

    void setGainMatrix(const Vector3f& K) {
	    K_(0, 0) = K(0);
	    K_(1, 1) = K(1);
	    K_(2, 2) = K(2);
    }

    void setFilter(float cutoff_freq) {
	    cutoff_freq_ = cutoff_freq;
    }

    void reset() {
        d_hat_.setZero();
        disturbance_prev.setZero();
    }

    void update(float dt, const Vector4f &u, const Vector3f &omg, const Vector3f &omg_d, Vector3f& d) {
        Vector4f force_motor = {0.0f, 0.0f, 0.0f, 0.0f};

        for(size_t i = 0; i < 4; i++) {
            force_motor(i) = k1_ * u(i) * u(i) + k2_ * u(i);
        }

        Vector3f torque{0.0f, 0.0f, 0.0f};

        torque(0) = -force_motor(0) * cy_ + force_motor(1) * cy_ + force_motor(2) * cy_ - force_motor(3) * cy_;
        torque(1) =  force_motor(0) * cx_ - force_motor(1) * cx_ + force_motor(2) * cx_ - force_motor(3) * cx_;
        torque(2) = 0; // not use now.

        // update
        Vector3f disturbance{0.0f, 0.0f, 0.0f};
        updateDisturbance(dt, torque, omg, omg_d, disturbance);
        d = disturbance;
    }



private:
    void updateDisturbance(float dt, Vector3f& torque, const Vector3f &omg, const Vector3f &omg_d, Vector3f& disturbance) {

        disturbance = K_ * (J_ * omg_d + omg.cross(J_*omg) - torque);

        // add a alpha filter
        float rc = 1.0f/ (2.0f * M_PI_F * cutoff_freq_);
        float alpha = dt / (rc + 0.004f);

        for (size_t i = 0; i < 3; i++) {
             disturbance(i) = alpha * disturbance(i) + (1-alpha) * disturbance_prev(i);
        }

        disturbance_prev = disturbance;
    }

    float cx_{0.0f}, cy_{0.0f};
    float k1_, k2_;
    Vector3f disturbance_prev{0.0f, 0.0f, 0.0f};
    Vector3f d_hat_;
    Matrix3f J_;
    Matrix3f K_;
    float cutoff_freq_{100.0f};
};
