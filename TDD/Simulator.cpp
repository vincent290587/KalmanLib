/*
 * Simulator.cpp
 *
 *  Created on: 21 mei 2019
 *      Author: v.golle
 */

#include "Simulator.hpp"

#include <stdint.h>
#include <string.h>
#include <cmath>
#include <cstdio>
#include <random>
#include "UDMatrix.h"
#include "kalman_ext.h"

#define SIM_DT     0.3f

typedef enum {
	eSlopeStateDownhill,
	eSlopeStateLevel,
	eSlopeStateUphill
} eSlopeState;

typedef struct {
	float alt;
	float speed;
	float acc[3];
	float tilt;
} sSimState;

typedef struct {
	float gps_alt;
	float bar_alt;
	float speed;
	float acc[3];
} sSensorState;

// used as conversion
constexpr long double operator"" _kmh (long double deg_)
{
    return deg_ / 3.6;
}

// used as conversion
constexpr long double operator"" _deg (long double deg_)
{
    return deg_ * 3.141592 / 180;
}

static uint32_t    m_sim_ind = 0;
static eSlopeState m_slo_state = eSlopeStateLevel;
static sSimState   m_sim_state;
static sSensorState m_sens_state;

sKalmanDescr descr;
sKalmanExtFeed feed;

static float _simulate_slope(void) {

	float val = 0.0f;

	if (m_sim_ind++ >= 60) {
		// reset var
		m_sim_ind = 0;

		switch (m_slo_state) {
		case eSlopeStateDownhill:
			m_slo_state = eSlopeStateLevel;
			break;
		case eSlopeStateLevel:
			m_slo_state = eSlopeStateUphill;
			break;
		case eSlopeStateUphill:
			m_slo_state = eSlopeStateDownhill;
			break;
		default:
			break;
		}
	}

	switch (m_slo_state) {
	case eSlopeStateDownhill:
		val = -5.0_deg;
		m_sim_state.speed = 30.0_kmh;
		break;
	case eSlopeStateLevel:
		val = 0.0_deg;
		m_sim_state.speed = 22.0_kmh;
		break;
	case eSlopeStateUphill:
		val = 10.0_deg;
		m_sim_state.speed = 15.0_kmh;
		break;
	default:
		break;
	}

	return val;
}

static void _update_state(float slo) {

	m_sim_state.alt += SIM_DT * m_sim_state.speed * tan(slo);

	m_sim_state.acc[0] = 9.81f * cosf(slo + m_sim_state.tilt);
	m_sim_state.acc[1] = 0.0f;
	m_sim_state.acc[2] = 9.81f * sinf(slo + m_sim_state.tilt);

}

static void _simulate_sensors(void) {

	// create noise

	static std::default_random_engine generator;

	static std::normal_distribution<float> distr_speed(0.0, 0.5_kmh);

	static std::normal_distribution<float> distr_altg(0.0, 1.0);
	static std::normal_distribution<float> distr_altb(153.0, 0.5);

	static std::normal_distribution<float> distr_acc(0.0, 2.0);

	// add noise to all

	m_sens_state.speed = m_sim_state.speed + distr_speed(generator);

	m_sens_state.bar_alt = m_sim_state.alt + distr_altb(generator);

	m_sens_state.gps_alt = m_sim_state.alt + distr_altg(generator);

	for (int i=0; i < 3; i++)
		m_sens_state.acc[i] = m_sim_state.acc[i] + distr_acc(generator);

	printf(">%.3f %.3f %.3f %.3f ",
			m_sens_state.bar_alt,
			m_sens_state.gps_alt,
			m_sens_state.speed,
			m_sim_state.acc[2] / m_sim_state.acc[0]);
}

void simulator_init(void) {

	// reset structs
	memset(&m_sim_state, 0, sizeof(m_sim_state));
	memset(&m_sens_state, 0, sizeof(m_sens_state));

	// set alti
	m_sim_state.alt = 500.f;

	// set tilt
	m_sim_state.tilt = 6.8_deg;

	// Kalman init

	descr.ker.ker_dim = 5;
	descr.ker.obs_dim = 5;

	feed.dt = SIM_DT;

	kalman_lin_init(&descr);

	/*
	 * Observations vector
	 *
	 *      ( h_b
	 * Z =  ( h_gps
	 *      ( v
	 *      ( a_0
	 *      ( h_p
	 *
	 */

	feed.matZ.resize(descr.ker.obs_dim, 1);

	/*
	 * State equations:
	 *
	 * h(n+1) = h(n) + h_p.dt
	 *
	 * h_b   = h + d_h
	 * h_gps = h
	 *
	 *
	 *      ( h
	 *      ( d_h
	 *  X=  ( v
	 *      ( h_p
	 *      ( a_0
	 *
	 */

	// set A (it must be the state transition matrix)
	descr.ker.matA.unity();
	descr.ker.matA.set(0, 3, feed.dt);

	descr.ker.matA.print("A");

	// Innov = Z - C * X
	// It maps observations to the state vector
	descr.ker.matC.ones(0);
	descr.ker.matC.set(0, 0, 1);
	descr.ker.matC.set(0, 1, 1);
	descr.ker.matC.set(1, 0, 1);
	descr.ker.matC.set(2, 2, 1);
	descr.ker.matC.set(3, 4, 1);
	descr.ker.matC.set(4, 3, 1);

	descr.ker.matC.print("C");

	// set Q: model noise (or stability)
	descr.ker.matQ.ones(0);
	descr.ker.matQ.set(0, 0, 0.25);
//	descr.ker.matQ.set(0, 3, 0.05);
//	descr.ker.matQ.set(3, 0, 0.05);
	descr.ker.matQ.set(1, 1, 0.001);
	descr.ker.matQ.set(2, 2, 0.08);
	descr.ker.matQ.set(3, 3, 0.02);
	descr.ker.matQ.set(4, 4, 0.0001);

	// set P
	descr.ker.matP.ones(900);

	// set R: environment noise
	descr.ker.matR.ones(0);
	descr.ker.matR.set(0, 0, 0.04);
	descr.ker.matR.set(1, 1, 1);
	descr.ker.matR.set(2, 2, 0.5_kmh);
	descr.ker.matR.set(3, 3, 2);
	descr.ker.matR.set(4, 4, 2);

	// set X
	descr.ker.matX.set(0, 0, 500);
	descr.ker.matX.set(1, 0, 150);
	descr.ker.matX.set(2, 0, 10.0_kmh);
	descr.ker.matX.set(3, 0, 0);
	descr.ker.matX.set(4, 0, 0);

}

void simulator_task(void) {

	float slope = _simulate_slope();

	_update_state(slope);

	_simulate_sensors();

	// Kalman work

	/*
	 * Observations vector
	 *
	 *      ( h_b
	 * Z =  ( h_gps
	 *      ( speed
	 *      ( a_0
	 *      ( h_p
	 */

	static float meas_a0 = 0;
	static float prev_alt = m_sens_state.bar_alt;

	float meas_hp = m_sens_state.bar_alt - prev_alt;
	float i_h0 = atan2(m_sim_state.acc[2], m_sim_state.acc[0]) - atan2(meas_hp, feed.dt * m_sens_state.speed);
	meas_a0 = 0.1 * meas_a0 + 0.9 * (i_h0);

	prev_alt = m_sens_state.bar_alt;

	meas_hp = m_sens_state.speed * tan(atan2(m_sim_state.acc[2], m_sim_state.acc[0]) - meas_a0);

//	printf("Meas hp: %f\n", meas_hp);

	feed.matZ.set(0, 0, m_sens_state.bar_alt);
	feed.matZ.set(1, 0, m_sens_state.gps_alt);
	feed.matZ.set(2, 0, m_sens_state.speed);
	feed.matZ.set(3, 0, meas_a0);
	feed.matZ.set(4, 0, meas_hp);

	// feed filter
	kalman_lin_feed(&descr, &feed);

	// print state vector
	UDMatrix res;
	res = descr.ker.matX.transpose();
	res.print();

//	exit(0);
}

void simulator_run(void) {

	for (int i=0; i < 1000; i++) simulator_task();
}

