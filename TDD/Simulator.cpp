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

#define SIM_DT     0.5f

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
		val = 5.0_deg;
		m_sim_state.speed = 12.5_kmh;
		break;
	default:
		break;
	}

	return val;
}

static void _update_state(float slo) {

	m_sim_state.alt += SIM_DT * m_sim_state.speed * tan(slo);

	m_sim_state.acc[0] = 9.81f * cos(slo + m_sim_state.tilt);
	m_sim_state.acc[1] = 0.0f;
	m_sim_state.acc[2] = 9.81f * sin(slo + m_sim_state.tilt);

}

static void _simulate_sensors(void) {

	// create noise

	static std::default_random_engine generator;

	static std::normal_distribution<float> distr_speed(0.0, 1.5_kmh);

	static std::normal_distribution<float> distr_altg(0.0, 10.0);
	static std::normal_distribution<float> distr_altb(0.0, 0.5);

	static std::normal_distribution<float> distr_acc(0.0, 2.0);

	// add noise to all

	m_sens_state.speed = m_sim_state.speed + distr_speed(generator);

	m_sens_state.bar_alt = m_sim_state.alt + 153. + distr_altb(generator);

	m_sens_state.gps_alt = m_sim_state.alt + distr_altg(generator);

	for (int i=0; i < 3; i++)
		m_sens_state.acc[i] = m_sim_state.acc[i] + distr_acc(generator);

	printf("%.3f %.3f %.3f %.3f\n",
			m_sens_state.speed,
			m_sens_state.bar_alt,
			m_sens_state.acc[0],
			m_sens_state.acc[2]);
}

void simulator_init(void) {

	// reset structs
	memset(&m_sim_state, 0, sizeof(m_sim_state));
	memset(&m_sens_state, 0, sizeof(m_sens_state));

	// set alti
	m_sim_state.alt = 500.f;

	// set tilt
	m_sim_state.tilt = 6.8_deg;

	// TODO Kalman init

	descr.ker.ker_dim = 3;
	descr.ker.obs_dim = 1;

	feed.dt = SIM_DT;

	kalman_ext_init(&descr);

	feed.matZ.resize(1, 1);

	// set A (it must be the jacobian matrix)
	// first val: A1 = dState1 / dState[co]
	descr.ker.matA.unity();

//	descr.ker.matA.set(1, 2, -omega * feed.dt);
//	descr.ker.matA.set(2, 1, omega * feed.dt);

	descr.ker.matA.print();

	// H maps the state vector to the measurements
	// first val: H1 = dMesure1 / dState[co]
	// i.e. expected measurements knowing X
	descr.ker_ext.matH.ones(0.0);

	// set Q
	descr.ker.matQ.unity(1 / 20.);

	// set P
	descr.ker.matP.ones(900);

	// set R
	descr.ker.matR.unity(0.01);

	// set Z
	feed.matZ.ones(0.0);
}

void simulator_task(void) {

	float slope = _simulate_slope();

	_update_state(slope);

	_simulate_sensors();

	// TODO Kalman work

	float val = 14;

	feed.matZ.set(0, 0, val);

	descr.ker_ext.matH.set(0, 0, 1);
	descr.ker_ext.matH.set(0, 1, 2);
	descr.ker_ext.matH.set(0, 2, 3);

	kalman_ext_feed(&descr, &feed);

	UDMatrix res;
	res = descr.ker.matX;
	res.print();
}

void simulator_run(void) {

	for (int i=0; i < 1000; i++) simulator_task();
}

