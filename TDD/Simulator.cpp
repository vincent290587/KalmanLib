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
#include <random>

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
	float alt;
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

static float _simulate_slope(void) {

	float val = 0.0f;

	if (m_sim_ind++ >= 60) {
		// reset var
		m_sim_ind = 0;

		m_slo_state = (eSlopeState)((uint32_t)m_slo_state+1);
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
	static std::normal_distribution<float> distr_alt(0.0, 1.0);

	static std::uniform_real_distribution<float> distr_acc(0.0, 2.0);

	// add noise to all

	m_sens_state.speed = m_sim_state.speed + distr_speed(generator);

	m_sens_state.alt = m_sim_state.alt + distr_alt(generator);

	for (int i=0; i < 3; i++)
		m_sens_state.acc[i] = m_sim_state.acc[i] + distr_acc(generator);
}

void simulator_init(void) {

	// reset structs
	memset(&m_sim_state, 0, sizeof(m_sim_state));
	memset(&m_sens_state, 0, sizeof(m_sens_state));

	// set alti
	m_sim_state.alt = 500.f;

	// set tilt
	m_sim_state.tilt = 6.8_deg;
}

void simulator_task(void) {

	float slope = _simulate_slope();

	_update_state(slope);

	_simulate_sensors();

	// TODO Kalman work
}

void simulator_run(void) {

	for (int i=0; i < 1000; i++) simulator_task();
}

