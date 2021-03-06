/*
 * main_tdd.cpp
 *
 *  Created on: 17 sept. 2018
 *      Author: Vincent
 */

#pragma STDC FENV_ACCESS on
#define _GNU_SOURCE 1

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "segger_wrapper.h"
#include "unit_testing.hpp"
#include "Simulator.hpp"


#include <fenv.h> // For feenableexcept
#include <execinfo.h> // For backtrace and backtrace_symbols_fd
#include <unistd.h> // For STDERR_FILENO
#include <signal.h> // To register the signal handler

void print_backtrace(void)
{
        static const char start[] = "BACKTRACE ------------\n";
        static const char end[] = "----------------------\n";

        void *bt[1024];
        int bt_size;
        char **bt_syms;
        int i;

        bt_size = backtrace(bt, 1024);
        bt_syms = backtrace_symbols(bt, bt_size);
        write(STDERR_FILENO, start, strlen(start));
        for (i = 1; i < bt_size; i++) {
                size_t len = strlen(bt_syms[i]);
                write(STDERR_FILENO, bt_syms[i], len);
                write(STDERR_FILENO, "\n", 1);
        }
        write(STDERR_FILENO, end, strlen(end));
    free(bt_syms);
}

void signalHandler( int signum ) {

    printf("Interrupt signal %d received.\n", signum);

    // Get a back trace
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    backtrace_symbols_fd(array, size, STDERR_FILENO);

    print_backtrace();

    // cleanup and close up stuff here
    // terminate program

    exit(signum);
}

void pipeHandler( int signum ) {
	printf("Screen shutdown detected\n");
    exit(signum);
}

void app_shutdown(void) {
	LOG_INFO("App shutdown now");
	exit(0);
}

/**
 *
 * @return 0
 */
int main(void)
{
	// Enable exceptions for certain floating point results
	feenableexcept(FE_INVALID   |
			FE_DIVBYZERO |
			FE_OVERFLOW  |
			FE_UNDERFLOW);

	// Install the trap handler
	// register signal SIGINT and signal handler
	signal(SIGFPE, signalHandler);
	signal(SIGSEGV, signalHandler);
	signal(SIGABRT, signalHandler);

	// pipe handler
	signal(SIGPIPE, pipeHandler);

	LOG_INFO("Unit testing...");

	if (!test_kalman_ext()) {
		exit(__LINE__);
	}

	simulator_init();

	simulator_run();

	LOG_INFO("End of program");

	return 0;
}


