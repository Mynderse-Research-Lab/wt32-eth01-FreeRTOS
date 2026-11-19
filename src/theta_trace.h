/**
 * @file theta_trace.h
 * @brief 1 kHz trace of one theta move (HCS01 Class 1 feedback), console `thtrace`.
 */

#ifndef THETA_TRACE_H
#define THETA_TRACE_H

#include <cstddef>

#include "Gantry.h"

// thtrace arm             - record the next theta move (any source: move, pick)
// thtrace                 - summary of the last recorded move
// thtrace raw <start> <n> - samples as log lines (t_us:rel_deg:vel_deg_s:status), <= 300 per call
// thtrace off             - disarm or stop a recording
// Writes one line into out. False on a usage error or when there is no trace.
bool thetaTraceCommand(Gantry::Gantry *gantry, const char *args, char *out, size_t out_len);

#endif  // THETA_TRACE_H
