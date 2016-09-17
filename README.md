# ms5611
MEAS MS5611  pressure sensor library for altitude / variometer applications

Integer code provided in datasheet does not have adequate resolution for variometer (rate-of-climb) applications. So I've modified some of the computations to use floating point operations. This does not mean the improved-resolution  pa measurements are more accurate. You will still need noise-filtering algorithms to process the data. Sliding-window averaging is fine for barometric pressure/altimeter applications, but more sophisticated processing will be required for variometer applications. E.g. Kalman filtering, linear regression etc.

The pressure to altitude conversion is done via look up tables followed by linear interpolation.  This is useful if using an 8bit/16bit processor and the processing time is critical. Accuracy can always be improved by decreasing the lookup table interval (more entries), but at the cost of code space.

