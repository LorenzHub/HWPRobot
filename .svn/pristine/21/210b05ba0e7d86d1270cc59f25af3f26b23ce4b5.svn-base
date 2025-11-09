/**
 * @file remoteDataProcessing.h
 * @ingroup tools
 *
 * Interface with a remote data processor (such as MATLAB, Octave, Scilab)
 * through HWPCS. Arbitrary data and commands for processing can be transferred
 * and executed by the remote data processor. This can for example be useful for
 * calibration or collection and processing of experimental data.
 */

#ifndef REMOTEDATAPROCESSING_H_
#define REMOTEDATAPROCESSING_H_

#include <stdbool.h>
#include <stdint.h>


/**
 * Send a command on communication channel #CH_OUT_RDP to a remote data
 * processor (for example MATLAB, Octave, Scilab) through HWPCS. The commands
 * are added to a list (a script to be executed by a configured data processor)
 * within HWPCS as long as the parameter execute is false. In order to add
 * the last command and execute the whole script, set the parameter execute
 * to true.
 *
 * Note that an internal buffer of 514 bytes needs to be temporarily
 * allocated during this function.
 *
 * @param   execute   false to just add the command to the list,
 *                    true to add the command and execute the whole script
 * @param   command   a string containing the command, for format please see
 *                    <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html" target="_blank">vfprintf</a>
 *                    from AVR Libc.
 *                    The resulting string will be truncated after 512 characters
 *                    excluding null termination.
 * @return  false if no memory for an internal buffer could be allocated, true on success
 */
bool remoteDataProcessing_command(const bool execute, const char* command, ...);


/**
 * Send a command on communication channel #CH_OUT_RDP to a remote data
 * processor (for example MATLAB, Octave, Scilab) through HWPCS. The commands
 * are added to a list (a script to be executed by a configured data processor)
 * within HWPCS as long as the parameter execute is false. In order to add
 * the last command and execute the whole script, set the parameter execute
 * to true.
 *
 * Note that an internal buffer of maxLength+2 bytes needs to be temporarily
 * allocated during this function.
 *
 * This function behaves identical to remoteDataProcessing_command() but allows
 * to specify the size of the internal buffer.
 *
 * @param   maxLength maximum size of resulting command string excluding
 *                    null termination. Am internal buffer of size maxLength + 2
 *                    is temporarily allocated during this function.
 * @param   execute   false to just add the command to the list,
 *                    true to add the command and execute the whole script
 * @param   command   a string containing the command, for format please see
 *                    <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html" target="_blank">vfprintf</a>
 *                    from AVR Libc.
 *                    The resulting string will be truncated after maxLength characters
 *                    excluding null termination.
 * @return  false if no memory for an internal buffer could be allocated, true on success
 */
bool remoteDataProcessing_command_maxLength(const uint16_t maxLength, const bool execute, const char* command, ...);


/**
 * Clear the list of commands (the script) contained in HWPCS. This function
 * should be used in order to start a new script and before commands are added
 * with remoteDataProcessing_command().
 */
void remoteDataProcessing_clear(void);

#endif /* REMOTEDATAPROCESSING_H_ */
