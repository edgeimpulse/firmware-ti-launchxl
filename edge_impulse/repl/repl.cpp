/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/* Include ----------------------------------------------------------------- */
#include "repl.h"

#include "ei_device_ti_launchxl.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
// #include "at_cmd_interface.h"

#include <stdio.h>
#include <string>
#include <vector>


extern bool ei_at_cmd_handle(const char *cmd_in);
extern "C" void Serial_Out(char *string, int length);

using namespace std;

class ReplBuffer {
public:
    ReplBuffer() {
        position = 0;
    }

    void clear() {
        buffer.clear();
        position = 0;
    }

    void add(string s) {
        for(string::iterator it = s.begin(); it != s.end(); ++it) {
            buffer.insert(buffer.begin() + position, *it);
            position++;
        }
    }

    void add(char c) {
        buffer.insert(buffer.begin() + position, c);
        position++;
    }

    vector<char>::iterator begin() {
        return buffer.begin();
    }

    vector<char>::iterator end() {
        return buffer.end();
    }

    size_t getPosition() {
        return position;
    }

    void setPosition(size_t pos) {
        position = pos;
    }

    size_t size() {
        return buffer.size();
    }

private:
    vector<char> buffer;
    size_t position;
};

/* Private variables ------------------------------------------------------- */
static bool inControlChar = false;
static vector<char> controlSequence;
static vector<string> history;
static size_t historyPosition;
static ReplBuffer buffer;

#define MAX_HISTORY 5

/* Private functions prototypes -------------------------------------------- */
static void handleBackspace(void);
static void runBuffer(void);

void rx_callback(char c) {
    // control characters start with 0x1b and end with a-zA-Z
    if (inControlChar) {

        controlSequence.push_back(c);

        // if a-zA-Z then it's the last one in the control char...
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
            inControlChar = false;

            // up
            if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x41) {
                ei_printf("\033[u"); // restore current position

                if (historyPosition == 0) {
                    // cannot do...
                }
                else {
                    historyPosition--;
                    // reset cursor to 0, do \r, then write the new command...
                    ei_printf("\33[2K\r> %s", history[historyPosition].c_str());

                    buffer.clear();
                    buffer.add(history[historyPosition]);
                }
            }
            // down
            else if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x42) {
                ei_printf("\033[u"); // restore current position

                if (historyPosition == history.size()) {
                    // no-op
                }
                else if (historyPosition == history.size() - 1) {
                    historyPosition++;

                    // put empty
                    // reset cursor to 0, do \r, then write the new command...
                    ei_printf("\33[2K\r> ");

                    buffer.clear();
                }
                else {
                    historyPosition++;
                    // reset cursor to 0, do \r, then write the new command...
                    ei_printf("\33[2K\r> %s", history[historyPosition].c_str());

                    buffer.clear();
                    buffer.add(history[historyPosition]);
                }
            }
            // left
            else if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x44) {
                size_t curr = buffer.getPosition();

                // at pos0? prevent moving to the left
                if (curr == 0) {
                    ei_printf("\033[u"); // restore current position
                }
                // otherwise it's OK, move the cursor back
                else {
                    buffer.setPosition(curr - 1);

                    ei_putchar('\033');
                    for (size_t ix = 0; ix < controlSequence.size(); ix++) {
                        ei_putchar(controlSequence[ix]);
                    }
                }
            }
            // right
            else if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x43) {
                size_t curr = buffer.getPosition();
                size_t size = buffer.size();

                // already at the end?
                if (curr == size) {
                    ei_printf("\033[u"); // restore current position
                }
                else {
                    buffer.setPosition(curr + 1);

                    ei_putchar('\033');
                    for (size_t ix = 0; ix < controlSequence.size(); ix++) {
                        ei_putchar(controlSequence[ix]);
                    }
                }
            }
            else {
                // not up/down? Execute original control sequence
                ei_putchar('\033');
                for (size_t ix = 0; ix < controlSequence.size(); ix++) {
                    ei_putchar(controlSequence[ix]);
                }
            }

            controlSequence.clear();
        }

        return;
    }

    switch (c) {
        case '\r': /* want to run the buffer */
            ei_putchar(c);
            ei_putchar('\n');
            runBuffer();
            break;
        case '\n': /* Ignore newline as input */
            break;
        case 0x08: /* backspace */
        case 0x7f: /* also backspace on some terminals */
            handleBackspace();
            break;
        case 0x1b: /* control character */
            // wait until next a-zA-Z
            inControlChar = true;                
            ei_printf("\033[s"); // save current position


            break; /* break out of the callback (ignore all other characters) */
        default:        	
            size_t curr_pos = buffer.getPosition();
            size_t buffer_size = buffer.size();            
            if (curr_pos == buffer_size) {
                buffer.add(c);
                ei_putchar(c);
            }
            else {
                // super inefficient...
                string v(buffer.begin(), buffer.end());
                v.insert(curr_pos, 1, c);

                buffer.clear();
                buffer.add(v);

                buffer.setPosition(curr_pos + 1);

                ei_printf("\r> %s\033[%dG", v.c_str(), int(curr_pos) + 4);
            }
            break;
    }
}

static void handleBackspace(void)
{
    size_t curr_pos = buffer.getPosition();

    string v(buffer.begin(), buffer.end());

    if (v.size() == 0 || curr_pos == 0) return;

    bool endOfLine = curr_pos == v.size();

    v.erase(curr_pos - 1, 1);

    buffer.clear();
    buffer.add(v);

    buffer.setPosition(curr_pos - 1);

    if (endOfLine) {
        ei_printf("\b \b");
    }
    else {
        // carriage return, new text, set cursor, empty until end of line
        ei_printf("\r\033[K> %s\033[%dG", v.c_str(), curr_pos + 2);
    }
}

static void runBuffer(void)
{
    string rawCode(buffer.begin(), buffer.end());

    //pc->printf("Running: %s\r\n", rawCode.c_str());

    if (history.size() == MAX_HISTORY) {

        // copy
        vector<string> hist_copy;
        unsigned int i;
        for (i=1; i < history.size(); i++) {
            hist_copy.push_back(history.at(i));
        }
        hist_copy.push_back(rawCode);

        //restore
        history.clear();
        for (i=0; i < hist_copy.size(); i++) {
            history.push_back(hist_copy.at(i));
        }
        historyPosition = history.size();

    } else {
        history.push_back(rawCode);
        historyPosition = history.size();
    }

    // pc->printf("Executing (%s): ", rawCode.c_str());
    // for (size_t ix = 0; ix < rawCode.size(); ix++) {
    //     pc->printf(" %02x ", rawCode.at(ix));
    // }
    // pc->printf("\r\n");

    bool printNewState = true;
    // if (commandCallback) {
    //     printNewState = commandCallback(rawCode.c_str());
    // }  
    printNewState = ei_at_cmd_handle(rawCode.c_str());

    buffer.clear();

    if (printNewState) {
        Serial_Out((char *)"> ",sizeof("> "));
    }
}
