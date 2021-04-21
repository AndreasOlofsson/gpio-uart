/* Copyright (C) 2021  Andreas Olofsson
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <linux/types.h>
#include <linux/timekeeping.h>
#include <linux/math64.h>

typedef struct
{
    /// The current level
    int level;
    /// The current read bits of the current byte.
    u_int8_t byte;
    /// The next expected bit to receive, -1 is not currently reading a frame.
    int next_bit;
    /// The time of the first bit of the byte (offset by sample_offset).
    u64 byte_start_time;

    /// The current configured baud rate
    int baud_rate;
    /// Inverse of the current time per bit (shifted by 32 bits to avoid divisions in interrupt).
    /// Calculated as (baud_rate << 32) / NS_PER_S
    u64 inv_time_per_bit;
    /// How much (in ns) to offset each sampling of bits within a frame.
    u64 sample_offset;
    /// The expected time of a full frame, used to determine when a frame has ended.
    u64 byte_length;
} uart;

/// Nanoseconds per second
#define NS_PER_S 1000000000

void uart_init(uart *uart, int baud_rate)
{
    int time_per_bit = div64_ul(NS_PER_S, baud_rate);

    uart->level = 1;
    uart->next_bit = -1;
    uart->baud_rate = baud_rate;
    uart->inv_time_per_bit = div64_ul(((u64)baud_rate) << 32, NS_PER_S);
    uart->sample_offset = time_per_bit >> 1;
    uart->byte_length = time_per_bit * 9 + (time_per_bit >> 1);
}

/// Update the uart state with a level transition, high if this is a rising edge, otherwise falling edge.
///
/// Returns -1 if no byte has been read, otherwise returns the byte value in the lower 8 bits.
int uart_update_state(uart *uart, int high)
{
    u64 now = ktime_get_ns();
    int ret = -1;

    if (uart->next_bit == -1)
    {
        if (!high)
        {
            // Start of byte
            uart->next_bit = 0;
            uart->byte = 0;
            uart->level = 0;
            uart->byte_start_time = now + uart->sample_offset;
        }

        return -1;
    }
    else
    {
        int current_bit = ((now - uart->byte_start_time) * uart->inv_time_per_bit) >> 32;

        // Set bits unchanged since last level change
        int i;
        for (i = uart->next_bit; i < min(current_bit, 8); i++)
        {
            if (uart->level)
            {
                uart->byte |= 1 << i;
            }
        }

        if (current_bit >= 8)
        {
            if (high || uart->level)
            {
                // End of byte
                ret = uart->byte;
                if (!high && current_bit >= 9)
                {
                    // Also start of next byte
                    uart->next_bit = 0;
                    uart->byte = 0;
                    uart->level = 0;
                    uart->byte_start_time = now + uart->sample_offset;
                }
                else
                {
                    uart->next_bit = -1;
                }
            }
            else
            {
                // Too late for a level change, discard the invalid byte.
                uart->next_bit = -1;
            }
        }
        else
        {
            if (high)
            {
                uart->byte |= 1 << current_bit;
            }
            uart->next_bit = current_bit + 1;
        }
        uart->level = high;
    }

    return ret;
}

/// Try to finish a currently read byte.
/// This is needed in case the frame ends in a high bit, in which case no level transition will be sent to end the frame.
///
/// Returns -1 if no byte has been read, otherwise returns the byte value in the lower 8 bits.
int uart_finish_byte(uart *uart)
{
    int i;

    if (uart->next_bit == -1)
    {
        // Not currently reading a byte
        return -1;
    }

    if (ktime_get_ns() < uart->byte_start_time + uart->byte_length)
    {
        // Byte not yet finished
        return -1;
    }

    if (!uart->level)
    {
        // Byte should have been finished by now, discard the invalid byte.
        uart->next_bit = -1;
        return -1;
    }

    // Set remaining bits to last level
    for (i = uart->next_bit; i < 8; i++)
    {
        if (uart->level)
        {
            uart->byte |= 1 << i;
        }
    }

    uart->next_bit = -1;
    return uart->byte;
}