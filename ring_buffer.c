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
#include <linux/minmax.h>

#define RING_BUFFER_SIZE 4096
#define RING_BUFFER_WRAP(n) ((n) % RING_BUFFER_SIZE)

typedef struct
{
    u_int8_t buffer[RING_BUFFER_SIZE];
    int read;
    int write;
} ring_buffer;

/// Copy from ring buffer to user space
/// Returns number of bytes copied
int ring_buffer_copy_to_user(ring_buffer *ring, u_int8_t *buffer, int max_length)
{
    if (ring->read < ring->write)
    {
        int not_copied, copied;
        max_length = min(max_length, ring->write - ring->read);
        not_copied = copy_to_user(buffer, &ring->buffer[ring->read], max_length);
        copied = max_length - not_copied;
        ring->read = RING_BUFFER_WRAP(ring->read + copied);
        return copied;
    }
    else if (ring->read > ring->write)
    {
        int max_length_1 = min(max_length, RING_BUFFER_SIZE - ring->read);
        int not_copied_1 = copy_to_user(buffer, &ring->buffer[ring->read], max_length_1);
        int copied_1 = max_length_1 - not_copied_1;
        ring->read = RING_BUFFER_WRAP(ring->read + copied_1);
        if (copied_1 < max_length_1)
        {
            return copied_1;
        }
        else
        {
            int max_length_2 = min(max_length - max_length_1, ring->write - ring->read);
            int not_copied_2 = copy_to_user(buffer, &ring->buffer[ring->read], max_length_2);
            int copied_2 = max_length_2 - not_copied_2;
            ring->read = RING_BUFFER_WRAP(ring->read + copied_2);
            return copied_1 + copied_2;
        }
    }
    else
    {
        // Buffer is empty
        return 0;
    }
}

bool ring_buffer_write_byte(ring_buffer *ring, u_int8_t b)
{
    int next_index = RING_BUFFER_WRAP(ring->write + 1);

    if (next_index == ring->read)
    {
        return false;
    }

    ring->buffer[ring->write] = b;
    ring->write = next_index;
    return true;
}