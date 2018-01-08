#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <asm/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <rtdm/driver.h>

struct ring_buffer {
    uint8_t *buf;
    unsigned long long  size;
    unsigned long long  head;
    unsigned long long  tail;
    atomic_long_t  cur_size ;
};

int ring_buffer_init(struct ring_buffer* ring, unsigned long long size)
{
    ring->buf = (uint8_t*)rtdm_malloc(size);
    if (!ring->buf)
        return -ENOMEM;
    atomic_long_set(&ring->cur_size, 0);
    ring->size = size;
    ring->head = 0;
    ring->tail = 0;
    return 0;
}

void ring_buffer_destroy(struct ring_buffer *ring)
{
    ring->size = 0;
    ring->head = 0;
    ring->tail = 0;
    rtdm_free(ring->buf);
}

void ring_buffer_push_back(struct ring_buffer *ring, uint8_t d)
{
    ring->buf[ring->tail] = d;

    ring->tail++;
    if (ring->tail >= ring->size) {
        ring->tail = 0;
    }

    atomic_long_inc(&ring->cur_size);
    if (atomic_long_read(&ring->cur_size) > ring->size)
        atomic_long_set(&ring->cur_size, ring->size);

    if (atomic_long_read(&ring->cur_size) == ring->size) {
        ring->head++;
        if (ring->head == ring->size) {
            ring->head = 0;
        }
    }
}

void ring_buffer_npush_back(struct ring_buffer *ring, uint8_t *buf, unsigned long long  size)
{
    unsigned long long  i = 0;
    for (; i < size; i++) {
        ring_buffer_push_back(ring, buf[i]);
    }
}

uint8_t ring_buffer_get(struct ring_buffer *ring)
{
    char c = ring->buf[ring->head];
    atomic_long_dec(&ring->cur_size);
    ring->head++;
    if (ring->head == ring->size)
        ring->head = 0;

    return c;
}

unsigned long long  ring_buffer_get_n(struct ring_buffer *ring, uint8_t *buf, unsigned long long  size)
{
    long long i;
    if (size > atomic_long_read(&ring->cur_size))
        size = atomic_long_read(&ring->cur_size);
    for (i = 0; i < size; i++) {
        buf[i] = ring_buffer_get(ring);
    }
    return size;
}

unsigned long long  ring_buffer_is_empty(struct ring_buffer *ring)
{
    if (atomic_long_read(&ring->cur_size) <= 0)
        return 1;
    return 0;
}

#endif