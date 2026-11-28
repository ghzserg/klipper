// Commands for sending messages to a TMC2208 via its single wire UART
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_MACH_N32G455
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#if CONFIG_MACH_N32G455
#include "tmcuart_sg.h" // tmcuart_sg_decode
#endif

struct tmcuart_s {
    struct timer timer;
    struct gpio_out tx_pin;
    struct gpio_in rx_pin;
    uint8_t flags;
    uint8_t pos, read_count, write_count;
    uint32_t cfg_bit_time, bit_time;
    uint8_t data[10];
};

enum {
    TU_LINE_HIGH = 1<<0, TU_ACTIVE = 1<<1, TU_READ_SYNC = 1<<2,
    TU_REPORT = 1<<3, TU_PULLUP = 1<<4, TU_SINGLE_WIRE = 1<<5
};

static struct task_wake tmcuart_wake;

#if CONFIG_MACH_N32G455
// Буфер выделяется только после штатной очереди движений.
#define TU_BUFFER_MAX 6000
#define TU_BUFFER_SAMPLE 6
static struct {
    struct tmcuart_s *owner;
    uint8_t *data;
    uint32_t interval, next_clock, deadline, available;
    volatile uint32_t tx, rx, end;
    uint32_t query_min, query_max;
    uint16_t token, count, total, errors;
    uint8_t active, request[5];
} burst;
#endif

// Restore uart line to normal "idle" mode
static void
tmcuart_reset_line(struct tmcuart_s *t)
{
    if (t->flags & TU_SINGLE_WIRE)
        gpio_out_reset(t->tx_pin, 1);
    else
        gpio_out_write(t->tx_pin, 1);
    t->flags = (t->flags & (TU_PULLUP | TU_SINGLE_WIRE)) | TU_LINE_HIGH;
}

// Helper function to end a transmission and schedule a response
static uint_fast8_t
tmcuart_finalize(struct tmcuart_s *t)
{
#if CONFIG_MACH_N32G455
    if (burst.active && burst.owner == t)
        burst.end = timer_read_time();
#endif
    tmcuart_reset_line(t);
    t->flags |= TU_REPORT;
    sched_wake_task(&tmcuart_wake);
    return SF_DONE;
}

// Event handler for reading uart bits
static uint_fast8_t
tmcuart_read_event(struct timer *timer)
{
    struct tmcuart_s *t = container_of(timer, struct tmcuart_s, timer);
    uint8_t v = gpio_in_read(t->rx_pin);
    // Read and store bit
    uint8_t pos = t->pos, mask = 1 << (pos & 0x07), data = t->data[pos >> 3];
    if (v)
        data |= mask;
    else
        data &= ~mask;
    t->data[pos >> 3] = data;
    pos++;
    if (pos >= t->read_count)
        return tmcuart_finalize(t);
    t->pos = pos;
    t->timer.waketime += t->bit_time;
    return SF_RESCHEDULE;
}

// Event handler for detecting start of data reception
static uint_fast8_t
tmcuart_read_sync_event(struct timer *timer)
{
    struct tmcuart_s *t = container_of(timer, struct tmcuart_s, timer);
    uint8_t v = gpio_in_read(t->rx_pin);
    if (v) {
        t->flags |= TU_READ_SYNC;
    } else if (t->flags & TU_READ_SYNC) {
        // Now synchronized - begin reading
#if CONFIG_MACH_N32G455
        if (burst.active && burst.owner == t)
            burst.rx = timer_read_time();
#endif
        t->pos = 0;
        t->timer.func = tmcuart_read_event;
        return tmcuart_read_event(timer);
    }
    if (t->pos++ >= 64) {
        // Timeout
        t->read_count = 0;
        return tmcuart_finalize(t);
    }
    t->timer.waketime += t->bit_time;
    return SF_RESCHEDULE;
}

// Event handler called at end of uart writing
static uint_fast8_t
tmcuart_send_finish_event(struct timer *timer)
{
    struct tmcuart_s *t = container_of(timer, struct tmcuart_s, timer);
    if (!t->read_count)
        // This is a tx only operation - success
        return tmcuart_finalize(t);
    // Prepare for message rx
    if (t->flags & TU_SINGLE_WIRE)
        gpio_in_reset(t->rx_pin, t->flags & TU_PULLUP);
    t->pos = 0;
    t->timer.func = tmcuart_read_sync_event;
    t->timer.waketime += t->bit_time * 4;
    return SF_RESCHEDULE;
}

// Event handler for sending uart bits
static uint_fast8_t
tmcuart_send_event(struct timer *timer)
{
    struct tmcuart_s *t = container_of(timer, struct tmcuart_s, timer);
    // Toggle uart output
    gpio_out_toggle_noirq(t->tx_pin);
    t->flags ^= TU_LINE_HIGH;
    // Determine next toggle time
    uint8_t line_state = t->flags & TU_LINE_HIGH;
    uint32_t bit_time = t->bit_time, next = bit_time;
    uint8_t pos = t->pos;
    for (;;) {
        pos++;
        if (pos >= t->write_count) {
            // No more toggles necessary - schedule finish event
            t->timer.func = tmcuart_send_finish_event;
            t->timer.waketime += next;
            return SF_RESCHEDULE;
        }
        uint8_t data = t->data[pos >> 3], bit = (data >> (pos & 0x07)) & 0x01;
        if (bit != line_state)
            break;
        next += bit_time;
    }
    t->pos = pos;
    t->timer.waketime += next;
    return SF_RESCHEDULE;
}

// Event handler for sending sync nibble with enhanced baud detection
static uint_fast8_t
tmcuart_send_sync_event(struct timer *timer)
{
    struct tmcuart_s *t = container_of(timer, struct tmcuart_s, timer);
    // Toggle uart output and note toggle time
    gpio_out_toggle_noirq(t->tx_pin);
    uint32_t cur = timer_read_time();
    t->flags ^= TU_LINE_HIGH;
    // Determine next wakeup time
    t->pos++;
    if (t->pos == 1) {
#if CONFIG_MACH_N32G455
        if (burst.active && burst.owner == t) {
            burst.tx = cur;
            burst.rx = 0;
        }
#endif
        // First bit just sent - record scheduling offset for later use
        uint32_t offset = cur - t->timer.waketime;
        t->bit_time = offset;
    } else if (t->pos >= 5) {
        // Last bit of sync nibble just sent - calculate actual baud rate used
        uint32_t offset = cur - t->timer.waketime, start_offset = t->bit_time;
        int32_t diff = offset - start_offset;
        t->bit_time = t->cfg_bit_time + DIV_ROUND_CLOSEST(diff, 4);
        t->timer.func = tmcuart_send_event;
        t->timer.waketime += diff + t->bit_time;
        return SF_RESCHEDULE;
    }
    t->timer.waketime += t->cfg_bit_time;
    return SF_RESCHEDULE;
}

void
command_config_tmcuart(uint32_t *args)
{
    struct tmcuart_s *t = oid_alloc(args[0], command_config_tmcuart
                                    , sizeof(*t));
    uint8_t pull_up = args[2];
    uint32_t rx_pin = args[1], tx_pin = args[3];
    t->rx_pin = gpio_in_setup(rx_pin, !!pull_up);
    t->tx_pin = gpio_out_setup(tx_pin, 1);
    t->cfg_bit_time = args[4];
    t->flags = (TU_LINE_HIGH | (pull_up ? TU_PULLUP : 0)
                | (rx_pin == tx_pin ? TU_SINGLE_WIRE : 0));
}
DECL_COMMAND(command_config_tmcuart,
             "config_tmcuart oid=%c rx_pin=%u pull_up=%c"
             " tx_pin=%u bit_time=%u");

// Подготовка обычного или диагностического UART-запроса.
static void
tmcuart_start(struct tmcuart_s *t, const uint8_t *write, uint8_t write_len
              , uint8_t read_len, uint32_t clock)
{
    memcpy(t->data, write, write_len);
    t->pos = 0;
    t->flags = (t->flags & (TU_LINE_HIGH|TU_PULLUP|TU_SINGLE_WIRE)) | TU_ACTIVE;
    t->write_count = write_len * 8;
    t->read_count = read_len * 8;
    if (write_len >= 1 && (t->data[0] & 0x3f) == 0x2a) {
        t->timer.func = tmcuart_send_sync_event;
    } else {
        t->bit_time = t->cfg_bit_time;
        t->timer.func = tmcuart_send_event;
    }
    irq_disable();
    t->timer.waketime = clock;
    sched_add_timer(&t->timer);
    irq_enable();
}

// Parse and schedule a TMC UART transmission request
void
command_tmcuart_send(uint32_t *args)
{
    struct tmcuart_s *t = oid_lookup(args[0], command_config_tmcuart);
#if CONFIG_MACH_N32G455
    if (burst.active && burst.owner == t)
        return;
#endif
    if (t->flags & TU_ACTIVE)
        // Uart is busy - silently drop this request (host should retransmit)
        return;
    uint8_t write_len = args[1];
    uint8_t *write = command_decode_ptr(args[2]);
    uint8_t read_len = args[3];
    if (write_len > sizeof(t->data) || read_len > sizeof(t->data))
        shutdown("tmcuart data too large");
    tmcuart_start(t, write, write_len, read_len,
                  timer_read_time() + timer_from_us(200));
}
DECL_COMMAND(command_tmcuart_send, "tmcuart_send oid=%c write=%*s read=%c");

#if CONFIG_MACH_N32G455
// Разрешено читать только SG_RESULT (0x41), без записи регистров.
// token позволяет безопасно повторить запрос при потере ответа.
void
command_tmcuart_buffer_start(uint32_t *args)
{
    uint8_t oid = args[0], addr = args[2];
    uint16_t token = args[1], count = args[3];
    uint32_t clock = args[4], interval_us = args[5];
    struct tmcuart_s *t = oid_lookup(oid, command_config_tmcuart);
    uint8_t status = 0;
    if (burst.owner == t && burst.token == token)
        goto report;
    if (burst.active || (t->flags & (TU_ACTIVE | TU_REPORT))) {
        status = 1;
        goto report;
    }
    int32_t ahead = clock - timer_read_time();
    if (addr > 3 || !count || count > TU_BUFFER_MAX
        || interval_us < 2000 || interval_us > 20000
        || (uint32_t)count * interval_us > 12000000
        || ahead < (int32_t)timer_from_us(1000)
        || ahead > (int32_t)timer_from_us(2000000)) {
        status = 2;
        goto report;
    }

    burst.data = alloc_optional_scratch(TU_BUFFER_MAX * TU_BUFFER_SAMPLE,
                                        &burst.available);
    if (!burst.data) {
        status = 3;
        goto report;
    }
    tmcuart_sg_request(addr, burst.request);
    burst.owner = t;
    burst.token = token;
    burst.count = count;
    burst.total = burst.errors = 0;
    burst.query_min = UINT32_MAX;
    burst.query_max = 0;
    burst.deadline = clock + timer_from_us(13000000);
    burst.interval = timer_from_us(interval_us);
    burst.next_clock = clock;
    burst.active = 1;
    tmcuart_start(t, burst.request, sizeof(burst.request), 10, clock);
report:
    sendf("tmcuart_buffer_started oid=%c token=%hu status=%c available=%u",
          oid, token, status, burst.available);
}
DECL_COMMAND(command_tmcuart_buffer_start,
             "tmcuart_buffer_start oid=%c token=%hu addr=%c count=%hu"
             " clock=%u interval_us=%u");

// При активном сборе возвращается только статус, буфер не читается.
void
command_tmcuart_buffer_status(uint32_t *args)
{
    uint8_t oid = args[0], status = 2;
    uint16_t token = args[1], total = 0, errors = 0;
    uint32_t query_min = 0, query_max = 0;
    struct tmcuart_s *t = oid_lookup(oid, command_config_tmcuart);
    if (burst.owner == t && burst.token == token) {
        status = burst.active;
        total = burst.total;
        errors = burst.errors;
        if (total) {
            query_min = burst.query_min;
            query_max = burst.query_max;
        }
    }
    sendf("tmcuart_buffer_state oid=%c token=%hu status=%c total=%hu"
          " errors=%hu query_min=%u query_max=%u",
          oid, token, status, total, errors, query_min, query_max);
}
DECL_COMMAND(command_tmcuart_buffer_status,
             "tmcuart_buffer_status oid=%c token=%hu");

// Повторяемое чтение до семи записей после завершения сбора.
void
command_tmcuart_buffer_read(uint32_t *args)
{
    uint8_t oid = args[0], status = 2, len = 0;
    uint16_t token = args[1], first = args[2];
    struct tmcuart_s *t = oid_lookup(oid, command_config_tmcuart);
    uint8_t *data = burst.data;
    if (burst.owner == t && burst.token == token) {
        if (burst.active) {
            status = 1;
        } else if (first < burst.total) {
            uint16_t n = burst.total - first;
            if (n > 7)
                n = 7;
            status = 0;
            len = n * TU_BUFFER_SAMPLE;
            data += first * TU_BUFFER_SAMPLE;
        }
    }
    sendf("tmcuart_buffer_data oid=%c token=%hu first=%hu status=%c data=%*s",
          oid, token, first, status, len, data);
}
DECL_COMMAND(command_tmcuart_buffer_read,
             "tmcuart_buffer_read oid=%c token=%hu first=%hu");
#endif

// Report completed response message back to host
void
tmcuart_task(void)
{
    if (!sched_check_wake(&tmcuart_wake))
        return;
    uint8_t oid;
    struct tmcuart_s *t;
    foreach_oid(oid, t, command_config_tmcuart) {
        if (!(t->flags & TU_REPORT))
            continue;
        irq_disable();
        t->flags &= ~TU_REPORT;
        irq_enable();
#if CONFIG_MACH_N32G455
        if (burst.active && burst.owner == t) {
            // Сохраняем ответ до запуска следующей UART-транзакции.
            uint8_t raw[10], len = t->read_count / 8;
            memcpy(raw, t->data, sizeof(raw));
            uint32_t rx = burst.rx, tx = burst.tx;
            uint32_t query_ticks = burst.end - tx;
            uint16_t index = burst.total;
            uint8_t more = index + 1 < burst.count;
            if (more) {
                uint32_t next = burst.next_clock + burst.interval;
                uint32_t earliest = timer_read_time() + timer_from_us(200);
                if (timer_is_before(next, earliest))
                    next = earliest;
                more = timer_is_before(next, burst.deadline);
                if (more) {
                    burst.next_clock = next;
                    tmcuart_start(t, burst.request, sizeof(burst.request),
                                  10, next);
                }
            }
            // CRC вне таймера. Ошибки сохраняем отдельными записями.
            uint16_t sg = tmcuart_sg_decode(raw, len);
            if (sg == TU_SG_INVALID) {
                burst.errors++;
                rx = tx;
            }
            tmcuart_sg_store(&burst.data[index * TU_BUFFER_SAMPLE], rx, sg);
            if (query_ticks < burst.query_min)
                burst.query_min = query_ticks;
            if (query_ticks > burst.query_max)
                burst.query_max = query_ticks;
            burst.total++;
            if (!more)
                burst.active = 0;
            continue;
        }
#endif
        sendf("tmcuart_response oid=%c read=%*s"
              , oid, t->read_count / 8, t->data);
    }
}
DECL_TASK(tmcuart_task);

void
tmcuart_shutdown(void)
{
#if CONFIG_MACH_N32G455
    burst.active = 0;
    burst.owner = NULL;
#endif
    uint8_t i;
    struct tmcuart_s *t;
    foreach_oid(i, t, command_config_tmcuart) {
        tmcuart_reset_line(t);
    }
}
DECL_SHUTDOWN(tmcuart_shutdown);
