// Проверяем настоящий аллокатор MCU на моделируемой области RAM.
#include <assert.h>
#include <stdlib.h>
#include "../src/basecmd.c"

static _Alignas(max_align_t) uint8_t ram[128 * 1024];
static size_t ram_size;

void *dynmem_start(void) { return ram; }
void *dynmem_end(void) { return ram + ram_size; }
uint8_t ctr_lookup_static_string(const char *str) { return 0; }
void sched_shutdown(uint_fast8_t reason) { abort(); }
uint8_t sched_is_shutdown(void) { return 1; }
void sched_timer_reset(void) {}
void sched_clear_shutdown(void) {}

static void
configure(size_t size)
{
    ram_size = size;
    config_reset(NULL);
    alloc_chunk(1024);
    struct move_queue_head q;
    move_queue_setup(&q, 16);
}

int main(void)
{
    const size_t queue_size = 16 * 1024, scratch_size = 36000;
    uint32_t available;
    configure(sizeof(ram));
    void *before = alloc_end;
    assert(!alloc_optional_scratch(scratch_size, &available));
    assert(alloc_end == before);
    move_finalize();
    assert(move_count == 1024);
    before = alloc_end;
    void *scratch = alloc_optional_scratch(scratch_size, &available);
    assert(scratch == before);
    assert(available == sizeof(ram) - 1024 - queue_size);
    assert(move_count == 1024);
    assert(alloc_end == (uint8_t *)scratch + scratch_size);
    assert(alloc_optional_scratch(scratch_size, &available) == scratch);
    assert(alloc_end == (uint8_t *)scratch + scratch_size);
    assert(!alloc_optional_scratch(scratch_size + 1, &available));
    memset(scratch, 0xa5, scratch_size);
    for (unsigned i = 0; i < 1024; i++) {
        void *node = move_alloc();
        assert((uintptr_t)node >= (uintptr_t)move_list);
        assert((uintptr_t)node + 16 <= (uintptr_t)scratch);
    }
    assert(!move_free_list);
    for (unsigned i = 0; i < scratch_size; i++)
        assert(((uint8_t *)scratch)[i] == 0xa5);

    // Точное попадание в остаток RAM и повтор после сброса конфигурации.
    configure(1024 + queue_size + scratch_size);
    assert(!optional_scratch);
    move_finalize();
    assert(alloc_optional_scratch(scratch_size, &available));
    assert(available == scratch_size);
    assert(alloc_end == dynmem_end());
    configure(1024 + queue_size + scratch_size - 1);
    move_finalize();
    before = alloc_end;
    assert(!alloc_optional_scratch(scratch_size, &available));
    assert(available == scratch_size - 1);
    assert(move_count == 1024 && alloc_end == before);

    // Неполная штатная очередь: диагностика не должна запускаться.
    configure(1024 + 100 * 16);
    move_finalize();
    assert(move_count == 100);
    before = alloc_end;
    assert(!alloc_optional_scratch(1, &available));
    assert(alloc_end == before && move_count == 100);
    return 0;
}
