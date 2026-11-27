#ifndef __SG_TEST_IRQ_H
#define __SG_TEST_IRQ_H
typedef unsigned long irqstatus_t;
static inline irqstatus_t irq_save(void) { return 0; }
static inline void irq_restore(irqstatus_t flag) {}
static inline void irq_disable(void) {}
static inline void irq_enable(void) {}
#endif
